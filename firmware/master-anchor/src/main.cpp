/*
 * =============================================================================
 *  MASTER ANCHOR — CCP Broadcaster (UDP)
 * =============================================================================
 *
 *  Topology: 1 MA + 4 SA + 1 Tag (TDoA 3D positioning)
 *  Role    : MA broadcasts CCP every 150ms; does NOT receive blinks.
 *
 *  Network : All devices on same LAN via router (192.168.10.x subnet)
 *            ESP32 uses static IP, sends UDP to laptop's static IP.
 *            Laptop is wired (Ethernet) to router.
 *
 *  Output (UDP unicast to LAPTOP):
 *    MASTER_CLOCK,<ma_id>,<session_id>,<seq>,<tx_hex_40bit>
 *    HELLO,MA,<ma_id>,<session_id>,<ip>
 *    RESET,MA,<ma_id>,<session_id>,<reason>
 *    HB,MA,<ma_id>,<session_id>,<uptime_s>,<ccp_ok>
 *
 *  EDIT BEFORE FLASH: WIFI_SSID, WIFI_PASS, MA_ID, ANTENNA_DELAY,
 *                     STATIC_IP, LAPTOP_IP, GATEWAY_IP
 * =============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_random.h>
#include <esp_system.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/soc.h>
#include <DW1000.h>

// --- Network ---
const char*    WIFI_SSID        = "indoorpos";      // <-- Router SSID
const char*    WIFI_PASS        = "indoorpos24";       // <-- Router password
const uint16_t UDP_PORT         = 5555;

// --- Static IP Configuration ---
// This device (MA)
IPAddress STATIC_IP  (192, 168, 10, 11);
// Router
IPAddress GATEWAY_IP (192, 168, 10, 1);
IPAddress SUBNET_MASK(255, 255, 255, 0);
IPAddress DNS_IP     (192, 168, 10, 1);
// Laptop (UDP target — wired to router via Ethernet)
IPAddress LAPTOP_IP  (192, 168, 10, 100);

// --- UWB ---
const uint8_t  MA_ID            = 1;
const uint8_t  NETWORK_ID       = 10;
const uint32_t CCP_INTERVAL_MS  = 30;
const uint32_t TX_WAIT_MS       = 100;
uint16_t       ANTENNA_DELAY    = 16384;

// --- Pins (ESP32-WROOM-32) ---
#define EN_UWB     25
#define SPI_SCK    14
#define SPI_MISO   12
#define SPI_MOSI   13
const uint8_t PIN_RST = 26;
const uint8_t PIN_IRQ = 27;
const uint8_t PIN_SS  = 15;

// --- Packet structure ---
struct __attribute__((packed)) CcpPacket {
  uint8_t  frameType;     // 0xB0
  uint8_t  maId;
  uint32_t seq;
  uint8_t  tk_bytes[5];
};

// --- State ---
uint16_t  sessionId   = 0;
volatile bool sent    = false;
CcpPacket ccpPacket;
uint32_t  ccpSeq      = 0;
uint32_t  txOk        = 0;
uint32_t  txTimeout   = 0;
int64_t   lastTxTs    = 0;
uint32_t  nextCcpMs   = 0;
uint32_t  lastReportMs = 0;
uint32_t  lastHbMs    = 0;
uint32_t  bootMs      = 0;
WiFiUDP   udp;

void IRAM_ATTR handleSent() { sent = true; }

void connectWiFi() {
  Serial.print("Connecting WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  // Set static IP BEFORE WiFi.begin()
  if (!WiFi.config(STATIC_IP, GATEWAY_IP, SUBNET_MASK, DNS_IP)) {
    Serial.println("\n[WARN] Static IP config failed!");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    if (millis() - t0 > 20000) {
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      t0 = millis();
    }
  }
  Serial.printf("\nConnected. local=%s gw=%s target=%s rssi=%d\n",
                WiFi.localIP().toString().c_str(),
                WiFi.gatewayIP().toString().c_str(),
                LAPTOP_IP.toString().c_str(), WiFi.RSSI());
  udp.begin(UDP_PORT);
}

inline void udpSend(const char* line, uint16_t len) {
  udp.beginPacket(LAPTOP_IP, UDP_PORT);       // <-- Send to LAPTOP, not gateway
  udp.write((const uint8_t*)line, len);
  udp.endPacket();
}

inline void hex40(const byte ts[5], char out[11]) {
  snprintf(out, 11, "%02X%02X%02X%02X%02X",
           ts[4], ts[3], ts[2], ts[1], ts[0]);
}

void broadcastReset(const char* reason) {
  char line[80];
  int len = snprintf(line, sizeof(line), "RESET,MA,%u,%u,%s",
                     MA_ID, sessionId, reason);
  for (int i = 0; i < 5; i++) { udpSend(line, len); delay(20); }
  Serial.println(line);
}

void broadcastHello() {
  char line[80];
  int len = snprintf(line, sizeof(line), "HELLO,MA,%u,%u,%s",
                     MA_ID, sessionId, WiFi.localIP().toString().c_str());
  for (int i = 0; i < 3; i++) { udpSend(line, len); delay(20); }
  Serial.println(line);
}

void sendHeartbeat() {
  char line[80];
  uint32_t up_s = (millis() - bootMs) / 1000;
  int len = snprintf(line, sizeof(line), "HB,MA,%u,%u,%u,%u",
                     MA_ID, sessionId, up_s, txOk);
  udpSend(line, len);
}

void setup() {
  // Disable brownout — TX current spike won't reset us
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  pinMode(EN_UWB, OUTPUT);
  digitalWrite(EN_UWB, HIGH);
  delay(100);

  Serial.begin(115200);
  delay(500);

  esp_reset_reason_t reason = esp_reset_reason();
  const char* reason_str =
      (reason == ESP_RST_POWERON)  ? "power_on"  :
      (reason == ESP_RST_SW)       ? "software"  :
      (reason == ESP_RST_PANIC)    ? "panic"     :
      (reason == ESP_RST_INT_WDT)  ? "int_wdt"   :
      (reason == ESP_RST_TASK_WDT) ? "task_wdt"  :
      (reason == ESP_RST_BROWNOUT) ? "brownout"  : "unknown";

  sessionId = (uint16_t)(esp_random() & 0xFFFF);
  if (sessionId == 0) sessionId = 1;
  bootMs = millis();

  Serial.println("### MASTER ANCHOR (CCP only) ###");
  Serial.printf("MA_ID=%u  SESSION=%u  RESET=%s  ANT_DELAY=%u\n",
                MA_ID, sessionId, reason_str, ANTENNA_DELAY);
  Serial.printf("STATIC_IP=%s  LAPTOP=%s  GW=%s\n",
                STATIC_IP.toString().c_str(),
                LAPTOP_IP.toString().c_str(),
                GATEWAY_IP.toString().c_str());

  connectWiFi();
  broadcastReset(reason_str);
  delay(100);
  broadcastHello();
  delay(100);

  // DW1000 init
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);  delay(50);
  digitalWrite(PIN_RST, HIGH); delay(500);

  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(MA_ID);
  DW1000.setNetworkId(NETWORK_ID);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.setAntennaDelay(ANTENNA_DELAY);
  DW1000.commitConfiguration();

  DW1000.attachSentHandler(handleSent);
  DW1000.enableDebounceClock();
  DW1000.enableLedBlinking();
  DW1000.setGPIOMode(MSGP0, LED_MODE);

  Serial.println("Ready. CCP every 150ms.");
  nextCcpMs = millis();
  lastReportMs = millis();
  lastHbMs = millis();
}

void transmitCcp() {
  sent = false;
  ccpPacket.frameType = 0xB0;
  ccpPacket.maId      = MA_ID;
  ccpPacket.seq       = ccpSeq++;
  memset(ccpPacket.tk_bytes, 0, 5);

  DW1000.newTransmit();
  DW1000.setDefaults();
  DW1000.setData((byte*)&ccpPacket, sizeof(ccpPacket));
  DW1000.startTransmit();

  uint32_t start = millis();
  while (!sent && (millis() - start) < TX_WAIT_MS) yield();

  if (sent) {
    txOk++;
    DW1000Time txActual;
    DW1000.getTransmitTimestamp(txActual);
    int64_t ts = txActual.getTimestamp();

    byte tsBytes[5];
    txActual.getTimestamp(tsBytes);
    char txHex[11];
    hex40(tsBytes, txHex);

    char line[96];
    int len = snprintf(line, sizeof(line), "MASTER_CLOCK,%u,%u,%u,%s",
                       MA_ID, sessionId, ccpPacket.seq, txHex);
    udpSend(line, len);

    if (ts == lastTxTs && txOk > 1) {
      Serial.printf("# WARN_SAME_TS seq=%u\n", ccpPacket.seq);
    }
    lastTxTs = ts;

    if ((ccpPacket.seq % 50) == 0) Serial.println(line);
  } else {
    txTimeout++;
    Serial.printf("# TX_TIMEOUT seq=%u\n", ccpPacket.seq);
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("# WiFi lost, reconnecting...");
    connectWiFi();
    broadcastHello();
  }

  uint32_t now = millis();
  if ((int32_t)(now - nextCcpMs) >= 0) {
    nextCcpMs += CCP_INTERVAL_MS;
    transmitCcp();
  }

  if ((millis() - lastHbMs) >= 5000) {
    lastHbMs = millis();
    sendHeartbeat();
  }

  if ((millis() - lastReportMs) >= 10000) {
    lastReportMs = millis();
    uint32_t total = txOk + txTimeout;
    if (total > 0) {
      Serial.printf("# STATS sess=%u up=%us ccp_ok=%u fail=%u rate=%.1f%% rssi=%d\n",
                    sessionId, (millis() - bootMs) / 1000,
                    txOk, txTimeout, 100.0f * txOk / total, WiFi.RSSI());
    }
  }

  yield();
}
