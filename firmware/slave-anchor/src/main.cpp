/*
 * =============================================================================
 *  SLAVE ANCHOR — CCP + Tag Blink Receiver (Dual-Core FreeRTOS, UDP)
 * =============================================================================
 *
 *  Topology: 1 MA + 4 SA + 1 Tag (TDoA 3D positioning)
 *
 *  Network : All devices on same LAN via router (192.168.10.x subnet)
 *            ESP32 uses static IP, sends UDP to laptop's static IP.
 *            Laptop is wired (Ethernet) to router.
 *
 *  Architecture (FreeRTOS dual-core):
 *    Core 1 (App CPU)  : taskUWB — DW1000 RX handler. Reads packet + RX
 *                         timestamp ASAP, formats line, pushes to queue.
 *                         NEVER blocks on WiFi/UDP.
 *    Core 0 (PRO CPU)  : taskUDP — pops from queue, sends UDP packet.
 *                         May block briefly during WiFi congestion, but
 *                         RX timing is not affected.
 *
 *  Why split:
 *    - udp.beginPacket()/endPacket() can block 3-15ms under WiFi load.
 *    - Single-core SA: while UDP blocks, the next CCP/blink RX flag is set
 *      but not handled in time → rxBuffer can be overwritten by hardware
 *      before software reads it → silent packet loss.
 *    - Dual-core: Core 1 services RX immediately (sub-millisecond), Core 0
 *      handles WiFi blocking independently. Queue absorbs backlog.
 *
 *  Output (UDP unicast to LAPTOP):
 *    <sa_id>,MASTER,<ma_id>,<seq>,<tk_hex>,<rx_hex>      (CCP reception)
 *    <sa_id>,TAG,<tag_id>,<seq>,NA,<rx_hex>              (blink reception)
 *    HELLO,SA,<sa_id>,<ip>
 *
 *  EDIT BEFORE FLASH per unit: WIFI_SSID, WIFI_PASS, SA_ID, ANTENNA_DELAY,
 *                              STATIC_IP (unique per unit!)
 * =============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <DW1000.h>

// --- Network ---
const char*    WIFI_SSID        = "indoorpos";      // <-- Router SSID
const char*    WIFI_PASS        = "indoorpos24";       // <-- Router password
const uint16_t UDP_PORT         = 5555;

// --- Static IP Configuration ---
// ┌──────────┬───────────────────┐
// │  SA_ID   │  STATIC_IP        │
// ├──────────┼───────────────────┤
// │  2       │  192.168.10.12    │
// │  3       │  192.168.10.13    │
// │  4       │  192.168.10.14    │
// │  5       │  192.168.10.15    │
// └──────────┴───────────────────┘

// This device (EDIT PER UNIT — must match SA_ID)
IPAddress STATIC_IP  (192, 168, 10, 15);     // <-- Change per unit!
// Router
IPAddress GATEWAY_IP (192, 168, 10, 1);
IPAddress SUBNET_MASK(255, 255, 255, 0);
IPAddress DNS_IP     (192, 168, 10, 1);
// Laptop (UDP target — wired to router via Ethernet)
IPAddress LAPTOP_IP  (192, 168, 10, 100);

// --- UWB (EDIT PER UNIT) ---
const uint8_t  SA_ID            = 5;       // 2, 3, 4, or 5
const uint8_t  NETWORK_ID       = 10;
uint16_t       ANTENNA_DELAY    = 16440;   // calibrate per unit

// --- Antenna Delay (EDIT PER UNIT) ---
// ┌──────────┬───────────────────┐
// │  SA_ID   │  ANTENNA_DELAY    │
// ├──────────┼───────────────────┤
// │  2       │      16384        │
// │  3       │      16428        │
// │  4       │      16404        │
// │  5       │      16440        │
// └──────────┴───────────────────┘

// --- Pins (ESP32-WROOM-32) ---
#define EN_UWB     25
#define SPI_SCK    14
#define SPI_MISO   12
#define SPI_MOSI   13
const uint8_t PIN_RST = 26;
const uint8_t PIN_IRQ = 27;
const uint8_t PIN_SS  = 15;

// --- Packet structures ---
struct __attribute__((packed)) CcpPacket {
  uint8_t  frameType;     // 0xB0
  uint8_t  maId;
  uint32_t seq;
  uint8_t  tk_bytes[5];
};

struct __attribute__((packed)) BlinkPacket {
  uint8_t  frameType;     // 0xA0
  uint8_t  tagId;
  uint32_t seq;
};

// =============================================================================
// QUEUE MESSAGE
// =============================================================================
// Fixed-size struct (avoid String/dynamic alloc → safe for queue copy)
// 96 bytes covers any output line we generate.
struct UdpMsg {
  uint16_t len;
  char     line[96];
};

// Queue depth — 32 messages.
// At 5Hz blink + 6.6Hz CCP ≈ 12 msg/s, 32 slots = ~2.6s buffer.
// More than enough to absorb a few hundred ms of WiFi stalls.
const int UDP_QUEUE_DEPTH = 32;
QueueHandle_t udpQueue = NULL;

// =============================================================================
// STATE
// =============================================================================
volatile bool rxReady = false;
volatile bool rxError = false;
byte     rxBuffer[32];

// Stats (read by both tasks — atomic uint32 reads on ESP32 are safe)
volatile uint32_t ccpCount   = 0;
volatile uint32_t blinkCount = 0;
volatile uint32_t udpSent    = 0;
volatile uint32_t udpDropped = 0;   // queue full — RX faster than UDP can drain
volatile uint32_t rxErrors   = 0;

WiFiUDP udp;
bool    wifiReady = false;

// FreeRTOS handles
TaskHandle_t hTaskUWB = NULL;
TaskHandle_t hTaskUDP = NULL;

// =============================================================================
// ISRs
// =============================================================================
void IRAM_ATTR onRx()       { rxReady = true; }
void IRAM_ATTR onRxError()  { rxError = true; }

// =============================================================================
// HELPERS
// =============================================================================
inline void hex40(const byte ts[5], char out[11]) {
  snprintf(out, 11, "%02X%02X%02X%02X%02X",
           ts[4], ts[3], ts[2], ts[1], ts[0]);
}

// Push to UDP queue. Non-blocking (timeout=0). If queue full, drop & count.
inline void enqueueUdp(const char* line, uint16_t len) {
  UdpMsg msg;
  if (len >= sizeof(msg.line)) len = sizeof(msg.line) - 1;
  memcpy(msg.line, line, len);
  msg.line[len] = '\0';
  msg.len = len;

  if (xQueueSend(udpQueue, &msg, 0) != pdTRUE) {
    udpDropped++;
  }
}

// =============================================================================
// CONNECT WIFI (called from Core 0 / UDP task)
// =============================================================================
void connectWiFi() {
  Serial.print("[WiFi] Connecting");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (!WiFi.config(STATIC_IP, GATEWAY_IP, SUBNET_MASK, DNS_IP)) {
    Serial.println("\n[WiFi] Static IP config failed!");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(250));
    Serial.print(".");
    if (millis() - t0 > 20000) {
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      t0 = millis();
    }
  }
  Serial.printf("\n[WiFi] Connected. local=%s target=%s rssi=%d\n",
                WiFi.localIP().toString().c_str(),
                LAPTOP_IP.toString().c_str(),
                WiFi.RSSI());
  udp.begin(UDP_PORT);
  wifiReady = true;
}

// =============================================================================
// CORE 1 TASK: UWB RX HANDLER
//
// Pinned to Core 1. Polls rxReady flag set by ISR, reads packet + RX
// timestamp from DW1000, formats output line, pushes to queue. NEVER
// touches WiFi/UDP.
//
// Tight loop (1ms tick) — ensures no RX is missed even if UDP task stalls.
// =============================================================================
void taskUWB(void* parameter) {
  Serial.printf("[taskUWB] running on core %d\n", xPortGetCoreID());

  for (;;) {
    if (rxReady) {
      rxReady = false;

      uint16_t dataLen = DW1000.getDataLength();
      if (dataLen > sizeof(rxBuffer)) dataLen = sizeof(rxBuffer);
      DW1000.getData(rxBuffer, dataLen);

      DW1000Time rxTime;
      DW1000.getReceiveTimestamp(rxTime);
      byte rx_bytes[5];
      rxTime.getTimestamp(rx_bytes);
      char rxHex[11];
      hex40(rx_bytes, rxHex);

      uint8_t frameType = rxBuffer[0];
      char line[96];
      int len = 0;

      if (frameType == 0xB0 && dataLen == sizeof(CcpPacket)) {
        // CCP from MA
        CcpPacket* ccp = (CcpPacket*)rxBuffer;
        char tkHex[11];
        hex40(ccp->tk_bytes, tkHex);
        len = snprintf(line, sizeof(line), "%u,MASTER,%u,%u,%s,%s",
                       SA_ID, ccp->maId, ccp->seq, tkHex, rxHex);
        enqueueUdp(line, len);
        ccpCount++;
        if ((ccpCount % 50) == 0) Serial.println(line);
      }
      else if (frameType == 0xA0 && dataLen == sizeof(BlinkPacket)) {
        // Blink from Tag
        BlinkPacket* blink = (BlinkPacket*)rxBuffer;
        len = snprintf(line, sizeof(line), "%u,TAG,%u,%u,NA,%s",
                       SA_ID, blink->tagId, blink->seq, rxHex);
        enqueueUdp(line, len);
        blinkCount++;
        if ((blinkCount % 50) == 0) Serial.println(line);
      }
    }

    if (rxError) {
      rxError = false;
      rxErrors++;
    }

    // 1ms tick — keeps RX latency well below blink/CCP intervals (≥150ms)
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =============================================================================
// CORE 0 TASK: UDP SENDER
//
// Pinned to Core 0 (where WiFi stack natively runs → no IPC overhead).
// Blocks on xQueueReceive until message arrives, then sends UDP.
// May block on udp.endPacket() during WiFi congestion — that's fine,
// taskUWB on Core 1 keeps servicing RX.
// =============================================================================
void taskUDP(void* parameter) {
  Serial.printf("[taskUDP] running on core %d\n", xPortGetCoreID());

  // Connect WiFi first
  connectWiFi();

  // Send HELLO once WiFi is up
  {
    char line[64];
    int len = snprintf(line, sizeof(line), "HELLO,SA,%u,%s",
                       SA_ID, WiFi.localIP().toString().c_str());
    udp.beginPacket(LAPTOP_IP, UDP_PORT);
    udp.write((const uint8_t*)line, len);
    udp.endPacket();
    Serial.println(line);
  }

  uint32_t lastReportMs = millis();
  UdpMsg   msg;

  for (;;) {
    // WiFi watchdog
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] lost, reconnecting...");
      wifiReady = false;
      connectWiFi();
    }

    // Wait up to 1s for next message
    if (xQueueReceive(udpQueue, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {
      udp.beginPacket(LAPTOP_IP, UDP_PORT);
      udp.write((const uint8_t*)msg.line, msg.len);
      udp.endPacket();
      udpSent++;
    }

    // Stats every 10s
    if ((millis() - lastReportMs) >= 10000) {
      lastReportMs = millis();
      UBaseType_t qDepth = uxQueueMessagesWaiting(udpQueue);
      Serial.printf("# STATS ccp=%u blink=%u udp_sent=%u dropped=%u "
                    "rx_err=%u q=%u rssi=%d\n",
                    ccpCount, blinkCount, udpSent, udpDropped,
                    rxErrors, (unsigned)qDepth, WiFi.RSSI());
    }
  }
}

// =============================================================================
// SETUP (runs on Core 1 by default — same core as taskUWB)
// =============================================================================
void setup() {
  pinMode(EN_UWB, OUTPUT);
  digitalWrite(EN_UWB, HIGH);
  delay(100);

  Serial.begin(115200);
  delay(500);

  Serial.println("\n### SLAVE ANCHOR (Dual-Core) ###");
  Serial.printf("SA_ID=%u  ANT_DELAY=%u  UDP=%u\n",
                SA_ID, ANTENNA_DELAY, UDP_PORT);
  Serial.printf("STATIC_IP=%s  LAPTOP=%s  GW=%s\n",
                STATIC_IP.toString().c_str(),
                LAPTOP_IP.toString().c_str(),
                GATEWAY_IP.toString().c_str());

  // ---------- DW1000 INIT (must complete before tasks start) ----------
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);  delay(50);
  digitalWrite(PIN_RST, HIGH); delay(500);

  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(SA_ID);
  DW1000.setNetworkId(NETWORK_ID);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.setAntennaDelay(ANTENNA_DELAY);
  DW1000.commitConfiguration();

  DW1000.attachReceivedHandler(onRx);
  DW1000.attachReceiveFailedHandler(onRxError);
  DW1000.attachErrorHandler(onRxError);

  DW1000.enableDebounceClock();
  DW1000.enableLedBlinking();
  DW1000.setGPIOMode(MSGP0, LED_MODE);

  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();

  Serial.println("[UWB] DW1000 init OK. Listening for CCP + blinks.");

  // ---------- FREERTOS QUEUE + TASKS ----------
  udpQueue = xQueueCreate(UDP_QUEUE_DEPTH, sizeof(UdpMsg));
  if (udpQueue == NULL) {
    Serial.println("[FATAL] Queue create failed!");
    while (true) delay(1000);
  }

  // taskUWB on Core 1 — RX handler, must be jitter-free
  xTaskCreatePinnedToCore(
    taskUWB,
    "taskUWB",
    8192,
    NULL,
    3,           // priority — higher than UDP task
    &hTaskUWB,
    1            // core 1
  );

  // taskUDP on Core 0 — WiFi/UDP send (Core 0 hosts WiFi stack natively)
  xTaskCreatePinnedToCore(
    taskUDP,
    "taskUDP",
    8192,
    NULL,
    2,           // priority
    &hTaskUDP,
    0            // core 0
  );

  Serial.println("=========================================");
  Serial.println("Tasks spawned. Setup done.");
}

// =============================================================================
// LOOP — empty; all work runs in FreeRTOS tasks
// =============================================================================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}