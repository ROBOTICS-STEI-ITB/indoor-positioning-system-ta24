/*
 * =============================================================================
 *  TAG — UWB Blink + BNO055 IMU Telemetry (Dual-Core FreeRTOS, headless)
 * =============================================================================
 *
 *  Topology: 1 MA + 4 SA + 1 Tag (TDoA 3D positioning)
 *
 *  Architecture (mirrors MA/SA dual-core pattern):
 *    Core 1 (App CPU)  : taskUWB — UWB blink TX @ 25 Hz. Jitter-free.
 *    Core 0 (PRO CPU)  : taskIMU — WiFi + BNO055 read + UDP TX @ 20 Hz.
 *                         IMU lives here because its data leaves via UDP,
 *                         and WiFi stack runs natively on Core 0.
 *
 *  Headless design: NO serial logging. Final product is wireless.
 *    Diagnostics are sent over UDP instead (like MA's HELLO/HB):
 *      HELLO,TAG,<id>,<ip>                         (on connect)
 *      STAT,TAG,<id>,<blink_ok>,<blink_to>,<imu_sent>,<rssi>,<calib>
 *
 *  IMU UDP packet (seq + timestamp + blink anchor for loss/jitter/alignment):
 *      $,<seq>,<ms>,<blink>,YAW,PITCH,ROLL,QW,QX,QY,QZ,GX,GY,GZ,AX,AY,AZ
 *    <blink> = latest UWB blink number (lets the host time-align IMU samples
 *    with UWB-derived position by blink #). Euler (deg) + Quaternion (native
 *    fusion) + gyro (rad/s) + linear accel (m/s², gravity removed). Remapped frame.
 *
 *  EDIT BEFORE FLASH: WIFI_SSID, WIFI_PASS, TAG_ID, network IPs
 * =============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_system.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/soc.h>
#include <DW1000.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ===========================================================================
// CONFIG
// ===========================================================================
const uint8_t  TAG_ID          = 1;
const uint8_t  NETWORK_ID      = 10;
const int      BLINK_RATE_HZ   = 25;   // UWB blink rate
const uint16_t IMU_INTERVAL_MS = 40;   // 25 Hz IMU/UDP
const uint16_t TX_WAIT_MS      = 30;   // TX-complete timeout (safe at 25Hz)
const uint32_t STAT_INTERVAL_MS = 5000;

uint16_t ANTENNA_DELAY = 16384;

// ===========================================================================
// WIFI CONFIG
// ===========================================================================
const char* WIFI_SSID = "indoorpos";
const char* WIFI_PASS = "indoorpos24";

const uint16_t UDP_PORT = 5555;

IPAddress STATIC_IP  (192, 168, 10, 10); //192, 168, 10, 10
IPAddress GATEWAY_IP (192, 168, 10, 1);  //192, 168, 10, 1
IPAddress SUBNET_MASK(255, 255, 255, 0);  //255, 255, 255, 0
IPAddress DNS_IP     (192, 168, 10, 1);  //192, 168, 10, 1
IPAddress LAPTOP_IP  (192, 168, 10, 100);  //192, 168, 10, 100

// ===========================================================================
// PINOUT
// ===========================================================================
// UWB
#define EN_UWB     7
#define SPI_SCK    36
#define SPI_MISO   37
#define SPI_MOSI   35
#define DW_CS      34

const uint8_t PIN_RST = 38;
const uint8_t PIN_IRQ = 33;
const uint8_t PIN_SS  = 34;

// BNO055
#define SDA_PIN     39
#define SCL_PIN     40
#define NRESET_PIN  41

// ===========================================================================
// BNO055 CALIBRATION OFFSETS
// ===========================================================================
const int16_t ACCEL_OFFSET_X = 0;
const int16_t ACCEL_OFFSET_Y = 0;
const int16_t ACCEL_OFFSET_Z = 0;

const int16_t GYRO_OFFSET_X  = 16334;
const int16_t GYRO_OFFSET_Y  = 0;
const int16_t GYRO_OFFSET_Z  = 0;

const int16_t MAG_OFFSET_X   = 0;
const int16_t MAG_OFFSET_Y   = 0;
const int16_t MAG_OFFSET_Z   = -11296;

const int16_t ACCEL_RADIUS   = 0;
const int16_t MAG_RADIUS     = 0;

// ===========================================================================
// BNO055 AXIS REMAP — mounted standing (rotated 90° about X, chip +Y down)
// Validated values: maps physical Y (vertical) -> fusion Z, so upright reads
// pitch~0/roll~0. Applied via the library (raw register writes failed on this
// board). See test_bno_remap.cpp for the validation.
// ===========================================================================
const uint8_t REMAP_CONFIG = 0x18;   // fusion Z<-chipY, Y<-chipZ, X<-chipX
const uint8_t REMAP_SIGN   = 0x01;   // Z negative (chip +Y points down)

// BNO055 register addresses used to READ BACK and verify the remap landed
// (writes go through the library; raw writes were unreliable on this board).
#define BNO055_I2C_ADDR        0x29
#define REG_AXIS_MAP_CONFIG    0x41
#define REG_AXIS_MAP_SIGN      0x42

// ===========================================================================
// UWB PACKET
// ===========================================================================
struct __attribute__((packed)) BlinkPacket {
  uint8_t  frameType;
  uint8_t  tagId;
  uint32_t seq;
};

// ===========================================================================
// GLOBAL STATE
// ===========================================================================
volatile boolean txDone = false;

BlinkPacket blinkPacket;
volatile uint32_t blinkSeq     = 0;   // written Core1, also read Core0 (anchor)
volatile uint32_t blinkOk      = 0;   // written Core1, read Core0 (atomic u32)
volatile uint32_t blinkTimeout = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
WiFiUDP udp;

uint32_t imuSeq        = 0;
volatile uint32_t imuSent = 0;
bool wifiReady         = false;
bool bnoReady          = false;

TaskHandle_t hTaskUWB = NULL;
TaskHandle_t hTaskIMU = NULL;

// ===========================================================================
// ISR
// ===========================================================================
void IRAM_ATTR onTxDone() {
  txDone = true;
}

// ===========================================================================
// UDP HELPER (diagnostics — sent from Core 0 only)
// ===========================================================================
inline void udpSend(const char* line, uint16_t len) {
  udp.beginPacket(LAPTOP_IP, UDP_PORT);
  udp.write((const uint8_t*)line, len);
  udp.endPacket();
}

// ===========================================================================
// RESET BNO055
// ===========================================================================
void resetBNO055() {
  pinMode(NRESET_PIN, OUTPUT);
  digitalWrite(NRESET_PIN, LOW);
  delay(10);
  digitalWrite(NRESET_PIN, HIGH);
  delay(700);
}

// ===========================================================================
// READ ONE BNO055 REGISTER (STOP-based; repeated-start was unreliable here)
// ===========================================================================
uint8_t readBnoReg(uint8_t reg) {
  Wire.beginTransmission(BNO055_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(true) != 0) return 0xFF;
  if (Wire.requestFrom((int)BNO055_I2C_ADDR, 1) != 1) return 0xFF;
  return Wire.read();
}

// ===========================================================================
// APPLY BNO055 AXIS REMAP (retry until verified)
// Uses the library's setAxisRemap/setAxisSign (proven write path), then reads
// the registers back to confirm. The I2C writes are flaky on this board (took
// 3 tries in testing), so retry until readback matches. Headless: no serial,
// result reflected in gRemapOk (surfaced via STAT over UDP).
// ===========================================================================
bool gRemapOk = false;

void applyRemap() {
  for (int attempt = 0; attempt < 10; attempt++) {
    bno.setAxisRemap((Adafruit_BNO055::adafruit_bno055_axis_remap_config_t)REMAP_CONFIG);
    bno.setAxisSign((Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t)REMAP_SIGN);
    delay(20);
    uint8_t cfg  = readBnoReg(REG_AXIS_MAP_CONFIG);
    uint8_t sign = readBnoReg(REG_AXIS_MAP_SIGN);
    gRemapOk = (cfg == REMAP_CONFIG && sign == REMAP_SIGN);
    if (gRemapOk) break;
    delay(20);
  }
}

// ===========================================================================
// APPLY BNO055 CALIBRATION
// ===========================================================================
void applyCalibration() {
  adafruit_bno055_offsets_t offsets;

  offsets.accel_offset_x = ACCEL_OFFSET_X;
  offsets.accel_offset_y = ACCEL_OFFSET_Y;
  offsets.accel_offset_z = ACCEL_OFFSET_Z;

  offsets.gyro_offset_x  = GYRO_OFFSET_X;
  offsets.gyro_offset_y  = GYRO_OFFSET_Y;
  offsets.gyro_offset_z  = GYRO_OFFSET_Z;

  offsets.mag_offset_x   = MAG_OFFSET_X;
  offsets.mag_offset_y   = MAG_OFFSET_Y;
  offsets.mag_offset_z   = MAG_OFFSET_Z;

  offsets.accel_radius   = ACCEL_RADIUS;
  offsets.mag_radius     = MAG_RADIUS;

  bno.setMode(OPERATION_MODE_CONFIG);
  delay(25);
  bno.setSensorOffsets(offsets);
  bno.setMode(OPERATION_MODE_NDOF);   // NDOF required for LINEARACCEL fusion
  delay(20);
}

// ===========================================================================
// CONNECT WIFI (Core 0)
// ===========================================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  WiFi.config(STATIC_IP, GATEWAY_IP, SUBNET_MASK, DNS_IP);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(250));
    if (millis() - t0 > 20000) {
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      t0 = millis();
    }
  }

  udp.begin(UDP_PORT);
  vTaskDelay(pdMS_TO_TICKS(100));   // let lwIP allocate TX buffers (avoid err 12)
  wifiReady = true;

  // Announce presence (mirrors HELLO,SA pattern)
  char line[64];
  int len = snprintf(line, sizeof(line), "HELLO,TAG,%u,%s",
                     TAG_ID, WiFi.localIP().toString().c_str());
  udpSend(line, len);
}

// ===========================================================================
// TASK UWB — CORE 1  (25 Hz blink, jitter-free)
// ===========================================================================
void taskUWB(void* parameter) {
  const TickType_t blinkPeriod = pdMS_TO_TICKS(1000 / BLINK_RATE_HZ);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    txDone = false;

    DW1000.newTransmit();
    DW1000.setDefaults();

    blinkPacket.frameType = 0xA0;
    blinkPacket.tagId     = TAG_ID;
    blinkPacket.seq       = blinkSeq++;

    DW1000.setData((byte*)&blinkPacket, sizeof(blinkPacket));
    DW1000.startTransmit();

    uint32_t t0 = millis();
    while (!txDone && (millis() - t0) < TX_WAIT_MS) {
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (txDone) blinkOk++;
    else        blinkTimeout++;

    vTaskDelayUntil(&lastWake, blinkPeriod);
  }
}

// ===========================================================================
// TASK IMU — CORE 0  (WiFi + BNO055 + UDP @ 20 Hz)
// ===========================================================================
void taskIMU(void* parameter) {
  connectWiFi();

  vTaskDelay(pdMS_TO_TICKS(200));

  resetBNO055();
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);        // 100kHz for I2C reliability

  if (bno.begin()) {
    bno.setExtCrystalUse(true);
    applyRemap();               // axis remap for 90°-standing mount (chip +Y down)
    applyCalibration();
    bnoReady = true;
  } else {
    bnoReady = false;
  }

  const TickType_t imuPeriod = pdMS_TO_TICKS(IMU_INTERVAL_MS);
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastStatMs = millis();

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      wifiReady = false;
      connectWiFi();
      lastWake = xTaskGetTickCount();   // reset to avoid catch-up burst
    }

    if (bnoReady && wifiReady) {
      imu::Vector<3> euler    = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      imu::Vector<3> gyro     = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      imu::Quaternion quat    = bno.getQuat();   // native fusion output
      uint32_t blinkAnchor    = blinkSeq;        // UWB blink counter (Core 1) for time-alignment

      // $,seq,ms,blink,YAW,PITCH,ROLL,QW,QX,QY,QZ,GX,GY,GZ,AX,AY,AZ
      char packet[200];
      int len = snprintf(packet, sizeof(packet),
               "$,%u,%lu,%u,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
               imuSeq++, (unsigned long)millis(), blinkAnchor,
               euler.x(),       // YAW
               euler.z(),       // PITCH
               euler.y(),       // ROLL
               quat.w(),        // QW
               quat.x(),        // QX
               quat.y(),        // QY
               quat.z(),        // QZ
               gyro.x(),        // GX
               gyro.y(),        // GY
               gyro.z(),        // GZ
               linAccel.x(),    // AX
               linAccel.y(),    // AY
               linAccel.z()     // AZ
      );

      udpSend(packet, len);
      imuSent++;
    }

    // Periodic status over UDP (replaces serial stats)
    if (millis() - lastStatMs >= STAT_INTERVAL_MS) {
      lastStatMs = millis();

      uint8_t cs = 0, cg = 0, ca = 0, cm = 0;
      if (bnoReady) bno.getCalibration(&cs, &cg, &ca, &cm);

      char line[96];
      int len = snprintf(line, sizeof(line),
                "STAT,TAG,%u,%u,%u,%u,%d,%u%u%u%u",
                TAG_ID, blinkOk, blinkTimeout, imuSent,
                wifiReady ? WiFi.RSSI() : 0,
                cs, cg, ca, cm);
      udpSend(line, len);
    }

    vTaskDelayUntil(&lastWake, imuPeriod);
  }
}

// ===========================================================================
// SETUP (Core 1)
// ===========================================================================
void setup() {
  // Disable brownout — WiFi + UWB TX current spikes won't reset us
  // (same as MA firmware)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // ---------- UWB INIT (before tasks start) ----------
  pinMode(EN_UWB, OUTPUT);
  digitalWrite(EN_UWB, HIGH);
  delay(100);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);  delay(50);
  digitalWrite(PIN_RST, HIGH); delay(500);

  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);

  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(TAG_ID);
  DW1000.setNetworkId(NETWORK_ID);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.setAntennaDelay(ANTENNA_DELAY);
  DW1000.commitConfiguration();

  DW1000.attachSentHandler(onTxDone);

  DW1000.enableDebounceClock();
  DW1000.enableLedBlinking();
  DW1000.setGPIOMode(MSGP0, LED_MODE);

  // ---------- FREERTOS TASKS ----------
  xTaskCreatePinnedToCore(
    taskUWB, "taskUWB", 8192, NULL, 3, &hTaskUWB, 1);   // Core 1, prio 3

  xTaskCreatePinnedToCore(
    taskIMU, "taskIMU", 8192, NULL, 2, &hTaskIMU, 0);   // Core 0, prio 2
}

// ===========================================================================
// LOOP — empty; all work runs in FreeRTOS tasks
// ===========================================================================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}