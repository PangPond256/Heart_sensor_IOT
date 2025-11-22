#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h"
#include <math.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- Sensors ----------
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX30105 maxSensor;

// ---------- Pins ----------
#define PIN_BUZZER 2   // Buzzer

// ---------- BLE ----------
#define BLE_DEV_NAME "HeartSense-ESP32"
#define BLE_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_NOTIFY_CHAR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CMD_CHAR_UUID     "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *server = nullptr;
BLECharacteristic *notifyChar = nullptr;
BLECharacteristic *cmdChar = nullptr;
bool bleConnected = false;

// ---------- HR / TEMP ----------
float avgBPM     = 75.0f;
float smoothTemp = 36.5f;
float bodyTemp   = 36.5f;
unsigned long lastSendMs = 0;

// ---------- HR Alert ----------
#define HR_LIMIT_TACHY 100
#define HR_LIMIT_HIGH  120

// ---------- Fever Threshold ----------
#define FEVER_THRESHOLD 37.5
bool feverBeeped = false;

// ---------- Buzzer ----------
void buzzerOn()  { digitalWrite(PIN_BUZZER, LOW); }
void buzzerOff() { digitalWrite(PIN_BUZZER, HIGH); }

// ---------- BLE Callback ----------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    bleConnected = true;
    Serial.println("✅ BLE Connected");
  }
  void onDisconnect(BLEServer *pServer) override {
    bleConnected = false;
    Serial.println("⚠️ BLE Disconnected, restarting advertising...");
    BLEDevice::startAdvertising();
  }
};

class CommandCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String cmd = c->getValue();
    if (cmd.length() == 0) return;

    cmd.trim();
    Serial.printf("📩 Command received: %s\n", cmd.c_str());

    if (cmd.equalsIgnoreCase("PING")) {
      Serial.println("📡 PING OK");
    } else if (cmd.equalsIgnoreCase("RESET")) {
      Serial.println("♻️ Restarting ESP32...");
      delay(300);
      ESP.restart();
    } else if (cmd.equalsIgnoreCase("DISCONNECT")) {
      Serial.println("🔌 Disconnect");
      BLEDevice::startAdvertising();
    }
  }
};

// ---------- HR Alert ----------
void driveBuzzer(float bpm) {
  static bool beep100 = false;
  static bool beep120 = false;

  if (bpm > HR_LIMIT_HIGH && !beep120) {
    for (int i = 0; i < 2; i++) {
      buzzerOn();
      delay(100);
      buzzerOff();
      delay(100);
    }
    beep120 = true;
    beep100 = true;
  }
  else if (bpm > HR_LIMIT_TACHY && !beep100) {
    buzzerOn();
    delay(120);
    buzzerOff();
    beep100 = true;
  }

  if (bpm < HR_LIMIT_TACHY - 5) {
    beep100 = false;
    beep120 = false;
  }
}

// --------------------------------------------------
//  Wrist-Optimized Heart Rate Detection
// --------------------------------------------------

float dc = 0;
float bp_x1 = 0, bp_x2 = 0;
float bp_y1 = 0, bp_y2 = 0;

#define BPM_BUF 5
float bpmBuf[BPM_BUF] = {75, 75, 75, 75, 75};
int bpmIndex = 0;

// Bandpass filter for wrist (0.5–4 Hz)
float bandpassFilter(float x) {
  float y = 1.894427 * bp_y1 - 0.9025 * bp_y2 +
            0.0047 * x - 0.0094 * bp_x1 + 0.0047 * bp_x2;

  bp_x2 = bp_x1;
  bp_x1 = x;
  bp_y2 = bp_y1;
  bp_y1 = y;

  return y;
}

// median of 5 values
float median5(float *a) {
  float b[5];
  memcpy(b, a, sizeof(b));
  for (int i = 0; i < 5; i++) {
    for (int j = i+1; j < 5; j++) {
      if (b[j] < b[i]) {
        float t = b[i];
        b[i] = b[j];
        b[j] = t;
      }
    }
  }
  return b[2];
}

unsigned long lastBeatTime = 0;

// Updated HR detection
void updateHeartRateFromMAX30102() {
  long ir = maxSensor.getIR();
  if (ir < 3000) return;

  dc = 0.97 * dc + 0.03 * ir;
  float ac = ir - dc;

  float bp = bandpassFilter(ac);
  if (fabs(bp) < 25) return; // SNR check

  static float lastBP = 0;
  static bool rising = false;

  if (!rising && bp > 0 && lastBP <= 0) {
    rising = true;
  }

  if (rising && bp < 0 && lastBP >= 0) {
    unsigned long now = millis();
    unsigned long ibi = now - lastBeatTime;

    if (lastBeatTime != 0 && ibi > 300 && ibi < 2000) {
      float bpm = 60000.0 / ibi;

      bpmBuf[bpmIndex++] = bpm;
      if (bpmIndex >= BPM_BUF) bpmIndex = 0;

      float filteredBPM = median5(bpmBuf);
      avgBPM = 0.85 * avgBPM + 0.15 * filteredBPM;

      Serial.print("❤️ Wrist BPM = ");
      Serial.println(avgBPM, 1);
    }

    lastBeatTime = now;
    rising = false;
  }

  lastBP = bp;
}

// --------------------------------------------------
// Setup
// --------------------------------------------------
void setup() {
  Serial.begin(115200);

  Wire.begin();

  pinMode(PIN_BUZZER, OUTPUT);
  buzzerOff();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed!");
    while (1);
  }

  display.clearDisplay();
  display.setCursor(10, 25);
  display.print("HEALTHY MONITOR");
  display.display();

  if (!mlx.begin()) {
    Serial.println("MLX90614 not found!");
    while (1);
  }

  if (!maxSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found!");
    while (1);
  }

  maxSensor.setup();
  maxSensor.setLEDMode(2);
  maxSensor.setSampleRate(100);
  maxSensor.setPulseWidth(411);
  maxSensor.setADCRange(16384);
  maxSensor.setPulseAmplitudeIR(0x5F);
  maxSensor.setPulseAmplitudeRed(0x1F);

  BLEDevice::init(BLE_DEV_NAME);
  BLEDevice::setPower(ESP_PWR_LVL_P9);
  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *svc = server->createService(BLE_SERVICE_UUID);

  notifyChar = svc->createCharacteristic(
      BLE_NOTIFY_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  notifyChar->addDescriptor(new BLE2902());

  cmdChar = svc->createCharacteristic(
      BLE_CMD_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  cmdChar->setCallbacks(new CommandCallback());

  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->start();
}

// --------------------------------------------------
// Loop
// --------------------------------------------------
void loop() {
  // --- Temperature oversampling ---
  float tempSum = 0;
  for (int i = 0; i < 10; i++) {
    tempSum += mlx.readObjectTempC();
    delay(2);
  }
  float rawTemp = tempSum / 10.0;

  smoothTemp = 0.95 * smoothTemp + 0.05 * rawTemp;

  // Convert wrist temp → body temp
  bodyTemp = smoothTemp + 2.0;

  // --- Heart Rate ---
  updateHeartRateFromMAX30102();
  driveBuzzer(avgBPM);

  // --- Fever alert ---
  if (bodyTemp >= FEVER_THRESHOLD && !feverBeeped) {
    buzzerOn();
    delay(150);
    buzzerOff();
    feverBeeped = true;
  }
  if (bodyTemp < FEVER_THRESHOLD - 0.3) feverBeeped = false;

  // --- BLE data ---
  if (bleConnected && millis() - lastSendMs > 1000) {
    lastSendMs = millis();
    char buf[25];
    snprintf(buf, sizeof(buf), "%d,%.1f", (int)avgBPM, bodyTemp);
    notifyChar->setValue((uint8_t *)buf, strlen(buf));
    notifyChar->notify();
  }

  // --- OLED Display ---
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(18, 0);
  display.print("HEALTHY MONITOR");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(5, 20);
  display.printf("HEART RATE  %3d BPM", (int)avgBPM);

  display.setCursor(5, 35);
  display.printf("TEMP        %.1f C", bodyTemp);

  display.setCursor(5, 55);
  display.printf("BLE : %s", bleConnected ? "CONNECTED" : "DISCONNECTED");

  display.display();

  delay(10);
}
