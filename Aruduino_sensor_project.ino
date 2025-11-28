#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------- OLED ----------------
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire);

// ---------------- Sensors ----------------
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX30105 sensor;

// ---------------- Pins ----------------
#define PIN_BUZZER 4

// ---------------- BLE ----------------
#define BLE_NAME "HeartSense-ESP32"
#define BLE_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_NOTIFY_UUID   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CMD_UUID      "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *notifyChar;
BLECharacteristic *cmdChar;
bool bleConnected = false;

// ---------------- HR / TEMP ----------------
float filteredHR   = 0.0f;   // ค่า BPM ที่ผ่าน filter แล้ว
float smoothTemp   = 36.5f;
float bodyTemp     = 36.5f;
bool wristDetected = false;

unsigned long lastBeat        = 0;    // เวลา beat ล่าสุด (ms)
unsigned long lastValidBeat   = 0;    // เวลา beat ล่าสุดที่คำนวณ BPM ได้จริง
unsigned long lastOLED        = 0;
unsigned long lastBLE         = 0;
unsigned long lastSerial      = 0;

// สำหรับ signal processing
float dcLevel    = 0.0f;
float prevAC     = 0.0f;
float prevPrevAC = 0.0f;

// ---------------- BUZZER ----------------
void beepShort() {
  tone(PIN_BUZZER, 3000);
  delay(150);
  noTone(PIN_BUZZER);
}

void beepDouble() {
  for (int i = 0; i < 2; i++) {
    tone(PIN_BUZZER, 3000);
    delay(120);
    noTone(PIN_BUZZER);
    delay(120);
  }
}

// ---------------- BLE ----------------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) {
    bleConnected = true;
    Serial.println("BLE Connected");
  }
  void onDisconnect(BLEServer *server) {
    bleConnected = false;
    Serial.println("BLE Disconnected → Re-Advertising");
    BLEDevice::startAdvertising();
  }
};

class CommandCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String cmd = c->getValue();
    cmd.trim();
    Serial.print("CMD: "); Serial.println(cmd);

    if (cmd == "RESET") ESP.restart();
    if (cmd == "PING")  Serial.println("PING OK");
  }
};

// ---------------- OLED UI ----------------
void drawUI() {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  display.setCursor(18, 0);
  display.print("HEALTHY MONITOR");
  display.drawLine(0, 12, 127, 12, SH110X_WHITE);

  display.setCursor(5,25); display.print("HEART RATE :");
  display.setCursor(5,40); display.print("TEMP      :");
  display.setCursor(5,55); display.print("BLE       :");

  display.display();
}

void updateOLED() {
  // HR
  display.fillRect(100,25,28,10,SH110X_BLACK);
  display.setCursor(100,25);
  if (!wristDetected || filteredHR <= 0.1f) display.print("--");
  else display.printf("%3d", (int)filteredHR);

  // TEMP
  display.fillRect(100,40,28,10,SH110X_BLACK);
  display.setCursor(100,40);
  display.printf("%.1f", bodyTemp);

  // BLE
  display.fillRect(100,55,28,10,SH110X_BLACK);
  display.setCursor(100,55);
  display.print(bleConnected ? "OK" : "NO");

  display.display();
}

// ---------------- TEMP SAFE READ ----------------
float readTempSafe() {
  float t = mlx.readObjectTempC();
  if (isnan(t)) {
    delay(30);
    t = mlx.readObjectTempC();
  }
  if (isnan(t)) {
    Serial.println("⚠️ TEMP = NAN");
    return smoothTemp;
  }
  return t;
}

// ---------------- HEART RATE (ปรับใหม่ให้ไม่ค้าง) ----------------
//
//  - ใช้ threshold IR ต่ำลง → ไม่ต้องอุดแสงจนมืด
//  - ใช้ AC amplitude threshold ลดลง (จาก 500 → 150)
//  - ถ้า 3 วินาทีไม่มี beat → ค่อย ๆ ลด HR ลงจนเป็น 0 (--)
//
void updateHeartRate() {
  long ir = sensor.getIR();

  // 1) ตรวจจับว่ามีผิวแตะเซ็นเซอร์ไหม
  // จากค่าที่คุณให้มา IR ตอนแตะ ~150k–210k, baseline ต่ำกว่านั้น
  // ลด threshold ลง เพื่อไม่ต้องอุดเซ็นเซอร์จนมืดสนิท
  if (ir < 40000) {  // เดิม 80000 → ทำให้ง่ายขึ้น
    wristDetected = false;
    // ถ้าไม่มีข้อมือ ให้ HR ค่อย ๆ ลดลง
    if (filteredHR > 1.0f) {
      filteredHR *= 0.9f; // decay
      if (filteredHR < 1.0f) filteredHR = 0.0f;
    } else {
      filteredHR = 0.0f;
    }
    return;
  }

  wristDetected = true;

  // 2) แยก DC/AC
  dcLevel = 0.995f * dcLevel + 0.005f * (float)ir;
  float ac = (float)ir - dcLevel;

  // 3) คำนวณ slope
  float s1 = ac - prevAC;
  float s2 = prevAC - prevPrevAC;

  bool peak = false;

  // Peak เงื่อนไข:
  // - ก่อนหน้า s2 > 0 (กำลังขึ้น)
  // - ตอนนี้ s1 < 0 (เริ่มลง) = จุดบนสุดของคลื่น
  // - amplitude มีขนาดพอสมควร (ใช้ 150 แทน 500 ให้จับง่ายขึ้น)
  if (s2 > 0 && s1 < 0 && fabs(prevAC) > 150.0f) {
    peak = true;
  }

  unsigned long now = millis();

  if (peak) {
    unsigned long ibi = now - lastBeat;
    lastBeat = now;
    lastValidBeat = now;

    // กรอง period (IBI)
    if (ibi > 300 && ibi < 2000) {   // 30–200 bpm
      float bpm = 60000.0f / (float)ibi;
      // ทำ filter ให้นุ่มลง (จาก 0.85/0.15 → 0.6/0.4 เพื่อให้ BPM ขยับเร็วขึ้น)
      if (filteredHR < 10.0f) {
        filteredHR = bpm;  // เริ่มต้นให้เท่ากับ bpm จริงก่อน
      } else {
        filteredHR = 0.6f * filteredHR + 0.4f * bpm;
      }

      Serial.print("❤️ BPM = ");
      Serial.println(filteredHR, 1);
    }
  }

  // 4) ถ้าไม่มี beat มานานเกิน 3 วินาที → ค่อย ๆ ลด HR ลง
  if (lastValidBeat > 0 && (now - lastValidBeat) > 3000) {
    if (filteredHR > 1.0f) {
      filteredHR *= 0.9f;  // decay ทีละนิด
      if (filteredHR < 1.0f) filteredHR = 0.0f;
    } else {
      filteredHR = 0.0f;
    }
  }

  prevPrevAC = prevAC;
  prevAC = ac;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  // I2C (ใช้ 11/12 ตามที่บอร์ดคุณต่ออยู่)
  Wire.begin(11,12);
  Wire.setClock(400000);

  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);

  // OLED
  display.begin(0x3C, true);
  drawUI();

  // MLX90614
  if (!mlx.begin()) {
    Serial.println("❌ MLX90614 Not Found!");
    while(1);
  }
  Serial.println("✓ MLX90614 OK");

  // MAX30102 Clone (ใช้ config ที่คุณเทสแล้วไฟติด)
  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("❌ MAX30102 Not Found!");
    while(1);
  }
  Serial.println("✓ MAX30102 OK");

  sensor.setup();
  sensor.setLEDMode(3);            // Red + IR
  sensor.setPulseAmplitudeRed(0xFF);
  sensor.setPulseAmplitudeIR(0xFF);
  sensor.setPulseWidth(411);
  sensor.setADCRange(16384);
  sensor.setSampleRate(800);

  sensor.writeRegister8(0x57, 0x0C, 0xFF);
  sensor.writeRegister8(0x57, 0x0D, 0xFF);

  Serial.println("🔥 IR LED BOOST MODE ACTIVE");

  // BLE
  BLEDevice::init(BLE_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *svc = server->createService(BLE_SERVICE_UUID);

  notifyChar = svc->createCharacteristic(
    BLE_NOTIFY_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  notifyChar->addDescriptor(new BLE2902());

  cmdChar = svc->createCharacteristic(
    BLE_CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  cmdChar->setCallbacks(new CommandCallback());

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->start();

  Serial.println("System Ready.");
}

// ---------------- LOOP ----------------
void loop() {

  // Temperature
  float raw = readTempSafe();
  smoothTemp = 0.95f * smoothTemp + 0.05f * raw;
  bodyTemp   = smoothTemp + 2.0f;   // offset ตามที่คุณใช้

  // Heart Rate
  updateHeartRate();

  // ----------- Buzzer Alert ----------- 
  static bool beep100 = false;
  static bool beep120 = false;
  static bool feverBeep = false;

  if (wristDetected && filteredHR > 0) {
    if (filteredHR > 120 && !beep120) {
      beepDouble();
      beep120 = true;
      beep100 = true;
    }
    else if (filteredHR > 100 && !beep100) {
      beepShort();
      beep100 = true;
    }

    if (filteredHR < 95) {
      beep100 = false;
      beep120 = false;
    }
  }

  if (bodyTemp > 37.5 && !feverBeep) {
    beepShort();
    feverBeep = true;
  }
  if (bodyTemp < 37.2) feverBeep = false;

  // OLED
  if (millis() - lastOLED > 80) {
    lastOLED = millis();
    updateOLED();
  }

  // BLE
  if (bleConnected && millis() - lastBLE > 1000) {
    lastBLE = millis();
    char buf[32];
    if (!wristDetected || filteredHR <= 0.1f)
      snprintf(buf, sizeof(buf),"--,%.1f",bodyTemp);
    else
      snprintf(buf, sizeof(buf),"%d,%.1f",(int)filteredHR,bodyTemp);

    notifyChar->setValue((uint8_t*)buf, strlen(buf));
    notifyChar->notify();
  }

  // SERIAL
  if (millis() - lastSerial > 1000) {
    lastSerial = millis();
    if (!wristDetected || filteredHR <= 0.1f)
      Serial.printf("HR: -- | TEMP: %.1f | BLE: %s\n",
        bodyTemp, bleConnected ? "OK" : "NO");
    else
      Serial.printf("HR: %3d | TEMP: %.1f | BLE: %s\n",
        (int)filteredHR, bodyTemp, bleConnected ? "OK" : "NO");
  }

  delay(10);
}
