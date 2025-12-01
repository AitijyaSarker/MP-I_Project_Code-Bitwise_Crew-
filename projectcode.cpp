#define BLYNK_TEMPLATE_ID "TMPL6GEsuyvV9"
#define BLYNK_TEMPLATE_NAME "IoT based fall detection device 2 "
#define BLYNK_AUTH_TOKEN "c7A6TuwpCKvZmav_jFymWRL4A0CrzC-X"

#define BLYNK_PRINT Serial
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <HardwareSerial.h>
#include <math.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define LED_PIN 2 // LED pin

#define BLYNK_SERVER "blynk.cloud"
#define BLYNK_PORT 80

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Utso123";
char pass[] = "12345678";

Adafruit_MPU6050 mpu;
BlynkTimer timer;
HardwareSerial SIM900(2); // RX=16, TX=17

const unsigned long SENSOR_INTERVAL_MS = 200;
const uint8_t MPU_ADDR = 0x68;
bool mpuInitialized = false;

const char emergencyNumber[] = "+8801306270430"; // calling number
const float FALL_THRESHOLD = 11.0; // g force magnitude threshold 

// Call state machine 
enum CallState { CALL_IDLE, CALL_DIALING, CALL_COOLDOWN };
CallState callState = CALL_IDLE;

bool callingInProgress = false;
unsigned long callStateTimestamp = 0;
const unsigned long DIAL_DURATION_MS = 30000; // Ringing time
const unsigned long COOLDOWN_MS = 20000;      // cool down time

// ---------- Helpers ----------
String readSIM900(unsigned long timeout_ms) {
  unsigned long start = millis();
  String resp = "";
  while (millis() - start < timeout_ms) {
    while (SIM900.available()) {
      char c = (char)SIM900.read();
      Serial.write(c);
      resp += c;
      start = millis();
    }
    delay(5);
  }
  Serial.println();
  return resp;
}

void simSend(const char* cmd) {
  Serial.print("> "); Serial.println(cmd);
  SIM900.println(cmd);
}

bool ensureNetworkQuick(int retries = 3, unsigned long waitMs = 1000) {
  for (int i = 0; i < retries; ++i) {
    Serial.printf("ensureNetworkQuick: attempt %d/%d\n", i+1, retries);
    simSend("AT+CREG?");
    String creg = readSIM900(600);
    if (creg.indexOf(",1") >= 0 || creg.indexOf(",5") >= 0) {
      // check signal
      simSend("AT+CSQ");
      String csq = readSIM900(600);
      int idx = csq.indexOf("+CSQ:");
      if (idx >= 0) {
        int comma = csq.indexOf(',', idx);
        if (comma > idx) {
          String val = csq.substring(idx + 5, comma);
          val.trim();
          int rssi = val.toInt();
          Serial.printf("CSQ rssi=%d\n", rssi);
          if (rssi != 99 && rssi >= 6) return true; 
          else Serial.println("Signal low; trying again.");
        } else {
          return true; 
        }
      } else {
        return true; // registered and no CSQ parse — accept
      }
    } else {
      Serial.println("Not registered yet.");
    }
    delay(waitMs);
  }

  // If simple retries failed, try soft radio reset once (AT+CFUN=1)
  Serial.println("ensureNetworkQuick: retries failed -> trying AT+CFUN=1 (soft radio reset)");
  simSend("AT+CFUN=1");
  delay(3000);
  readSIM900(1200);
  // give some time then re-check
  delay(2000);
  simSend("AT+CREG?");
  String creg2 = readSIM900(800);
  if (creg2.indexOf(",1") >= 0 || creg2.indexOf(",5") >= 0) {
    Serial.println("Registered after CFUN");
    return true;
  }
  Serial.println("ensureNetworkQuick: still not registered after CFUN");
  return false;
}

// start dialing using the exact working sequence you had
void startDialSimple(const char* number) {
  if (callState != CALL_IDLE) {
    Serial.println("startDialSimple: not IDLE, ignoring");
    return;
  }

  Serial.println("startDialSimple: checking network quickly...");
  if (!ensureNetworkQuick()) {
    Serial.println("Network not ready -> entering cooldown");
    callState = CALL_COOLDOWN;
    callStateTimestamp = millis();
    callingInProgress = false;
    digitalWrite(LED_PIN, LOW);
    return;
  }

  // EXACT working commands & timing 
  SIM900.println("AT");
  delay(1000);
  SIM900.println("AT+CREG?");
  delay(2000);
  // send dial command in same format that worked
  String dcmd = String("ATD") + String(number) + String(";");
  SIM900.println(dcmd);
  Serial.print("Dialing via command: "); Serial.println(dcmd);
  callingInProgress = true;
  digitalWrite(LED_PIN, HIGH);
  callState = CALL_DIALING;
  callStateTimestamp = millis();
}

// clean hangup & prepare for next call
void hangupAndPrepare() {
  SIM900.println("ATH");
  delay(500);
  callingInProgress = false;
  digitalWrite(LED_PIN, LOW);
  Serial.println("Call ended. Flushing and checking SIM900 readiness...");

  // flush
  while (SIM900.available()) SIM900.read();
  delay(500);

  // quick AT to ensure module is responsive
  SIM900.println("AT");
  delay(300);
  readSIM900(600);

  // go to cooldown
  callState = CALL_COOLDOWN;
  callStateTimestamp = millis();
  Serial.println("Entered COOLDOWN.");
}

// ---------- MPU and Blynk logic (same as yours) ----------
int readWhoAmI(uint8_t addr = MPU_ADDR) {
  Wire.beginTransmission(addr);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return -1;
  Wire.requestFrom((int)addr, 1);
  if (Wire.available()) return Wire.read();
  return -1;
}

void initMPU_Reliable() {
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(50);

  Serial.println("Trying mpu.begin()...");
  if (mpu.begin()) {
    Serial.println("mpu.begin() succeeded.");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuInitialized = true;
    return;
  }

  Serial.println("mpu.begin() failed. Trying WHO_AM_I...");
  int who = readWhoAmI(MPU_ADDR);
  if (who == 0x68 || who == 0x69) {
    Serial.println("MPU detected via WHO_AM_I. Marking as initialized.");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuInitialized = true;
    return;
  }

  sensors_event_t a, g, temp;
  if (mpu.getEvent(&a, &g, &temp)) {
    Serial.println("mpu.getEvent() returned data -> treating MPU as present.");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuInitialized = true;
    return;
  }

  mpuInitialized = false;
  Serial.println("MPU not detected. Check wiring/power.");
}

void publishSensor() {
  if (!mpuInitialized) return;

  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) return;

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  float accel_mag = sqrt(ax*ax + ay*ay + az*az);

  float gx = g.gyro.x;
  float gy = g.gyro.y;
  float gz = g.gyro.z;

  Serial.print("Ax:"); Serial.print(ax);
  Serial.print(" Ay:"); Serial.print(ay);
  Serial.print(" Az:"); Serial.print(az);
  Serial.print(" |A|:"); Serial.print(accel_mag);
  Serial.print(" Gx:"); Serial.print(gx);
  Serial.print(" Gy:"); Serial.print(gy);
  Serial.print(" Gz:"); Serial.println(gz);

  Blynk.virtualWrite(V0, ax);
  Blynk.virtualWrite(V1, ay);
  Blynk.virtualWrite(V2, az);
  Blynk.virtualWrite(V6, accel_mag);
  Blynk.virtualWrite(V3, gx);
  Blynk.virtualWrite(V4, gy);
  Blynk.virtualWrite(V5, gz);

  // Trigger only when idle or cooldown finished
  if (accel_mag > FALL_THRESHOLD && (callState == CALL_IDLE || (callState == CALL_COOLDOWN && (millis() - callStateTimestamp >= COOLDOWN_MS)))) {
    Serial.println("Fall detected -> will dial using simple sequence");
    startDialSimple(emergencyNumber);
  }
}

// ---------- Setup & Loop ----------
void initSIM900() {
  SIM900.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("Initializing SIM900...");
  delay(2000); // allow boot
  SIM900.println("AT");
  delay(300);
  readSIM900(400);
  SIM900.println("AT+CREG?");
  delay(300);
  readSIM900(400);
  SIM900.println("AT+CSQ");
  delay(300);
  readSIM900(400);
  Serial.println("SIM900 init done.");
}

void setup() {
  Serial.begin(115200);
  delay(50);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("=== MPU + Blynk + SIM900 + LED (stable dial variant) ===");
  initMPU_Reliable();
  if (mpuInitialized) Serial.println("MPU initialized — ready.");
  else Serial.println("MPU NOT initialized — sensor publishing disabled.");

  // WiFi
  Serial.print("Connecting WiFi: "); Serial.println(ssid);
  WiFi.begin(ssid, pass);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) Serial.println("WiFi connected. IP: " + WiFi.localIP().toString());
  else Serial.println("WiFi failed, status=" + String(WiFi.status()));

  // Blynk
  Serial.println("Configuring Blynk...");
  Blynk.config(auth, BLYNK_SERVER, BLYNK_PORT);
  if (Blynk.connect(5000)) Serial.println("✅ Blynk Connected Successfully!");
  else Serial.println("❌ Blynk connect() failed/timeout.");

  // SIM900 init
  initSIM900();

  if (mpuInitialized) timer.setInterval(SENSOR_INTERVAL_MS, publishSensor);
}

void loop() {
  Blynk.run();
  timer.run();

  // Manage call state
  if (callState == CALL_DIALING) {
    // watch responses quickly
    String resp = readSIM900(100);
    if (resp.indexOf("NO CARRIER") >= 0 || resp.indexOf("BUSY") >= 0) {
      Serial.println("Dial reported failure (NO CARRIER/BUSY). Hanging up and cooldown.");
      hangupAndPrepare();
    } else {
      // if dial time exceeds DIAL_DURATION_MS, hang up
      if (millis() - callStateTimestamp >= DIAL_DURATION_MS) {
        Serial.println("Dial timeout -> hanging up.");
        hangupAndPrepare();
      }
    }
  } else if (callState == CALL_COOLDOWN) {
    if (millis() - callStateTimestamp >= COOLDOWN_MS) {
      Serial.println("Cooldown over. Returning to IDLE.");
      callState = CALL_IDLE;
    }
  }

  // print any unsolicited GSM messages
  while (SIM900.available()) {
    Serial.write(SIM900.read());
  }

  // Blynk reconnect attempts
  static unsigned long lastTry = 0;
  if (!Blynk.connected() && millis() - lastTry > 5000) {
    lastTry = millis();
    Serial.println("Blynk disconnected -> trying connect()");
    Blynk.connect(3000);
  }

  // attempt to reinit MPU if missing
  static unsigned long lastMpuTry = 0;
  if (!mpuInitialized && millis() - lastMpuTry > 10000) {
    lastMpuTry = millis();
    Serial.println("Periodic MPU init attempt...");
    initMPU_Reliable();
    if (mpuInitialized) timer.setInterval(SENSOR_INTERVAL_MS, publishSensor);
  }

  delay(10);
}