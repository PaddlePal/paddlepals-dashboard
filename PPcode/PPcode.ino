// Comment this out when done developing to remove ALL serial output
#define DEBUG

#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>
#include "TickTwo.h"

// =============================================================================
// ZONE CONFIG
// =============================================================================
const int zonePins[6] = {A6, A5, A4, A3, A2, A1};
const int numZones    = 6;
const int threshold   = 5; // 8-bit threshold

// =============================================================================
// CONTROL & LED PINS
// =============================================================================
const int button1     = 8;
const int kill_switch = 9;
const int interrupt_1 = 10;
const int P_G1 = 2, P_R1 = 3, P_B1 = 4;   // Paddle RGB LED
const int H_G2 = 5, H_R2 = 6, H_B2 = 7;   // Handle RGB LED

// =============================================================================
// BLE
// =============================================================================
BLEService paddleService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");

BLEStringCharacteristic fsrCharacteristic(
  "beb5483e-36e1-4688-b7f5-ea07361b26a8",
  BLERead | BLENotify,
  100
);

BLEStringCharacteristic imuCharacteristic(
  "beb5483e-36e1-4688-b7f5-ea07361b26a9",
  BLERead | BLENotify,
  80
);

// =============================================================================
// HIT DETECTION
// =============================================================================
const int DEBOUNCE_TIME = 150;  // ms

int           zonePeak[6]    = {0};
bool          zoneActive[6]  = {false};
unsigned long zoneLastHit[6] = {0};

// =============================================================================
// IMU SNAPSHOT (for hit packets)
// =============================================================================
struct IMUSnapshot {
  float gx, gy, gz;
  float ax, ay, az;
  bool  fresh;
};
IMUSnapshot imuLatest = {0, 0, 0, 0, 0, 0, false};

// =============================================================================
// IMU POLL STATE
// =============================================================================
unsigned long lastIMURead   = 0;
const unsigned long IMU_INTERVAL_MS = 5; // ~200 Hz

// =============================================================================
// SESSION STATE
// =============================================================================
bool   sessionActive = false;
String hitDataString = "";
String imuDataString = "";

// =============================================================================
// LED HELPERS
// =============================================================================
void setLED(int r, int g, int b, int R, int G, int B) {
  digitalWrite(R, r); digitalWrite(G, g); digitalWrite(B, b);
}
void paddleOff()   { setLED(LOW,  LOW,  LOW,  P_R1, P_G1, P_B1); }
void paddleGreen() { setLED(LOW,  HIGH, LOW,  P_R1, P_G1, P_B1); }
void paddleRed()   { setLED(HIGH, LOW,  LOW,  P_R1, P_G1, P_B1); }
void paddleWhite() { setLED(HIGH, HIGH, HIGH, P_R1, P_G1, P_B1); }
void handleOff()   { setLED(LOW,  LOW,  LOW,  H_R2, H_G2, H_B2); }
void handleBlue()  { setLED(LOW,  LOW,  HIGH, H_R2, H_G2, H_B2); }
void handleGreen() { setLED(LOW,  HIGH, LOW,  H_R2, H_G2, H_B2); }
void handleWhite() { setLED(HIGH, HIGH, HIGH, H_R2, H_G2, H_B2); }

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void sendBLE();
void paddleLightOff();
void toggleBlink();
void checkHoldStart();
void checkHoldReset();

// =============================================================================
// SOFTWARE TICKERS
// =============================================================================
TickTwo bleTicker(sendBLE, 100, 0, MILLIS);
TickTwo paddleLightTicker(paddleLightOff, 300, 1, MILLIS);
TickTwo blinkTicker(toggleBlink, 500, 0, MILLIS);
TickTwo startHoldTicker(checkHoldStart, 1000, 1, MILLIS);
TickTwo resetHoldTicker(checkHoldReset, 2000, 1, MILLIS);

// =============================================================================
// TICKER CALLBACKS
// =============================================================================
void sendBLE() {
  if (!BLE.connected()) return;

  if (hitDataString.length() > 0) {
    fsrCharacteristic.writeValue(hitDataString);
    #ifdef DEBUG
      Serial.print("BLE HIT: "); Serial.println(hitDataString);
    #endif
    hitDataString = "";
  }

  if (imuDataString.length() > 0) {
    imuCharacteristic.writeValue(imuDataString);
    imuDataString = "";
  }
}

void paddleLightOff() { paddleOff(); }

bool blinkState = false;
void toggleBlink() {
  blinkState = !blinkState;
  blinkState ? handleWhite() : handleOff();
}

void checkHoldStart() {
  if (digitalRead(button1) == LOW && BLE.connected()) {
    sessionActive = true;
    blinkTicker.stop();
    bleTicker.start();
    handleGreen();
    #ifdef DEBUG
      Serial.println("SESSION STARTED");
    #endif
  }
}

void checkHoldReset() {
  if (digitalRead(button1) == LOW) {
    sessionActive = false;
    hitDataString = "";
    imuDataString = "";
    for (int i = 0; i < numZones; i++) {
      zoneActive[i]  = false;
      zonePeak[i]    = 0;
      zoneLastHit[i] = 0;
    }
    imuLatest = {0, 0, 0, 0, 0, 0, false};
    paddleLightTicker.stop();
    bleTicker.stop();
    paddleWhite();
    handleBlue();
    #ifdef DEBUG
      Serial.println("SESSION RESET");
    #endif
  }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  #ifdef DEBUG
    Serial.begin(9600);
    while (!Serial);
    Serial.println("System booting...");
  #endif

  pinMode(P_G1, OUTPUT); pinMode(P_R1, OUTPUT); pinMode(P_B1, OUTPUT);
  pinMode(H_G2, OUTPUT); pinMode(H_R2, OUTPUT); pinMode(H_B2, OUTPUT);
  pinMode(button1, INPUT_PULLUP);
  pinMode(interrupt_1, INPUT_PULLUP);
  pinMode(kill_switch, OUTPUT); digitalWrite(kill_switch, HIGH);
  analogReadResolution(8);

  delay(1000);
  if (!IMU.begin()) {
    #ifdef DEBUG
      Serial.println("IMU failed!");
    #endif
    while (1);
  }
  #ifdef DEBUG
    Serial.println("IMU initialized");
  #endif

  blinkTicker.start();
  paddleWhite();

  if (!BLE.begin()) {
    #ifdef DEBUG
      Serial.println("BLE failed!");
    #endif
    while (1);
  }
  BLE.setLocalName("PaddleSensor");
  BLE.setAdvertisedService(paddleService);
  paddleService.addCharacteristic(fsrCharacteristic);
  paddleService.addCharacteristic(imuCharacteristic);
  BLE.addService(paddleService);
  fsrCharacteristic.writeValue("Waiting...");
  imuCharacteristic.writeValue("Waiting...");
  BLE.advertise();

  #ifdef DEBUG
    Serial.println("BLE ready");
  #endif
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  BLE.poll(0);

  bleTicker.update();
  paddleLightTicker.update();
  blinkTicker.update();
  startHoldTicker.update();
  resetHoldTicker.update();

  if (digitalRead(interrupt_1) == LOW) {
    digitalWrite(kill_switch, LOW);
    while (1);
  }

  if (!BLE.connected()) {
    if (blinkTicker.state() != RUNNING) blinkTicker.start();
  } else if (!sessionActive) {
    blinkTicker.stop();
    handleBlue();
  }

  bool buttonPressed = (digitalRead(button1) == LOW);
  if (!sessionActive) {
    if (buttonPressed && BLE.connected()) {
      if (startHoldTicker.state() != RUNNING) startHoldTicker.start();
    } else {
      startHoldTicker.stop();
    }
  } else {
    if (buttonPressed) {
      if (resetHoldTicker.state() != RUNNING) resetHoldTicker.start();
    } else {
      resetHoldTicker.stop();
    }
  }

  if (sessionActive) {
    unsigned long now = millis();

    // ---------------------------------------------------------
    // IMU POLL — raw values + micros() timestamp
    // Frontend uses the timestamp to compute accurate dt
    // for the Madgwick filter, unaffected by BLE jitter
    // ---------------------------------------------------------
    if (now - lastIMURead >= IMU_INTERVAL_MS) {
      lastIMURead = now;

      float gx, gy, gz, ax, ay, az;
      if (IMU.gyroscopeAvailable()    && IMU.readGyroscope(gx, gy, gz) &&
          IMU.accelerationAvailable() && IMU.readAcceleration(ax, ay, az)) {

        // Stamp micros() immediately after reading — this is the true
        // sample time. micros() overflows every ~70 min; frontend handles it.
        unsigned long t = micros();

        imuLatest = {gx, gy, gz, ax, ay, az, true};

        // Format: "T:us,GX:v,GY:v,GZ:v,AX:v,AY:v,AZ:v"
        imuDataString  = "T:";  imuDataString += String(t);
        imuDataString += ",GX:"; imuDataString += String(gx, 1);
        imuDataString += ",GY:"; imuDataString += String(gy, 1);
        imuDataString += ",GZ:"; imuDataString += String(gz, 1);
        imuDataString += ",AX:"; imuDataString += String(ax, 2);
        imuDataString += ",AY:"; imuDataString += String(ay, 2);
        imuDataString += ",AZ:"; imuDataString += String(az, 2);
      }
    }

    // ---------------------------------------------------------
    // FSR ZONE SAMPLING — peak detection
    // ---------------------------------------------------------
    for (int i = 0; i < numZones; i++) {
      int reading = analogRead(zonePins[i]);
      if (reading > zonePeak[i]) zonePeak[i] = reading;
    }

    // ---------------------------------------------------------
    // HIT DETECTION
    // ---------------------------------------------------------
    for (int i = 0; i < numZones; i++) {
      bool overThreshold = zonePeak[i] > threshold;
      bool debounceOk    = (now - zoneLastHit[i]) >= DEBOUNCE_TIME;

      if (overThreshold && !zoneActive[i] && debounceOk) {
        zoneActive[i]  = true;
        zoneLastHit[i] = now;

        IMUSnapshot snap = imuLatest;
        imuLatest.fresh  = false;

        // Hit packet: zone + peak + raw gyro snapshot for swing classification
        hitDataString += "Z"; hitDataString += String(i + 1);
        hitDataString += ":"; hitDataString += String(zonePeak[i]);
        hitDataString += ",GX:"; hitDataString += String(snap.gx, 1);
        hitDataString += ",GY:"; hitDataString += String(snap.gy, 1);
        hitDataString += ",GZ:"; hitDataString += String(snap.gz, 1);
        hitDataString += "\n";

        if (i == 0) paddleGreen();
        else        paddleRed();
        paddleLightTicker.start();

        #ifdef DEBUG
          Serial.print("HIT Z"); Serial.print(i + 1);
          Serial.print(" val="); Serial.println(zonePeak[i]);
        #endif
      }

      if (!overThreshold) zoneActive[i] = false;
      zonePeak[i] = 0;
    }
  }
}
