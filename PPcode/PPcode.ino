#include <ArduinoBLE.h>
#include "TickTwo.h"

// ----------- ZONES -----------
const int zonePins[6] = {A6, A5, A4, A3, A2, A1};
const int numZones = 6;
const int threshold = 75;

// ----------- CONTROL PINS -----------
const int button1 = 8;
const int kill_switch = 9;
const int interrupt_1 = 10;

// ----------- LED PINS -----------
const int P_G1 = 2;
const int P_R1 = 3;
const int P_B1 = 4;
const int H_G2 = 5;
const int H_R2 = 6;
const int H_B2 = 7;

// ----------- BLE -----------
BLEService fsrService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
BLEStringCharacteristic fsrCharacteristic(
  "beb5483e-36e1-4688-b7f5-ea07361b26a8",
  BLERead | BLENotify,
  100
);

// ----------- HIT DETECTION -----------
const int DEBOUNCE_TIME = 150;
const int SAMPLE_BURST  = 8;

int  zonePeak[6]    = {0};
bool zoneActive[6]  = {false};
unsigned long zoneLastHit[6] = {0};

// ----------- SESSION -----------
bool sessionActive = false;
String dataString  = "";

// ----------- FORWARD DECLARATIONS -----------
void sendBLE();
void paddleLightOff();
void toggleBlink();
void checkHoldStart();
void checkHoldReset();


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


TickTwo bleTicker(sendBLE, 100, 0, MILLIS);
TickTwo paddleLightTicker(paddleLightOff, 300, 1, MILLIS);
TickTwo blinkTicker(toggleBlink, 500, 0, MILLIS);
TickTwo startHoldTicker(checkHoldStart, 1000, 1, MILLIS);
TickTwo resetHoldTicker(checkHoldReset, 2000, 1, MILLIS);


void sendBLE() {
  if (BLE.connected() && dataString.length() > 0) {
    fsrCharacteristic.writeValue(dataString);
    dataString = "";
  }
}

void paddleLightOff() {
  paddleOff();
}

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
    Serial.println("SESSION STARTED");
  }
}

void checkHoldReset() {
  if (digitalRead(button1) == LOW) {
    sessionActive = false;
    dataString    = "";
    for (int i = 0; i < numZones; i++) {
      zoneActive[i]  = false;
      zonePeak[i]    = 0;
      zoneLastHit[i] = 0;
    }
    paddleLightTicker.stop();
    bleTicker.stop();
    paddleWhite();
    handleBlue();
    Serial.println("SESSION RESET");
  }
}


void setup() {
  Serial.begin(9600);

  pinMode(P_G1, OUTPUT); pinMode(P_R1, OUTPUT); pinMode(P_B1, OUTPUT);
  pinMode(H_G2, OUTPUT); pinMode(H_R2, OUTPUT); pinMode(H_B2, OUTPUT);
  pinMode(button1, INPUT_PULLUP);
  pinMode(interrupt_1, INPUT_PULLUP);
  pinMode(kill_switch, OUTPUT);
  digitalWrite(kill_switch, HIGH);

  if (!BLE.begin()) { Serial.println("BLE failed!"); while (1); }
  BLE.setLocalName("FSR_Reader");
  BLE.setAdvertisedService(fsrService);
  fsrService.addCharacteristic(fsrCharacteristic);
  BLE.addService(fsrService);
  fsrCharacteristic.writeValue("Waiting...");
  BLE.advertise();

  blinkTicker.start();
  Serial.println("System Ready");
  paddleWhite();
}


void loop() {
  BLE.poll();

  // ---- UPDATE ALL TICKERS ----
  bleTicker.update();
  paddleLightTicker.update();
  blinkTicker.update();
  startHoldTicker.update();
  resetHoldTicker.update();

  // ---- KILL SWITCH ----
  if (digitalRead(interrupt_1) == LOW) {
    digitalWrite(kill_switch, LOW);
    while (1);
  }

  // ---- HANDLE LED (connection state) ----
  if (!BLE.connected()) {
    if (blinkTicker.state() != RUNNING) blinkTicker.start();
  } else if (!sessionActive) {
    blinkTicker.stop();
    handleBlue();
  }

  // ---- BUTTON ----
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

    // Burst sample all zones
    for (int s = 0; s < SAMPLE_BURST; s++) {
      for (int i = 0; i < numZones; i++) {
        int reading = analogRead(zonePins[i]);
        if (reading > zonePeak[i]) zonePeak[i] = reading;
      }
    }

    // Evaluate peaks
    for (int i = 0; i < numZones; i++) {
      bool overThreshold = zonePeak[i] > threshold;
      bool debounceOk    = (now - zoneLastHit[i]) >= DEBOUNCE_TIME;

      if (overThreshold && !zoneActive[i] && debounceOk) {
        zoneActive[i]  = true;
        zoneLastHit[i] = now;

        dataString += "Z";
        dataString += String(i + 1);
        dataString += ":";
        dataString += String(zonePeak[i]);
        dataString += "\n";

        if (i == 0) paddleGreen();
        else        paddleRed();

        paddleLightTicker.start();

        Serial.print("HIT Zone "); Serial.print(i + 1);
        Serial.print(" val=");     Serial.println(zonePeak[i]);
      }

      if (!overThreshold) zoneActive[i] = false;
      zonePeak[i] = 0;
    }
  }
  // No delay() — loop runs as fast as possible
}