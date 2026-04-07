// Comment this out when done developing to remove ALL serial output
//#define DEBUG

//#define USE_IMU

#include <ArduinoBLE.h>

#ifdef USE_IMU
#include <Arduino_BMI270_BMM150.h>
#endif

// =============================================================================
// ZONE CONFIG
// =============================================================================
const int zonePins[6] = {A6, A5, A4, A3, A2, A1};
const int numZones    = 6;
const int threshold   = 5;

// =============================================================================
// CONTROL & LED PINS
// =============================================================================
const int button1     = 8;
const int kill_switch = 9;
const int interrupt_1 = 10;
const int P_G1=2, P_R1=3, P_B1=4;
const int H_G2=5, H_R2=6, H_B2=7;

// =============================================================================
// BLE
// =============================================================================
BLEService paddleService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");

BLEStringCharacteristic fsrCharacteristic(
  "beb5483e-36e1-4688-b7f5-ea07361b26a8",
  BLERead | BLENotify, 100
);

BLEStringCharacteristic imuCharacteristic(
  "beb5483e-36e1-4688-b7f5-ea07361b26a9",
  BLERead | BLENotify, 120
);

BLEStringCharacteristic cmdCharacteristic(
  "beb5483e-36e1-4688-b7f5-ea07361b26aa",
  BLEWrite | BLERead, 32
);

// =============================================================================
// HIT BUFFER
// =============================================================================
struct HitRecord {
  uint8_t zone;
  uint8_t peak;
  float gx, gy, gz;
};

const int HIT_BUF_SIZE = 16;
HitRecord hitBuf[HIT_BUF_SIZE];
uint8_t hitHead = 0;
uint8_t hitTail = 0;

// =============================================================================
// FAST FSR STATE (NEW)
// =============================================================================
volatile uint8_t fsrLatched[6] = {0};
uint8_t fsrPeak[6] = {0};
unsigned long zoneLastHit[6] = {0};

const unsigned long DEBOUNCE_TIME = 8; // MUCH smaller

// =============================================================================
// IMU (OPTIONAL)
// =============================================================================
#ifdef USE_IMU
float latestGx=0, latestGy=0, latestGz=0;
unsigned long lastIMURead = 0;
const unsigned long IMU_INTERVAL_MS = 50;
char imuBuf[128];
bool imuReady = false;
#endif

// =============================================================================
// SESSION STATE
// =============================================================================
bool sessionActive  = false;
const char* pendingStatus = nullptr;

// =============================================================================
// TIMING
// =============================================================================
unsigned long lastBLESend = 0;
unsigned long lastBlink   = 0;
unsigned long paddleLightOffAt = 0;
bool blinkState = false;

const unsigned long BLE_SEND_INTERVAL = 50;   // faster BLE
const unsigned long BLINK_INTERVAL    = 300;

// =============================================================================
// LED HELPERS
// =============================================================================
void setLED(int r,int g,int b,int R,int G,int B){
  digitalWrite(R,r); digitalWrite(G,g); digitalWrite(B,b);
}
void paddleOff()   { setLED(LOW, LOW, LOW, P_R1,P_G1,P_B1); }
void paddleGreen() { setLED(LOW, HIGH,LOW, P_R1,P_G1,P_B1); }
void paddleWhite() { setLED(HIGH,HIGH,HIGH,P_R1,P_G1,P_B1); }
void handleOff()   { setLED(LOW, LOW, LOW, H_R2,H_G2,H_B2); }
void handleBlue()  { setLED(LOW, LOW, HIGH,H_R2,H_G2,H_B2); }
void handleGreen() { setLED(LOW, HIGH,LOW, H_R2,H_G2,H_B2); }
void handleWhite() { setLED(HIGH,HIGH,HIGH,H_R2,H_G2,H_B2); }

// =============================================================================
// SESSION CONTROL
// =============================================================================
void doStart() {
  sessionActive = true;
  pendingStatus = "STATUS:SESSION_STARTED";
  handleGreen();
}

void doReset() {
  sessionActive = false;
  pendingStatus = "STATUS:SESSION_RESET";
  hitHead = hitTail = 0;

  for (int i = 0; i < numZones; i++) {
    fsrLatched[i] = 0;
    fsrPeak[i] = 0;
    zoneLastHit[i] = 0;
  }

  paddleWhite();
  handleBlue();
}

void doShutdown() {
  digitalWrite(kill_switch, LOW);
}

// =============================================================================
// BLE SEND
// =============================================================================
void sendBLE() {
  if (!BLE.connected()) return;

  if (pendingStatus) {
    fsrCharacteristic.writeValue(pendingStatus);
    pendingStatus = nullptr;
    return;
  }

  if (hitHead != hitTail) {
    char buf[100];
    int pos = 0;

    while (hitTail != hitHead && pos < 80) {
      HitRecord &h = hitBuf[hitTail];

      int w = snprintf(buf + pos, sizeof(buf) - pos,
        "Z%d:%d\n", h.zone + 1, h.peak);

      if (w > 0) pos += w;
      hitTail = (hitTail + 1) & (HIT_BUF_SIZE - 1);
    }

    if (pos > 0) {
      buf[pos] = '\0';
      fsrCharacteristic.writeValue(buf);
    }
  }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {

  pinMode(P_G1,OUTPUT); pinMode(P_R1,OUTPUT); pinMode(P_B1,OUTPUT);
  pinMode(H_G2,OUTPUT); pinMode(H_R2,OUTPUT); pinMode(H_B2,OUTPUT);
  pinMode(button1,INPUT_PULLUP);
  pinMode(interrupt_1,INPUT_PULLUP);
  pinMode(kill_switch,OUTPUT); digitalWrite(kill_switch,HIGH);

  analogReadResolution(8);

  paddleWhite();

  if (!BLE.begin()) {
    while(1);
  }

  BLE.setLocalName("PaddleSensor");
  BLE.setAdvertisedService(paddleService);

  paddleService.addCharacteristic(fsrCharacteristic);
  paddleService.addCharacteristic(imuCharacteristic);
  paddleService.addCharacteristic(cmdCharacteristic);

  BLE.addService(paddleService);

  fsrCharacteristic.writeValue("Waiting...");
  BLE.advertise();
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  unsigned long now = millis();

  // =========================
  //  ULTRA-FAST FSR SAMPLING
  // =========================
  if (sessionActive) {
    for (int i = 0; i < numZones; i++) {
      int r = analogRead(zonePins[i]);

      if (r > fsrPeak[i]) fsrPeak[i] = r;

      if (r > threshold) {
        fsrLatched[i] = 1;
      }
    }
  }

  // =========================
  // PROCESS LATCHED HITS
  // =========================
  if (sessionActive) {
    for (int i = 0; i < numZones; i++) {

      if (fsrLatched[i]) {
        fsrLatched[i] = 0;

        if ((now - zoneLastHit[i]) >= DEBOUNCE_TIME) {

          zoneLastHit[i] = now;

          uint8_t next = (hitHead + 1) & (HIT_BUF_SIZE - 1);
          if (next != hitTail) {
            HitRecord &h = hitBuf[hitHead];
            h.zone = i;
            h.peak = fsrPeak[i];
            h.gx = h.gy = h.gz = 0;
            hitHead = next;
          }

          fsrPeak[i] = 0;
          paddleLightOffAt = now + 150;
        }
      }
    }
  }

  // =========================
  // NON-BLOCKING HOUSEKEEPING
  // =========================
  BLE.poll();

  if (now - lastBLESend >= BLE_SEND_INTERVAL) {
    lastBLESend = now;
    sendBLE();
  }

  if (!BLE.connected() && !sessionActive) {
    if (now - lastBlink >= BLINK_INTERVAL) {
      lastBlink = now;
      blinkState = !blinkState;
      blinkState ? handleWhite() : handleOff();
    }
  } else if (!sessionActive) {
    handleBlue();
  }

  if (paddleLightOffAt && now >= paddleLightOffAt) {
    paddleOff();
    paddleLightOffAt = 0;
  }

  if (digitalRead(interrupt_1) == LOW) {
    doShutdown();
  }

  if (cmdCharacteristic.written()) {
    String cmd = cmdCharacteristic.value();
    cmd.trim();

    if      (cmd == "START")    { doStart(); }
    else if (cmd == "RESET")    { doReset(); }
    else if (cmd == "SHUTDOWN") { doShutdown(); }

    cmdCharacteristic.writeValue("");
  }
}