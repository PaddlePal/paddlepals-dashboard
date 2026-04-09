// Comment this out when done developing to remove ALL serial output
//#define DEBUG

#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

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
// HIT RING BUFFER
// =============================================================================
struct HitRecord {
  uint8_t  zone;
  uint8_t  peak;
  float    gx, gy, gz;
};
const int HIT_BUF_SIZE = 16;
HitRecord hitBuf[HIT_BUF_SIZE];
uint8_t hitHead = 0;
uint8_t hitTail = 0;

// =============================================================================
// FSR STATE
// =============================================================================
int           zonePeak[6]    = {0};
bool          zoneActive[6]  = {false};
unsigned long zoneLastHit[6] = {0};

const unsigned long DEBOUNCE_TIME = 150;

// =============================================================================
// IMU
// =============================================================================
float latestGx=0, latestGy=0, latestGz=0;
float latestAx=0, latestAy=0, latestAz=0;
float cachedMx=0, cachedMy=0, cachedMz=0;

unsigned long lastIMURead = 0;
const unsigned long IMU_INTERVAL_MS = 50;

char imuBuf[128];
bool imuReady = false;

// =============================================================================
// SESSION STATE
// =============================================================================
bool sessionActive  = false;
const char* pendingStatus = nullptr;

// =============================================================================
// TIMING
// =============================================================================
unsigned long lastHousekeeping = 0;
unsigned long lastBLESend      = 0;
unsigned long lastBlink        = 0;
unsigned long paddleLightOffAt = 0;
bool          blinkState       = false;
unsigned long buttonDownSince  = 0;
bool          holdFired        = false;

// ── Housekeeping runs every 15 ms — that means the FSR loop runs
//    uninterrupted for ~15 ms at a time.  At ~40 µs per analogRead × 6
//    = ~240 µs per full scan, that's ~62 complete scans per burst,
//    or roughly one scan every 240 µs — well within the 1 ms hit window.
const unsigned long HOUSEKEEPING_INTERVAL = 100;
const unsigned long BLE_SEND_INTERVAL     = 300;
const unsigned long BLINK_INTERVAL        = 500;

// =============================================================================
// LED HELPERS
// =============================================================================
void setLED(int r,int g,int b,int R,int G,int B){ digitalWrite(R,r);digitalWrite(G,g);digitalWrite(B,b); }
void paddleOff()   { setLED(LOW, LOW, LOW, P_R1,P_G1,P_B1); }
void paddleGreen() { setLED(LOW, HIGH,LOW, P_R1,P_G1,P_B1); }
void paddleRed()   { setLED(HIGH,LOW, LOW, P_R1,P_G1,P_B1); }
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
  #ifdef DEBUG
    Serial.println("SESSION STARTED");
  #endif
}

void doReset() {
  sessionActive = false;
  pendingStatus = "STATUS:SESSION_RESET";
  hitHead = hitTail = 0;
  imuReady = false;
  for (int i = 0; i < numZones; i++) {
    zoneActive[i] = false; zonePeak[i] = 0; zoneLastHit[i] = 0;
  }
  latestGx=latestGy=latestGz=0;
  latestAx=latestAy=latestAz=0;
  paddleLightOffAt = 0;
  paddleWhite();
  handleBlue();
  #ifdef DEBUG
    Serial.println("SESSION RESET");
  #endif
}


void doEnd() {
  sessionActive = false;
  pendingStatus = "STATUS:SESSION_ENDED";

  handleBlue();   // or whatever idle color you want

  #ifdef DEBUG
    Serial.println("SESSION ENDED");
  #endif
}


void doShutdown() {
  digitalWrite(kill_switch, LOW);
  #ifdef DEBUG
    Serial.println("SHUTDOWN");
  #endif
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
        "Z%d:%d,GX:%.1f,GY:%.1f,GZ:%.1f\n",
        h.zone + 1, h.peak, h.gx, h.gy, h.gz);
      if (w > 0) pos += w;
      hitTail = (hitTail + 1) & (HIT_BUF_SIZE - 1);
    }
    if (pos > 0) {
      buf[pos] = '\0';
      fsrCharacteristic.writeValue(buf);
      #ifdef DEBUG
        Serial.print("BLE HIT: "); Serial.print(buf);
      #endif
    }
  }

  if (imuReady) {
    imuCharacteristic.writeValue(imuBuf);
    imuReady = false;
  }
}

// =============================================================================
// HOUSEKEEPING — runs every ~15 ms, handles BLE, IMU, buttons, LEDs
// =============================================================================
void doHousekeeping() {
  unsigned long now = millis();

  // ── Shutdown pin ──
  if (digitalRead(interrupt_1) == LOW) { doShutdown(); return; }

  // ── Paddle light auto-off ──
  if (paddleLightOffAt && now >= paddleLightOffAt) {
    paddleOff();
    paddleLightOffAt = 0;
  }

  // ── BLE poll ──
  BLE.poll(0);

  // ── BLE commands ──
  if (cmdCharacteristic.written()) {
    String cmd = cmdCharacteristic.value();
    cmd.trim();
    #ifdef DEBUG
      Serial.print("CMD received: "); Serial.println(cmd);
    #endif
  if      (cmd == "START")    { if (!sessionActive) doStart(); }
  else if (cmd == "RESET")    { doReset(); }
  else if (cmd == "END")      { if (sessionActive) doEnd(); }
  else if (cmd == "SHUTDOWN") { doShutdown(); }
    cmdCharacteristic.writeValue("");
  }

  // ── Connection LED ──
  if (!BLE.connected() && !sessionActive) {
    if (now - lastBlink >= BLINK_INTERVAL) {
      lastBlink = now;
      blinkState = !blinkState;
      blinkState ? handleWhite() : handleOff();
    }
  } else if (!sessionActive) {
    handleBlue();
  }

  // ── BLE send ──
  if (now - lastBLESend >= BLE_SEND_INTERVAL) {
    lastBLESend = now;
    sendBLE();
  }

  // ── Button ──
  bool bp = (digitalRead(button1) == LOW);
  if (bp) {
    if (buttonDownSince == 0) { buttonDownSince = now; holdFired = false; }
    if (!holdFired) {
      if (!sessionActive && BLE.connected() && (now - buttonDownSince >= 1000)) {
        doStart(); holdFired = true;
      }
      if (sessionActive && (now - buttonDownSince >= 2000)) {
        doReset(); holdFired = true;
      }
    }
  } else {
    buttonDownSince = 0;
  }

  // ── IMU — only during session ──
  if (sessionActive && (now - lastIMURead >= IMU_INTERVAL_MS)) {
    lastIMURead = now;
    float gx,gy,gz,ax,ay,az;
    if (IMU.gyroscopeAvailable()    && IMU.readGyroscope(gx,gy,gz) &&
        IMU.accelerationAvailable() && IMU.readAcceleration(ax,ay,az)) {

      latestGx=gx; latestGy=gy; latestGz=gz;
      latestAx=ax; latestAy=ay; latestAz=az;

      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(cachedMx, cachedMy, cachedMz);
      }

      unsigned long t = micros();
      snprintf(imuBuf, sizeof(imuBuf),
        "T:%lu,GX:%.1f,GY:%.1f,GZ:%.1f,AX:%.2f,AY:%.2f,AZ:%.2f,MX:%.1f,MY:%.1f,MZ:%.1f",
        t, gx,gy,gz, ax,ay,az, cachedMx,cachedMy,cachedMz);
      imuReady = true;
    }
  }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  #ifdef DEBUG
    Serial.begin(9600);
    while(!Serial);
    Serial.println("System booting...");
  #endif

  pinMode(P_G1,OUTPUT); pinMode(P_R1,OUTPUT); pinMode(P_B1,OUTPUT);
  pinMode(H_G2,OUTPUT); pinMode(H_R2,OUTPUT); pinMode(H_B2,OUTPUT);
  pinMode(button1,INPUT_PULLUP);
  pinMode(interrupt_1,INPUT_PULLUP);
  pinMode(kill_switch,OUTPUT); digitalWrite(kill_switch,HIGH);
  analogReadResolution(8);

  delay(1000);
  if (!IMU.begin()) {
    #ifdef DEBUG
      Serial.println("IMU failed!");
    #endif
    while(1);
  }

  #ifdef DEBUG
    Serial.print("Gyro sample rate: ");  Serial.println(IMU.gyroscopeSampleRate());
    Serial.print("Accel sample rate: "); Serial.println(IMU.accelerationSampleRate());
    Serial.print("Mag sample rate: ");   Serial.println(IMU.magneticFieldSampleRate());
  #endif

  paddleWhite();

  if (!BLE.begin()) {
    #ifdef DEBUG
      Serial.println("BLE failed!");
    #endif
    while(1);
  }
  BLE.setLocalName("PaddleSensor");
  BLE.setAdvertisedService(paddleService);
  paddleService.addCharacteristic(fsrCharacteristic);
  paddleService.addCharacteristic(imuCharacteristic);
  paddleService.addCharacteristic(cmdCharacteristic);
  BLE.addService(paddleService);
  fsrCharacteristic.writeValue("Waiting...");
  imuCharacteristic.writeValue("Waiting...");
  cmdCharacteristic.writeValue("");
  BLE.advertise();

  #ifdef DEBUG
    Serial.println("BLE ready");
  #endif
}

// =============================================================================
// MAIN LOOP
//
// Structure:
//   1. Check if housekeeping is due (every 15 ms) — if so, do it
//   2. Otherwise, slam through FSR reads as fast as possible
//
// During the ~15 ms between housekeeping windows, the loop does NOTHING
// except analogRead × 6 + peak/hit check.  At ~40 µs per read × 6 pins
// = ~240 µs per full scan, we get ~62 scans per 15 ms burst — roughly
// 4 scans per millisecond, well within the 1 ms dwell window.
// =============================================================================
void loop() {
  unsigned long now = millis();

  // ── Housekeeping break every 15 ms ──
  if (now - lastHousekeeping >= HOUSEKEEPING_INTERVAL) {
    lastHousekeeping = now;
    doHousekeeping();
    return;   // give the full loop iteration to housekeeping, resume scanning next pass
  }

  // ── If not in session, nothing to scan — just idle ──
  if (!sessionActive) return;

  // ══════════════════════════════════════════════════════════════════════
  // FSR HOT PATH — this is the ONLY code that runs between housekeeping
  // ══════════════════════════════════════════════════════════════════════

  // Sample all 6 pins
  for (int i = 0; i < numZones; i++) {
    int r = analogRead(zonePins[i]);
    if (r > zonePeak[i]) zonePeak[i] = r;
  }

  // Evaluate hits
  for (int i = 0; i < numZones; i++) {
    bool over     = (zonePeak[i] > threshold);
    bool debounce = (now - zoneLastHit[i]) >= DEBOUNCE_TIME;

    if (over && !zoneActive[i] && debounce) {
      zoneActive[i]  = true;
      zoneLastHit[i] = now;

      // Push to ring buffer — no string work
      uint8_t next = (hitHead + 1) & (HIT_BUF_SIZE - 1);
      if (next != hitTail) {
        HitRecord &h = hitBuf[hitHead];
        h.zone = i;
        h.peak = zonePeak[i];
        h.gx   = latestGx;
        h.gy   = latestGy;
        h.gz   = latestGz;
        hitHead = next;
      }

      // LED — direct GPIO, no function call overhead
      if (i == 0) {
        digitalWrite(P_R1, LOW); digitalWrite(P_G1, HIGH); digitalWrite(P_B1, LOW);
      } else {
        digitalWrite(P_R1, HIGH); digitalWrite(P_G1, LOW); digitalWrite(P_B1, LOW);
      }
      paddleLightOffAt = now + 1200;

      #ifdef DEBUG
        Serial.print("HIT Z"); Serial.print(i+1);
        Serial.print(" val="); Serial.println(zonePeak[i]);
      #endif
    }

    if (!over) zoneActive[i] = false;
    zonePeak[i] = 0;
  }
}
