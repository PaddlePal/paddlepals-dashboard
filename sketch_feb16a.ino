#include <ArduinoBLE.h>

// Define the 6 analog pins used for the zones
const int zonePins[6] = {A6, A5, A4, A3, A2, A1}; // A1 is zone 6... A2 is zone 5 so on
const int numZones = 6;

// const int zonePins[2] = {A0, A2};
// const int numZones = 2;

// Set a threshold to ignore minor electrical noise when no pressure is applied.
// Adjust this number up or down depending on your specific voltage divider.
const int threshold = 75; 

//LED digital out pin
// const int LED_PIN_G = 2;
// const int LED_PIN_B = 3;
// const int LED_PIN_R = 4;
const int H_LED_PIN_G = 5;
const int H_LED_PIN_R = 6;
const int H_LED_PIN_B = 7;

const int button1 = 8;
const int kill_switch = 9; //kill
const int interrupt_1 = 10;

// ----------- BLE Setup -----------
BLEService fsrService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
BLEStringCharacteristic fsrCharacteristic(
  "beb5483e-36e1-4688-b7f5-ea07361b26a8",
  BLERead | BLENotify,
  100
);

int loop_delay = 1; //in ms
int light_timer = 0;

// ----------- Timing Control -----------
unsigned long lastBLEsend = 0;
const int BLE_INTERVAL = 100; // ms → ~50 Hz BLE updates

String dataString = "";

void setup() {
  Serial.begin(9600);
  Serial.println("FSR Zone Monitoring Started...");
  
  // pinMode(LED_PIN_R, OUTPUT);
  // pinMode(LED_PIN_G, OUTPUT);
  pinMode(H_LED_PIN_G, OUTPUT);

  // ----------- BLE INIT -----------
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("FSR_Reader");
  BLE.setAdvertisedService(fsrService);

  fsrService.addCharacteristic(fsrCharacteristic);
  BLE.addService(fsrService);

  fsrCharacteristic.writeValue("Waiting...");

  BLE.advertise();
  Serial.println("BLE FSR Reader Ready...");
}

void loop() {

  // This keeps BLE stack alive (IMPORTANT)
  BLE.poll();

  // LED timing control
  if(light_timer <= 0){
    digitalWrite(H_LED_PIN_G, LOW); // led off
    light_timer = 0;
  }
  if(light_timer > 0){
    light_timer -= loop_delay;
  }

  //dataString = "";

  // ----------- FAST ANALOG POLLING -----------
  for (int i = 0; i < numZones; i++) {

    int forceValue = analogRead(zonePins[i]);
    
    if (forceValue > threshold) {
      Serial.print("Zone ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(forceValue);

      dataString += "Zone ";
      dataString += String(i + 1);
      dataString += ": ";
      dataString += String(forceValue);
      dataString += "\n";

      digitalWrite(H_LED_PIN_G, HIGH); //led on
      light_timer = 1000; // 1s
    }
  }

  // ----------- THROTTLED BLE SEND -----------
  if (millis() - lastBLEsend >= BLE_INTERVAL) {
    lastBLEsend = millis();

    if (BLE.connected() && dataString.length() > 0) {
      fsrCharacteristic.writeValue(dataString);
      dataString = ""; // ← KEY FIX

    }
  }

  delay(loop_delay); 
}