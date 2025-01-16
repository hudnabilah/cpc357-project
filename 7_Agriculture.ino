/*
  ESP32 publish telemetry data to VOne Cloud (Agriculture)
*/

#include "VOneMqttClient.h"
#include "DHT.h"
#include <ESP32Servo.h>

// Moisture range values
int MinMoistureValue = 4095;
int MaxMoistureValue = 1800;
int MinMoisture = 0;
int MaxMoisture = 100;
int Moisture = 0;

// Define device IDs
const char* DHT11Sensor = "33cb803d-a0bc-4889-bf82-a4a1a5752d7c";     // Replace with YOUR deviceID for the DHT11 sensor
const char* MoistureSensor = "b2869c48-87d5-4316-ac54-eecb9fa89fa0";  // Replace with YOUR deviceID for the moisture sensor
const char* ServoMotor = "6325f25b-4dd8-483e-99eb-8d64c25f2a96";
const char* Relay ="52fdc5f5-5266-438b-8a59-0d5bd9808685";
// Pin definitions
const int dht11Pin = 42;       // Right side Maker Port
const int moisturePin = A2;    // Middle Maker Port
const int servoPin = 5;        // Servo motor pin
const int relayPin = 39;       // Relay pin (controls water pump)

// Input sensor
#define DHTTYPE DHT11
DHT dht(dht11Pin, DHTTYPE);
Servo roofServo;

// Create an instance of VOneMqttClient
VOneMqttClient voneClient;

// Last message time
unsigned long lastMsgTime = 0;
const unsigned long CUSTOM_INTERVAL = 5000; // 5 seconds interval

void setup_wifi() {
  delay(10);
  // Connecting to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);

  setup_wifi();
  voneClient.setup();

  // Sensor and actuator initialization
  dht.begin();
  roofServo.attach(servoPin);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // Ensure relay is off initially
}

void loop() {
  if (!voneClient.connected()) {
    voneClient.reconnect();
    voneClient.publishDeviceStatusEvent(DHT11Sensor, true);
    voneClient.publishDeviceStatusEvent(MoistureSensor, true);
    voneClient.publishDeviceStatusEvent(ServoMotor, true);
    voneClient.publishDeviceStatusEvent(Relay, true);
  }
  voneClient.loop();

  unsigned long cur = millis();
  if (cur - lastMsgTime > INTERVAL) {
    lastMsgTime = cur;

    // Read DHT11 sensor values
    float h = dht.readHumidity();
    int t = dht.readTemperature();

    JSONVar payloadObject;
    payloadObject["Humidity"] = h;
    payloadObject["Temperature"] = t;
    voneClient.publishTelemetryData(DHT11Sensor, payloadObject);

    // Humidity-based roof control
    if (h > 85) { // High humidity, open roof
      roofServo.write(90); // Open roof
      Serial.println("High humidity detected. Roof opened.");
    } else if (h < 50) { // Low humidity, close roof
      roofServo.write(0); // Close roof
      Serial.println("Low humidity detected. Roof closed.");
    } else {
      roofServo.write(45); // Neutral position
      Serial.println("Optimal humidity. Roof in neutral position.");
    }

    // Read soil moisture values
    int sensorValue = analogRead(moisturePin);
    Moisture = map(sensorValue, MinMoistureValue, MaxMoistureValue, MinMoisture, MaxMoisture);
    voneClient.publishTelemetryData(MoistureSensor, "Soil moisture", Moisture);

    // Moisture-based irrigation control
    if (Moisture < 30) { // Low soil moisture
      digitalWrite(relayPin, HIGH); // Turn on water pump
      Serial.println("Low soil moisture detected. Water pump activated.");
    } else {
      digitalWrite(relayPin, LOW); // Turn off water pump
      Serial.println("Soil moisture adequate. Water pump deactivated.");
    }
  }
}
