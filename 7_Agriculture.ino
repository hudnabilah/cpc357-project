#include "VOneMqttClient.h"
#include "DHT.h"
#include <ESP32Servo.h>

// Moisture range values
int MinMoistureValue = 4095;
int MaxMoistureValue = 1800;
int MinMoisture = 0;
int MaxMoisture = 100;
int Moisture = 0;
unsigned long lastHumidityTime = 0;
unsigned long lastMoistureTime = 0;
const unsigned long HUMIDITY_INTERVAL = 30000; //30 seconds in miliseconds
const unsigned long MOISTURE_INTERVAL = 30000; //30 seconds in miliseconds

// Define device IDs
const char* DHT11Sensor = "33cb803d-a0bc-4889-bf82-a4a1a5752d7c";    
const char* MoistureSensor = "b2869c48-87d5-4316-ac54-eecb9fa89fa0"; 
const char* ServoMotor = "5c618bff-9d78-4e1f-9253-743509646b62";
const char* Relay ="bb952f6c-5320-40fa-ac1b-61b548736e1f";

// Pin definitions
const int dht11Pin = 42;      
const int moisturePin = A2;    
const int servoPin = 5;        
const int relayPin = 39;       

Servo roofServo;
#define DHTTYPE DHT11
DHT dht(dht11Pin, DHTTYPE);
VOneMqttClient voneClient;

void setup_wifi() {
  delay(10);
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

void triggerActuator_callback(const char* actuatorDeviceId, const char* actuatorCommand) {
    String errorMsg = "";
    JSONVar commandObjct = JSON.parse(actuatorCommand);
    JSONVar keys = commandObjct.keys();

    if (String(actuatorDeviceId) == ServoMotor) {
        String key = "";
        JSONVar commandValue = "";
        for (int i = 0; i < keys.length(); i++) {
            key = (const char*)keys[i];
            commandValue = commandObjct[keys[i]];
        }
        int angle = (int)commandValue;
        roofServo.write(angle);
        voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), true);
    } 
    else if (String(actuatorDeviceId) == Relay) {
        String state = (const char*)commandObjct["state"];
        if (state == "ON") {
            digitalWrite(relayPin, HIGH);
            Serial.println("Water pump activated via command.");
        } else if (state == "OFF") {
            digitalWrite(relayPin, LOW);
            Serial.println("Water pump deactivated via command.");
        }
        voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), true);
    }
    else {
        errorMsg = "No actuator found";
        voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), false);
    }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  voneClient.setup();
  voneClient.registerActuatorCallback(triggerActuator_callback);
  dht.begin();
  roofServo.attach(servoPin);
  roofServo.write(0);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
}

void loop() {
  if (!voneClient.connected()) {
    voneClient.reconnect();
    voneClient.publishDeviceStatusEvent(DHT11Sensor, true);
    voneClient.publishDeviceStatusEvent(MoistureSensor, true);
  }
  voneClient.loop();

  unsigned long currentTime = millis();

  // Handle humidity sensor every 5 minutes
  if (currentTime - lastHumidityTime > HUMIDITY_INTERVAL) {
    lastHumidityTime = currentTime;
    
    // Read DHT11 sensor values
    float h = dht.readHumidity();
    int t = dht.readTemperature();

    JSONVar payloadObject;
    payloadObject["Humidity"] = h;
    payloadObject["Temperature"] = t;
    voneClient.publishTelemetryData(DHT11Sensor, payloadObject);

    // Humidity-based roof control
    if (h > 75 || h < 65) {
      roofServo.write(60);
      Serial.println("High or Low humidity detected. Roof opened.");
      voneClient.publishActuatorStatusEvent(ServoMotor, "{\"angle\":60}", "", true);
    } else if (h == 75 || h == 65) {
      roofServo.write(30);
      Serial.println("Min Max humidity detected. Roof in Neural Position.");
      voneClient.publishActuatorStatusEvent(ServoMotor, "{\"angle\":30}", "", true);
    } else {
      roofServo.write(0);
      Serial.println("Optimal humidity. Roof closed.");
      voneClient.publishActuatorStatusEvent(ServoMotor, "{\"angle\":0}", "", true);
    }
  }

  // Handle moisture sensor every 15 minutes
  if (currentTime - lastMoistureTime > MOISTURE_INTERVAL) {
    lastMoistureTime = currentTime;
    
    // Read soil moisture values
    int sensorValue = analogRead(moisturePin);
    Moisture = map(sensorValue, MinMoistureValue, MaxMoistureValue, MinMoisture, MaxMoisture);
    voneClient.publishTelemetryData(MoistureSensor, "Soil moisture", Moisture);

    // Moisture-based irrigation control
    if (Moisture < 70) {
      digitalWrite(relayPin, HIGH);
      Serial.println("Low soil moisture detected. Water pump activated.");
      voneClient.publishActuatorStatusEvent(Relay, "{\"state\":\"ON\"}", "", true);
    } else {
      digitalWrite(relayPin, LOW);
      Serial.println("Soil moisture adequate. Water pump deactivated.");
      voneClient.publishActuatorStatusEvent(Relay, "{\"state\":\"OFF\"}", "", true);
    }
  }
}
