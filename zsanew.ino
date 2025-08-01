#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"
#include "SSD1306.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- I2C Configuration for XIAO ESP32C3 ---
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7

// Sensor and display objects
MAX30105 particleSensor;
SSD1306 display(0x3C, I2C_SDA_PIN, I2C_SCL_PIN);
Adafruit_MPU6050 mpu;

// WiFi and MQTT config
const char* ssid = "Team09";
const char* password = "H@ckTe@m)(";
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "zsa/sensor/data";

WiFiClient espClient;
PubSubClient client(espClient);

// Sensor data buffers
uint32_t irBuffer[100], redBuffer[100];
int32_t spo2, heartRate;
int8_t validSPO2, validHeartRate;
float temperatureC;
sensors_event_t a, g, temp_mpu;

// Timing
unsigned long lastDisplayUpdate = 0;
unsigned long lastMQTTSend = 0;
const unsigned long displayInterval = 500;
const unsigned long mqttInterval = 10000;

// --- WiFi ---
void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi connection failed.");
  }
}

// --- MQTT reconnect (non-blocking) ---
void reconnect() {
  if (client.connected()) return; // Already connected, no need to reconnect

  Serial.print("Attempting MQTT connection...");

  // Get MAC address as unique client ID
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[13]; // 12 hex digits + null terminator
  snprintf(macStr, sizeof(macStr), "%02X%02X%02X%02X%02X%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  String clientId = "ESP32Client-" + String(macStr);

  if (client.connect(clientId.c_str())) {
    Serial.println("connected with clientId: " + clientId);
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" will retry");
    // No delay here to avoid blocking — retry next loop iteration
  }
}

void sendJSON() {
  StaticJsonDocument<512> doc;

  doc["deviceId"] = "ESP32-ZSA-001";
  doc["heartRate"] = validHeartRate ? heartRate : -1;
  doc["spo2"] = validSPO2 ? spo2 : -1;
  doc["temperatureC"] = temperatureC;
  doc["accelX"] = a.acceleration.x;
  doc["accelY"] = a.acceleration.y;
  doc["accelZ"] = a.acceleration.z;
  doc["gyroZ"] = g.gyro.z;
  doc["status"] = "ok";

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  if (client.publish(mqtt_topic, buffer, n)) {
    Serial.println("MQTT message sent:");
    Serial.println(buffer);
  } else {
    Serial.println("MQTT message failed to send");
  }
}

// --- OLED Display ---
void updateOLED() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, validHeartRate ? "HR: " + String(heartRate) + " bpm" : "HR: Invalid");

  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 18, validSPO2 ? "SpO2: " + String(spo2) + " %" : "SpO2: Invalid");
  display.drawString(0, 28, "Temp: " + String(temperatureC, 1) + " C");
  display.drawString(0, 38, "AccX: " + String(a.acceleration.x, 1));
  display.drawString(64, 38, "AccY: " + String(a.acceleration.y, 1));
  display.drawString(0, 48, "AccZ: " + String(a.acceleration.z, 1));
  display.drawString(64, 48, "RotZ: " + String(g.gyro.z, 1));
  display.display();
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // OLED
  if (!display.init()) {
    Serial.println(F("OLED init failed!"));
    while (1);
  }
  display.flipScreenVertically();

  // MPU6050
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 not found!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // MAX30105
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found!");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  particleSensor.enableDIETEMPRDY();

  // WiFi & MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  display.drawString(0, 0, "Ready!");
  display.display();
  delay(1000);
}

// --- Loop ---
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Collect MAX30105 data
  int bufferLength = 100;
  for (int i = 0; i < bufferLength; i++) {
    unsigned long readStart = millis();
    while (!particleSensor.available()) {
      particleSensor.check();
      if (millis() - readStart > 2000) return;
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  temperatureC = particleSensor.readTemperature();
  mpu.getEvent(&a, &g, &temp_mpu);

  unsigned long currentMillis = millis();

  // Update OLED
  if (currentMillis - lastDisplayUpdate > displayInterval) {
    lastDisplayUpdate = currentMillis;
    updateOLED();
  }

  // Send MQTT JSON
  if (currentMillis - lastMQTTSend > mqttInterval) {
    lastMQTTSend = currentMillis;
    sendJSON();
  }

  // Debug to Serial
  Serial.print("HR="); Serial.print(heartRate);
  Serial.print(", SpO2="); Serial.print(spo2);
  Serial.print(", TempC="); Serial.print(temperatureC);
  Serial.print(", AccX="); Serial.print(a.acceleration.x);
  Serial.print(", AccY="); Serial.print(a.acceleration.y);
  Serial.print(", AccZ="); Serial.print(a.acceleration.z);
  Serial.print(", GyroZ="); Serial.print(g.gyro.z);
  Serial.println();
}
