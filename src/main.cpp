#define RELAY_PIN 4
#define SDA_PIN 21
#define SCL_PIN 22

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <Wire.h>
#include <MFRC522.h>
#include <SPI.h>
#include "OTA_Update_Callback.h"
constexpr char WIFI_SSID[] = "iPhone";
constexpr char WIFI_PASSWORD[] = "07072004";

constexpr char TOKEN[] = "HFzicWqo8stiFQG6006k";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr char CURRENT_FIRMWARE_TITLE[] = "ESP32_Firmware";
constexpr char CURRENT_FIRMWARE_VERSION[] = "1.0.0";

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

#define SS_PIN 10
#define RST_PIN 9
MFRC522 rfid(SS_PIN, RST_PIN);

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoardSized<8, 2, 5, 1> tb(mqttClient, MAX_MESSAGE_SIZE);
OTA_Update_Callback ota;
float voltage = 0.0;
float current = 0.0;
float power = 0.0;

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

const bool reconnect() {
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }
  InitWiFi();
  return true;
}

TaskHandle_t wifiTaskHandle;
TaskHandle_t tbTaskHandle;
TaskHandle_t rfidTaskHandle;
SemaphoreHandle_t acDataMutex;

void connectwf(void *parameter) {
  while (1) {
    if(!reconnect()){
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void connectTB(void *parameter) {
  while (1) {
    if(WiFi.status() == WL_CONNECTED){
      if(!tb.connected()){
        Serial.print("Connecting to: ");
        Serial.print(THINGSBOARD_SERVER);
        Serial.print(" with token ");
        Serial.println(TOKEN);
        if(!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
          Serial.println("Failed to connect");
        } else {
          Serial.println("Connected!");
          tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
          ota.Set_Firmware_Title(CURRENT_FIRMWARE_TITLE);
          ota.Set_Firmware_Version(CURRENT_FIRMWARE_VERSION);
          tb.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION);
          tb.Subscribe_Firmware_Update(ota);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

String getUIDString() {
  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    uid += String(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  return uid;
}

void taskRFID(void *parameter) {
  while (1) {
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    String uid = getUIDString();
    Serial.print("Detected RFID UID: ");
    Serial.println(uid);

    // Kiểm tra UID hợp lệ (có thể thay bằng danh sách UID động)
    if (uid == "12345678") {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Relay ON");
      tb.sendTelemetryData("relay", true);
    } else {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Relay OFF");
      tb.sendTelemetryData("relay", false);
    }

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void taskACMeasure(void *parameter) {
  while (1) {
    Wire.beginTransmission(0x50);  // HLW8032 default I2C addr
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(0x50, 24);

    if (Wire.available() == 24) {
      byte buffer[24];
      for (int i = 0; i < 24; i++) {
        buffer[i] = Wire.read();
      }

      uint32_t voltage_raw = (buffer[4] << 16) | (buffer[5] << 8) | buffer[6];
      uint32_t current_raw = (buffer[7] << 16) | (buffer[8] << 8) | buffer[9];
      uint32_t power_raw   = (buffer[10] << 16) | (buffer[11] << 8) | buffer[12];

      float new_voltage = voltage_raw / 100.0;
      float new_current = current_raw / 1000.0;
      float new_power   = power_raw / 100.0;

      if (xSemaphoreTake(acDataMutex, pdMS_TO_TICKS(100))) {
        voltage = new_voltage;
        current = new_current;
        power = new_power;
        xSemaphoreGive(acDataMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(3000));  // 3s
  }
}

void taskSendACData(void *parameter) {
  while (1) {
    if (tb.connected()) {
      float v, c, p;
      if (xSemaphoreTake(acDataMutex, pdMS_TO_TICKS(100))) {
        v = voltage;
        c = current;
        p = power;
        xSemaphoreGive(acDataMutex);
      }

      tb.sendTelemetryData("voltage", v);
      tb.sendTelemetryData("current", c);
      tb.sendTelemetryData("power", p);
    }

    vTaskDelay(pdMS_TO_TICKS(5000));  // mỗi 5s gửi 1 lần
  }
}


void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  SPI.begin();
  rfid.PCD_Init();
  Serial.println("RFID Ready");
  Wire.begin(SDA_PIN, SCL_PIN);
  acDataMutex = xSemaphoreCreateMutex();
  InitWiFi();

  xTaskCreatePinnedToCore(connectwf, "WiFi Task", 4096, NULL, 1, &wifiTaskHandle, 1);
  xTaskCreatePinnedToCore(connectTB, "TB Task", 4096, NULL, 1, &tbTaskHandle, 1);
  xTaskCreatePinnedToCore(taskRFID, "RFID Task", 4096, NULL, 1, &rfidTaskHandle, 1);
  xTaskCreatePinnedToCore(taskACMeasure, "AC Read Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskSendACData, "Send AC Task", 4096, NULL, 1, NULL, 1);
}

void loop() {} 
