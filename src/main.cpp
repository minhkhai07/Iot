#define LED_PIN 48
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>

constexpr char WIFI_SSID[] = "iPhone";
constexpr char WIFI_PASSWORD[] = "07072004";

constexpr char TOKEN[] = "g14piik3js3eq55lkbf2";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";

volatile bool attributesChanged = false;
volatile int ledMode = 0;
volatile bool ledState = false;

constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U;

uint32_t previousStateChange;

constexpr int16_t telemetrySendInterval = 10000U;
uint32_t previousDataSend;

constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  BLINKING_INTERVAL_ATTR
};


WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);
DHT20 dht20;

// RPC_Response setLedSwitchState(const RPC_Data &data) {
//   Serial.println("Received Switch state");
//   bool newState = data;
//   Serial.print("Switch state change: ");
//   Serial.println(newState);
//   digitalWrite(LED_PIN, newState);
//   attributesChanged = true;
//   return RPC_Response("setLedSwitchValue", newState);
// }

// const std::array<RPC_Callback, 1U> callbacks = {
// RPC_Callback{ "setLedSwitchValue", setLedSwitchState }
// };

// void processSharedAttributes(const Shared_Attribute_Data &data) {
// for (auto it = data.begin(); it != data.end(); ++it) {
//   if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0) {
//     const uint16_t new_interval = it->value().as<uint16_t>();
//     if (new_interval >= BLINKING_INTERVAL_MS_MIN && new_interval <= BLINKING_INTERVAL_MS_MAX) {
//       blinkingInterval = new_interval;
//       Serial.print("Blinking interval is set to: ");
//       Serial.println(new_interval);
//     }
//   } else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
//     ledState = it->value().as<bool>();
//     digitalWrite(LED_PIN, ledState);
//     Serial.print("LED state is set to: ");
//     Serial.println(ledState);
//   }
// }
// attributesChanged = true;
// }

// const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
// const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

const bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }
  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}
TaskHandle_t wifiTaskHandle;
TaskHandle_t tbTaskHandle;
TaskHandle_t TaskDHT20Handle;
TaskHandle_t TaskLEDHandle;
TaskHandle_t TaskSendAttributeHandle;

// --------------------------- Task: Kết nối WiFi ---------------------------
void connectwf(void *parameter) {
  while (1) {
    if(!reconnect()){
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(10000));  // Kiểm tra lại mỗi 10 giây
  }
}

// --------------------------- Task: Kết nối ThingsBoard ---------------------------
void connectTB(void *parameter) {
  while (1) {
    if(WiFi.status() == WL_CONNECTED){
      if(!tb.connected()){
        Serial.print("Connecting to: ");
        Serial.print(THINGSBOARD_SERVER);
        Serial.print(" with token ");
        Serial.println(TOKEN);
      }
      if(!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        Serial.println("Failed to connect");
      } 
      // else {
      //   Serial.println("Connected !");
      //   tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());

      //   Serial.println("Subscribing for RPC...");
      //   if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
      //     Serial.println("Failed to subscribe for RPC");
      //     return;
      //   }

      //   if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
      //     Serial.println("Failed to subscribe for shared attribute updates");
      //     return;
      //   }

      //   Serial.println("Subscribe done");

      //   if (!tb.Shared_Attributes_Request(attribute_shared_request_callback)) {
      //     Serial.println("Failed to request for shared attributes");
      //     return;
      //   }
      // }
    }
    vTaskDelay(pdMS_TO_TICKS(5000));  // Kiểm tra kết nối mỗi 5 giây
  }
}

// --------------------------- Task: Đọc cảm biến DHT20 ---------------------------
void taskDHT20Sensor(void *parameter) {
  while (1) {
    dht20.read();
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();

    if (!isnan(temperature) && !isnan(humidity)) {
      Serial.printf("Temperture: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);
      tb.sendTelemetryData("temperature", temperature);
      tb.sendTelemetryData("humidity", humidity);
    } else {
      Serial.println("Lỗi đọc cảm biến DHT20!");
    }

    vTaskDelay(pdMS_TO_TICKS(2000));  // Đọc mỗi 2 giây
  }
}

// --------------------------- Task: Điều khiển LED ---------------------------
void taskLEDBlink(void *parameter) {
  while (1) {
    if (attributesChanged) {
      attributesChanged = false;
      tb.sendAttributeData(LED_STATE_ATTR, digitalRead(LED_PIN));
    }
    if (ledMode == 1) {  // Kiểm tra trước khi thay đổi trạng thái LED
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      Serial.printf("LED state: %d\n", ledState);
      tb.sendAttributeData("ledState", ledState);
    }
    vTaskDelay(pdMS_TO_TICKS(blinkingInterval));
  }
}

// --------------------------- Task: Gửi Attribute Data ---------------------------
void taskSendAttributeData(void *parameter) {
  while (1) {
    if (tb.connected()) {
      tb.sendAttributeData("rssi", WiFi.RSSI());
      tb.sendAttributeData("channel", WiFi.channel());
      tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
      tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
      tb.sendAttributeData("ssid", WiFi.SSID().c_str());
    }
    vTaskDelay(pdMS_TO_TICKS(2000));  // Gửi mỗi 22 giây
  }
}

// --------------------------- Setup ---------------------------
void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(LED_PIN, OUTPUT);
  InitWiFi();
  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();
  
  // Tạo các Task của FreeRTOS
  xTaskCreatePinnedToCore(connectwf, "WiFi Task", 4096, NULL, 1, &wifiTaskHandle, 1);
  xTaskCreatePinnedToCore(connectTB, "TB Task", 4096, NULL, 1, &tbTaskHandle, 1);
  xTaskCreatePinnedToCore(taskDHT20Sensor, "DHT20 Task", 4096, NULL, 1, &TaskDHT20Handle, 1);
  xTaskCreatePinnedToCore(taskLEDBlink, "LED Blink Task", 2048, NULL, 1, &TaskLEDHandle, 0);
  xTaskCreatePinnedToCore(taskSendAttributeData, "Attribute Task", 4096, NULL, 1, &TaskSendAttributeHandle, 1);
}

void loop() {
}
