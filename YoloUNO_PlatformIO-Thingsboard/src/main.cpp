#define LED_PIN 48
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <Ticker.h>
#include "button.h"
// Task handles
TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;
TaskHandle_t Task3Handle = NULL;
TaskHandle_t Task4Handle = NULL;
Ticker buttonTicker;

#define DHTPIN 6
#define DHTTYPE DHT11

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

constexpr char WIFI_SSID[] = "HCMUT09";
constexpr char WIFI_PASSWORD[] = "12345678";

constexpr char TOKEN[] = "tQzF6d8OEpPxJAN4ZgCX";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";

volatile bool attributesChanged = false;
volatile bool state = false;
volatile int ledMode = 0;
volatile bool ledState = false;

constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U;

uint32_t previousStateChange;

constexpr int16_t telemetrySendInterval = 10000U;
uint32_t previousDataSend;


constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {  // Sửa size thành 6
  LED_STATE_ATTR,
  BLINKING_INTERVAL_ATTR,
};

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600, 60000); // Adjust the offset for your timezone (e.g., 7 hours for GMT+7)

// Hàm xử lý RPC để bật/tắt LED
RPC_Response setLedSwitchState(const RPC_Data &data) {
  Serial.println("Received RPC to set LED state");
  bool newState = data; // Lấy trạng thái từ RPC
  Serial.print("Switch state change: ");
  Serial.println(newState);

  // Cập nhật trạng thái LED
  digitalWrite(LED_PIN, newState);
  ledState = newState; // Lưu trạng thái LED hiện tại
  tb.sendAttributeData(LED_STATE_ATTR, ledState);
  // Trả về phản hồi cho ThingsBoard
  return RPC_Response("setLedSwitchValue", newState);
}

// Hàm xử lý RPC để lấy trạng thái LED
RPC_Response getLedSwitchState(const RPC_Data &data) {
  Serial.println("Received RPC to get LED state");
  // Trả về trạng thái hiện tại của LED
  
  return RPC_Response("getLedSwitchValue", ledState);
}

const std::array<RPC_Callback, 2U> callbacks = {
  RPC_Callback{ "setLedSwitchValue", setLedSwitchState },
  RPC_Callback{ "getLedSwitchValue", getLedSwitchState }
};

void processSharedAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0) {
      const uint16_t new_interval = it->value().as<uint16_t>();
      if (new_interval >= BLINKING_INTERVAL_MS_MIN && new_interval <= BLINKING_INTERVAL_MS_MAX) {
        blinkingInterval = new_interval;
        Serial.print("Blinking interval is set to: ");
        Serial.println(new_interval);
      }
    } else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
      ledState = it->value().as<bool>();
      digitalWrite(LED_PIN, ledState);
      Serial.print("LED state is set to: ");
      Serial.println(ledState);
      // Update and print the current time
      timeClient.update();
      Serial.print("at: ");
      Serial.println(timeClient.getFormattedTime());
    } 
  }
}

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

void InitWiFi() {
  Serial.println("Connecting to AP ...");// Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
  timeClient.begin(); // Start the NTP client
  timeClient.update(); // Update the time
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

void Task1(void *pvParameters) {
  while (1) {
    if (!reconnect()) {
      return;
    }
    
    if (!tb.connected()) {
      Serial.print("Connecting to: ");
      Serial.print(THINGSBOARD_SERVER);
      Serial.print(" with token ");
      Serial.println(TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        Serial.println("Failed to connect");
        return;
      }
    
      tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
    
      Serial.println("Subscribing for RPC...");
      if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
        Serial.println("Failed to subscribe for RPC");
        return;
      }
    
      if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
        Serial.println("Failed to subscribe for shared attribute updates");
        return;
      }
    
      Serial.println("Subscribe done");
    
      if (!tb.Shared_Attributes_Request(attribute_shared_request_callback)) {
        Serial.println("Failed to request for shared attributes");
        return;
      }
     
    }

    // Periodically send WiFi-related attributes
    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("channel", WiFi.channel());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());
    
    vTaskDelay(pdMS_TO_TICKS(5000)); 
  }
}

void Task2(void *pvParameters) {
  while (1) {
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    float temperature = event.temperature;
    dht.humidity().getEvent(&event);
    float humidity = event.relative_humidity;

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT11 sensor!");
    } else {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" °C, Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");

      tb.sendTelemetryData("temperature", temperature);
      tb.sendTelemetryData("humidity", humidity);
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void Task3(void *pvParameters) {
while (1) {
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck >= 100) { // Kiểm tra mỗi 100ms
    lastCheck = millis();
    for (int i = 0; i < NUM_BUTTONS; i++) {
      if (isButtonPressed(i)) {
        ledState = !ledState;
        tb.sendAttributeData(LED_STATE_ATTR, ledState);
        digitalWrite(LED_PIN, ledState);
        Serial.printf("State: %d \n", ledState);
        Serial.printf("Button %d pressed (short)\n", i);
      }
      if (isButtonLongPressed(i)) {
        Serial.printf("Button %d pressed (long)\n", i);
      }
    }
  }
}
}
// Hàm callback để đọc trạng thái nút
void buttonTask() {
getKeyInput();
}

void setup() {
  delay(1000);
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial.println("Helloworld...");
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
buttonTicker.attach_ms(10, buttonTask);
  delay(1000);
  InitWiFi();

  Wire.begin(SDA_PIN, SCL_PIN);
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  xTaskCreate(Task1, "MQTTTask", 10000, NULL, 1, &Task1Handle);
  xTaskCreate(Task2, "DHT20Task", 10000, NULL, 1, &Task2Handle);
  xTaskCreate(Task3, "ButtonTask", 10000, NULL, 1, &Task3Handle);
}

void loop() {
 tb.loop();
}