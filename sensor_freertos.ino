#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <Adafruit_HMC5883_U.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define DHTTYPE DHT11
#define DHTPin 17
DHT dht(DHTPin, DHTTYPE);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

const char* ssid = "Tom Ken";
const char* password = "quy132003";
const char* mqtt_server = "192.168.134.15";


WiFiClient espClient;
PubSubClient client(espClient);

// Biến toàn cục cho dữ liệu cảm biến
float lastHumidity = 0.0;
float lastTemperature = 0.0;
float lastCDDT = 0.0;
bool dataReady = false;

// Semaphore để đồng bộ truy cập client MQTT
SemaphoreHandle_t xMutex;

// Hàm kết nối WiFi
void setup_wifi() {
  delay(10);
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

// Hàm kết nối lại MQTT
void reconnectNodeRed() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection to Node-RED...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected to Node-RED broker");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

// Task đọc cảm biến
void SensorTask(void *pvParameters) {
  while (1) {
    sensors_event_t event;
    mag.getEvent(&event);
    float humidity = dht.readHumidity();
    float temperatureC = dht.readTemperature();
    float CDDT = sqrt(pow(event.magnetic.x, 2) + pow(event.magnetic.y, 2) + pow(event.magnetic.z, 2));

    if (isnan(humidity) || isnan(temperatureC)) {
      Serial.println("Failed to read from DHT sensor!");
    } else {
      // Lưu dữ liệu vào biến toàn cục
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        lastHumidity = humidity;
        lastTemperature = temperatureC;
        lastCDDT = CDDT;
        dataReady = true;
        xSemaphoreGive(xMutex);
      }

      // In dữ liệu ra Serial
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
      Serial.print("Temperature: ");
      Serial.print(temperatureC);
      Serial.println(" ºC");
      Serial.print("CDDT: ");
      Serial.println(CDDT);
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS); // Đọc mỗi 5 giây
  }
}

// Task gửi dữ liệu MQTT
void MQTTTask(void *pvParameters) {
  while (1) {
    if (!client.connected()) {
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        reconnectNodeRed();
        xSemaphoreGive(xMutex);
      }
    }
    client.loop();

    if (dataReady) {
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        // Gửi dữ liệu qua MQTT
        client.publish("room/magnetic", String(lastCDDT).c_str());
        delay(100);
        client.publish("room/temperature", String(lastTemperature).c_str());
        delay(100);
        client.publish("room/humidity", String(lastHumidity).c_str());
        Serial.println("Data sent to Node-RED");
        dataReady = false; // Đặt lại cờ
        xSemaphoreGive(xMutex);
      }
    }

    vTaskDelay(50 / portTICK_PERIOD_MS); // Kiểm tra thường xuyên
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883 sensor, check wiring!");
    while (1);
  }

  // Tạo semaphore
  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL) {
    Serial.println("Failed to create mutex!");
    while (1);
  }

  // Tạo các task FreeRTOS
  xTaskCreatePinnedToCore(SensorTask,  "SensorTask", 4096,NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(MQTTTask, "MQTT Task", 4096, NULL, 2, NULL, 1);

}

void loop() {
  // Vòng loop trống vì mọi thứ được xử lý trong task
}