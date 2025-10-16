#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// WiFi & MQTT
// const char* ssid = "1";
// const char* password = "quy132003";
// const char* mqtt_server = "192.168.134.15";
const char* ssid = "UTC_A2";
const char* password = "";
const char* mqtt_server = "10.90.110.181";
WiFiClient espClient;
PubSubClient client(espClient);

// RFID
#define SS_PIN 5
#define RST_PIN 17
MFRC522 rfid(SS_PIN, RST_PIN);

// INA219
Adafruit_INA219 ina219;

// Relay
#define RELAY_PIN 14
volatile bool isCharging = false;
volatile bool hasCharged = false;
volatile unsigned long stopChargingTime = 0;
volatile bool resetPending = false;

// Trạng thái từ trường và xe
volatile float CDDT = 0;
volatile float thresholdCDDT = 50.0;
volatile bool carDetected = false;
volatile unsigned long lastCarDetectedChange = 0;
const unsigned long CAR_DEBOUNCE_TIME = 1000; // 1 giây debounce

// Trạng thái thẻ
volatile bool isCardValid = false;
volatile unsigned long lastCardTime = 0;
String lastCardUID = "";
String previousCardUID = "";
String currentChargingUID = ""; // UID của thẻ đang sạc
String warningMessage = ""; // Thông báo cảnh báo

// Biến lưu giá trị trước đó để so sánh
float lastVoltage = -1.0;
float lastCurrent = -1.0;
String lastChargingStatus = "";

// Ngưỡng thay đổi
const float VOLTAGE_THRESHOLD = 0.1; // 0.1V
const float CURRENT_THRESHOLD = 1.0; // 1mA

// Biến dùng để đồng bộ giữa các task
SemaphoreHandle_t xMutex;

// Hiển thị trên OLED
void updateOLED(String uid, String cardStatus, String chargingStatus, String carStatus, float voltage, float current, String warning) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  display.println("UID: " + (uid == "" ? "None" : uid));
  display.println("Card: " + cardStatus);
  display.println("Charging: " + chargingStatus);
  display.println("Car: " + carStatus);
  display.println("Voltage: " + String(voltage, 2) + " V");
  display.println("Current: " + String(current, 2) + " mA");
  
  // Hiển thị cảnh báo ở 2 dòng cuối nếu có
  if (warning != "") {
    display.println("WARNING:");
    display.println(warning);
  }
  
  display.display();
}

void setup_wifi() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
  }
  Serial.println("\nWiFi kết nối thành công!");
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("🔄 Kết nối MQTT...");
    if (client.connect("ESP32_Station")) {
      Serial.println("✅ Kết nối MQTT thành công!");
      if (client.subscribe("rfid/status")) {
        Serial.println("Đã subscribe topic rfid/status thành công");
      } else {
        Serial.println("Lỗi khi subscribe topic rfid/status");
      }
      if (client.subscribe("room/magnetic")) {
        Serial.println("Đã subscribe topic room/magnetic thành công");
      } else {
        Serial.println("Lỗi khi subscribe topic room/magnetic");
      }
      if (client.subscribe("room/threshold")) {
        Serial.println("Đã subscribe topic room/threshold thành công");
      } else {
        Serial.println("Lỗi khi subscribe topic room/threshold");
      }
      if (client.subscribe("charging/status")) {
        Serial.println("Đã subscribe topic charging/status thành công");
      } else {
        Serial.println("Lỗi khi subscribe topic charging/status");
      }
      if (client.subscribe("charging/voltage")) {
        Serial.println("Đã subscribe topic charging/voltage thành công");
      } else {
        Serial.println("Lỗi khi subscribe topic charging/voltage");
      }
      if (client.subscribe("charging/current")) {
        Serial.println("Đã subscribe topic charging/current thành công");
      } else {
        Serial.println("Lỗi khi gửi current qua topic charging/current");
      }
      if (client.publish("request/threshold", "get")) {
        Serial.println("Đã gửi yêu cầu ngưỡng CDDT thành công");
      } else {
        Serial.println("Lỗi khi gửi yêu cầu ngưỡng CDDT");
      }
    } else {
      Serial.print("❌ Lỗi MQTT: ");
      Serial.println(client.state());
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

void checkChargingConditions() {
  String newChargingStatus = "";
  if (isCardValid && carDetected && !isCharging) {
    digitalWrite(RELAY_PIN, HIGH);
    isCharging = true;
    hasCharged = true;
    resetPending = false;
    newChargingStatus = "charging";
    currentChargingUID = lastCardUID; // Lưu UID thẻ đang sạc
  } else if (!isCardValid || !carDetected) {
    digitalWrite(RELAY_PIN, LOW);
    if (isCharging) {
      isCharging = false;
      stopChargingTime = millis();
      resetPending = true;
      newChargingStatus = "stopped";
      currentChargingUID = ""; // Xóa UID thẻ đang sạc
    }
  }
  if (newChargingStatus != "" && newChargingStatus != lastChargingStatus) {
    xSemaphoreTake(xMutex, portMAX_DELAY);
    if (client.publish("charging/status", newChargingStatus.c_str())) {
      Serial.println("Bật/Tắt sạc: " + newChargingStatus);
    } else {
      Serial.println("Lỗi khi gửi trạng thái sạc qua topic charging/status");
    }
    lastChargingStatus = newChargingStatus;
    xSemaphoreGive(xMutex);
  }
  // Cập nhật OLED
  String cardStatus = isCardValid ? "Valid" : "Invalid";
  String carStatus = carDetected ? "Detected" : "Not Detected";
  updateOLED(lastCardUID, cardStatus, lastChargingStatus, carStatus, lastVoltage, lastCurrent, warningMessage);
}

void resetChargingState() {
  isCardValid = false;
  xSemaphoreTake(xMutex, portMAX_DELAY);
  if (lastCardUID != "") {
    lastCardUID = "";
    previousCardUID = "";
    if (client.publish("rfid/scan", "")) {
      Serial.println("Đã đặt lại UID trên topic rfid/scan");
    } else {
      Serial.println("Lỗi khi đặt lại UID trên topic rfid/scan");
    }
  }
  if (client.publish("charging/voltage", "0")) {
    Serial.println("Đã đặt lại voltage");
  } else {
    Serial.println("Lỗi khi đặt lại voltage");
  }
  if (client.publish("charging/current", "0")) {
    Serial.println("Đã đặt lại current");
  } else {
    Serial.println("Lỗi khi đặt lại current");
  }
  lastVoltage = 0.0;
  lastCurrent = 0.0;
  lastChargingStatus = "stopped";
  currentChargingUID = "";
  warningMessage = ""; // Xóa cảnh báo
  Serial.println("Trạng thái sạc và thẻ đã được đặt lại!");
  xSemaphoreGive(xMutex);
  resetPending = false;
  // Cập nhật OLED
  updateOLED(lastCardUID, "Invalid", lastChargingStatus, carDetected ? "Detected" : "Not Detected", lastVoltage, lastCurrent, warningMessage);
}

// Task 1: Quét thẻ RFID
void RFIDTask(void *pvParameters) {
  while (1) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      String cardUID = "";
      for (byte i = 0; i < rfid.uid.size; i++) {
        cardUID += String(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
        cardUID += String(rfid.uid.uidByte[i], HEX);
      }
      cardUID.toLowerCase();
      rfid.PICC_HaltA();
      
      xSemaphoreTake(xMutex, portMAX_DELAY);
      // Kiểm tra nếu đang sạc và thẻ quẹt khác với thẻ đang sạc
      if (isCharging && cardUID != currentChargingUID) {
        warningMessage = "Khong duoc phep";
        Serial.println("Cảnh báo: Không được phép quẹt thẻ khác khi đang sạc!");
        if (client.publish("charging/warning", "Không được phép quẹt thẻ khác khi đang sạc!")) {
          Serial.println("Đã gửi cảnh báo qua topic charging/warning");
        } else {
          Serial.println("Lỗi khi gửi cảnh báo qua topic charging/warning");
        }
        updateOLED(lastCardUID, isCardValid ? "Valid" : "Invalid", lastChargingStatus, carDetected ? "Detected" : "Not Detected", lastVoltage, lastCurrent, warningMessage);
        xSemaphoreGive(xMutex);
      } else if (cardUID != previousCardUID) {
        warningMessage = ""; // Xóa cảnh báo nếu quẹt thẻ mới hợp lệ
        lastCardUID = cardUID;
        lastCardTime = millis();
        bool publishSuccess = false;
        for (int attempt = 0; attempt < 3; attempt++) {
          if (client.publish("rfid/scan", cardUID.c_str())) {
            Serial.println("UID Đang Quét: " + cardUID + " - Gửi thành công");
            publishSuccess = true;
            break;
          } else {
            Serial.println("Lỗi khi gửi UID qua topic rfid/scan, thử lại lần " + String(attempt + 1));
            vTaskDelay(100 / portTICK_PERIOD_MS);
          }
        }
        if (!publishSuccess) {
          Serial.println("Không thể gửi UID sau 3 lần thử, kiểm tra kết nối MQTT!");
        }
        previousCardUID = cardUID;
        xSemaphoreGive(xMutex);
      } else {
        xSemaphoreGive(xMutex);
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Task 2: Đo điện áp và dòng điện
void SensorTask(void *pvParameters) {
  while (1) {
    if (isCharging) {
      float voltage = ina219.getBusVoltage_V();
      float current = ina219.getCurrent_mA();

      if (voltage < 0 || current < 0 || voltage > 5) {
        voltage = 0;
        current = 0;
      }

      if (abs(voltage - lastVoltage) > VOLTAGE_THRESHOLD) {
        String voltagePayload = String(voltage);
        xSemaphoreTake(xMutex, portMAX_DELAY);
        if (client.publish("charging/voltage", voltagePayload.c_str())) {
          Serial.println("Voltage: " + voltagePayload + " V");
        } else {
          Serial.println("Lỗi khi gửi voltage qua topic charging/voltage");
        }
        lastVoltage = voltage;
        xSemaphoreGive(xMutex);
      }

      if (abs(current - lastCurrent) > CURRENT_THRESHOLD) {
        String currentPayload = String(current);
        xSemaphoreTake(xMutex, portMAX_DELAY);
        if (client.publish("charging/current", currentPayload.c_str())) {
          Serial.println("Current: " + currentPayload + " mA");
        } else {
          Serial.println("Lỗi khi gửi current qua topic charging/current");
        }
        lastCurrent = current;
        xSemaphoreGive(xMutex);
      }

      if (current < 0.1) {
        digitalWrite(RELAY_PIN, LOW);
        isCharging = false;
        isCardValid = false;
        stopChargingTime = millis();
        resetPending = true;
        String newChargingStatus = "stopped";
        xSemaphoreTake(xMutex, portMAX_DELAY);
        if (newChargingStatus != lastChargingStatus) {
          if (client.publish("charging/status", newChargingStatus.c_str())) {
            Serial.println("Sạc đã tháo, trạng thái dừng!");
          } else {
            Serial.println("Lỗi khi gửi trạng thái stopped");
          }
          lastChargingStatus = newChargingStatus;
        }
        if (client.publish("charging/voltage", "0")) {
          Serial.println("Đã đặt lại voltage");
        } else {
          Serial.println("Lỗi khi đặt lại voltage");
        }
        if (client.publish("charging/current", "0")) {
          Serial.println("Đã đặt lại current");
        } else {
          Serial.println("Lỗi khi đặt lại current");
        }
        lastVoltage = 0.0;
        lastCurrent = 0.0;
        currentChargingUID = "";
        warningMessage = ""; // Xóa cảnh báo
        xSemaphoreGive(xMutex);
      }
    } else {
      if (lastVoltage != 0.0 || lastCurrent != 0.0) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        if (client.publish("charging/voltage", "0")) {
          Serial.println("Đã đặt lại voltage");
        } else {
          Serial.println("Lỗi khi gửi voltage qua topic charging/voltage");
        }
        if (client.publish("charging/current", "0")) {
          Serial.println("Đã đặt lại current");
        } else {
          Serial.println("Lỗi khi gửi current qua topic charging/current");
        }
        lastVoltage = 0.0;
        lastCurrent = 0.0;
        xSemaphoreGive(xMutex);
      }
    }
    // Cập nhật OLED
    String cardStatus = isCardValid ? "Valid" : "Invalid";
    String carStatus = carDetected ? "Detected" : "Not Detected";
    updateOLED(lastCardUID, cardStatus, lastChargingStatus, carStatus, lastVoltage, lastCurrent, warningMessage);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Task 3: Xử lý MQTT và logic chính
void MQTTTask(void *pvParameters) {
  while (1) {
    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop();
    checkChargingConditions();

    if (resetPending && (millis() - stopChargingTime >= 1000)) {
      resetChargingState();
      hasCharged = false;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  xSemaphoreTake(xMutex, portMAX_DELAY);
  Serial.println("Nhận từ topic [" + String(topic) + "]: " + message);

  if (String(topic) == "rfid/status") {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);
    if (error) {
      Serial.println("Lỗi parse JSON: " + String(error.c_str()));
      xSemaphoreGive(xMutex);
      return;
    }
    bool valid = doc["valid"];
    isCardValid = valid;
    Serial.println("Trạng thái thẻ từ Node-RED: " + String(valid ? "hợp lệ" : "Không hợp lệ"));
    // Cập nhật OLED
    String carStatus = carDetected ? "Detected" : "Not Detected";
    updateOLED(lastCardUID, isCardValid ? "Valid" : "Invalid", lastChargingStatus, carStatus, lastVoltage, lastCurrent, warningMessage);
  } else if (String(topic) == "room/magnetic") {
    CDDT = message.toFloat();
    bool newCarDetected = (CDDT < thresholdCDDT);
    if (newCarDetected != carDetected && (millis() - lastCarDetectedChange >= CAR_DEBOUNCE_TIME)) {
      carDetected = newCarDetected;
      lastCarDetectedChange = millis();
      String payload = carDetected ? "Car detected" : "Car left";
      if (client.publish("vehicle/distance", payload.c_str())) {
        Serial.println("Trạng thái xe: " + payload);
      } else {
        Serial.println("Lỗi khi gửi trạng thái xe qua topic vehicle/distance");
      }
      // Cập nhật OLED
      String cardStatus = isCardValid ? "Valid" : "Invalid";
      updateOLED(lastCardUID, cardStatus, lastChargingStatus, carDetected ? "Detected" : "Not Detected", lastVoltage, lastCurrent, warningMessage);
    }
  } else if (String(topic) == "room/threshold") {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);
    if (!error && doc.containsKey("threshold")) {
      thresholdCDDT = doc["threshold"].as<float>();
      Serial.println("Ngưỡng CDDT nhận được: " + String(thresholdCDDT));
    } else {
      thresholdCDDT = message.toFloat();
      Serial.println("Ngưỡng CDDT nhận được (dạng số): " + String(thresholdCDDT));
    }
  } else if (String(topic) == "charging/status") {
    Serial.println("Trạng thái sạc: " + message);
    lastChargingStatus = message;
    // Cập nhật OLED
    String cardStatus = isCardValid ? "Valid" : "Invalid";
    String carStatus = carDetected ? "Detected" : "Not Detected";
    updateOLED(lastCardUID, cardStatus, lastChargingStatus, carStatus, lastVoltage, lastCurrent, warningMessage);
  } else if (String(topic) == "charging/voltage") {
    Serial.println("Voltage: " + message + " V");
    lastVoltage = message.toFloat();
    // Cập nhật OLED
    String cardStatus = isCardValid ? "Valid" : "Invalid";
    String carStatus = carDetected ? "Detected" : "Not Detected";
    updateOLED(lastCardUID, cardStatus, lastChargingStatus, carStatus, lastVoltage, lastCurrent, warningMessage);
  } else if (String(topic) == "charging/current") {
    Serial.println("Current: " + message + " mA");
    lastCurrent = message.toFloat();
    // Cập nhật OLED
    String cardStatus = isCardValid ? "Valid" : "Invalid";
    String carStatus = carDetected ? "Detected" : "Not Detected";
    updateOLED(lastCardUID, cardStatus, lastChargingStatus, carStatus, lastVoltage, lastCurrent, warningMessage);
  }
  xSemaphoreGive(xMutex);
}

void setup() {
  Serial.begin(115200);
  // Khởi tạo OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Lỗi khởi tạo OLED!");
    while (1) vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Starting...");
  display.display();

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  SPI.begin();
  rfid.PCD_Init();
  Serial.println("RFID reader initialized");
  if (!ina219.begin()) {
    Serial.println("Lỗi khởi tạo INA219!");
    while (1) vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  Serial.println("INA219 initialized");
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL) {
    Serial.println("Lỗi tạo semaphore!");
    while (1) vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  xTaskCreatePinnedToCore(RFIDTask, "RFID Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(SensorTask, "Sensor Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(MQTTTask, "MQTT Task", 4096, NULL, 2, NULL, 1);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}