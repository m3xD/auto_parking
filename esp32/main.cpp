#include "secrets.h" // File chứa thông tin bí mật (WiFi, AWS credentials, certificates)
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <time.h>
#include <esp_task_wdt.h>  // Watchdog Timer
#include <Preferences.h>   // Lưu trữ không bay hơi

// ======= ĐỊNH NGHĨA PIN CHO CẢM BIẾN VÀ SERVO ========
// Cảm biến cổng
#define SENSOR_VAO_1_PIN 26
#define SENSOR_VAO_2_PIN 34
#define SENSOR_RA_1_PIN  13
#define SENSOR_RA_2_PIN  23

// Servo điều khiển rào chắn
#define SERVO_ENTRY_PIN 19
#define SERVO_EXIT_PIN  18

// Cảm biến chỗ đỗ (4 cảm biến IR)
#define SLOT_SENSOR_PIN_1 32
#define SLOT_SENSOR_PIN_2 33
#define SLOT_SENSOR_PIN_3 25
#define SLOT_SENSOR_PIN_4 27

// ======= HẰNG SỐ VÀ CẤU HÌNH ========
// Cấu hình Watchdog
#define WDT_TIMEOUT 10  // 10 giây timeout cho watchdog

// Cấu hình đo lường thời gian và debounce
#define GATE_SENSOR_DEBOUNCE_INTERVAL 500   // ms
#define SLOT_DEBOUNCE_DELAY 1000            // ms
#define STATUS_REPORT_INTERVAL 60000        // 1 phút
#define STATE_SAVE_INTERVAL 300000          // 5 phút
#define RECONNECT_INTERVAL 5000             // 5 giây
#define MAX_RECONNECT_ATTEMPTS 5            // Số lần thử kết nối tối đa
#define ACTIVITY_TIMEOUT 1800000            // 30 phút trước khi chuyển sang chế độ tiết kiệm năng lượng

// Cấu hình MQTT
#define MAX_MQTT_QUEUE 10                   // Số lượng thông điệp MQTT tối đa trong hàng đợi
#define MAX_MQTT_PAYLOAD_SIZE 512           // Kích thước tối đa của payload MQTT

// Cấu hình NTP
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC 7 * 3600             // UTC+7 
#define DAYLIGHT_OFFSET_SEC 0

// ======= KHAI BÁO ĐỐI TƯỢNG TOÀN CỤC ========
// Khởi tạo đối tượng Servo
Servo servoEntry, servoExit;

// Khởi tạo đối tượng WiFi, MQTT và lưu trữ
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
Preferences preferences;

// ======= CÁC CẤU TRÚC DỮ LIỆU ========
// Cấu trúc quản lý cảm biến cổng
struct GateSensor {
  const int pin;
  const char* sensorIdentifier;
  const char* gateAreaMqttId;
  const char* eventTypeOnActive;
  bool isActive;
  bool prevIsActive;
  unsigned long lastDebounceTime;
  const int activeLogicLevel;     // Mức logic được coi là active (HIGH hoặc LOW)
  bool autoCloseBarrier;
  Servo* associatedServo;
  int servoClosePosition;
  const char* barrierTypeForState;
};

// Cấu trúc quản lý cảm biến chỗ đỗ
struct ParkingSlotSensor {
  const int pin;
  const char* slotMqttId;
  bool isOccupied;
  bool prevIsOccupied;
  unsigned long lastDebounceTime;
  const int activeLogicLevel;
};

// Cấu trúc cho hàng đợi MQTT
struct MQTTMessage {
  char topic[50];
  char payload[MAX_MQTT_PAYLOAD_SIZE];
  size_t length;
  bool retained;
  int qos;
  bool pending;
};

// ======= KHỞI TẠO CẢM BIẾN ========
// Cảm biến cổng với activeLogicLevel phù hợp
GateSensor gateSensors[] = {
  // {pin, identifier, gateArea, eventType, isActive, prevIsActive, lastDebounce, activeLogic, autoClose, servo, servoPos, barrierType}
  {SENSOR_VAO_1_PIN, "SENSOR_VAO_1", "entry_approach", "presence_detected", false, false, 0, HIGH, false, NULL, 0, ""},
  {SENSOR_VAO_2_PIN, "SENSOR_VAO_2", "entry_passed",   "vehicle_passed",    false, false, 0, HIGH, true, &servoEntry, 0, "entry"},
  {SENSOR_RA_1_PIN,  "SENSOR_RA_1",  "exit_approach",  "presence_detected", false, false, 0, HIGH, false, NULL, 0, ""},
  {SENSOR_RA_2_PIN,  "SENSOR_RA_2",  "exit_passed",    "vehicle_passed",    false, false, 0, LOW,  true, &servoExit, 90, "exit"}
};
const int NUM_GATE_SENSORS = sizeof(gateSensors) / sizeof(GateSensor);

// Cảm biến chỗ đỗ
ParkingSlotSensor parkingSlots[] = {
  // {pin, slotId, isOccupied, prevIsOccupied, lastDebounce, activeLogic}
  {SLOT_SENSOR_PIN_1, "S1", false, false, 0, HIGH},
  {SLOT_SENSOR_PIN_2, "S2", false, false, 0, HIGH},
  {SLOT_SENSOR_PIN_3, "S3", false, false, 0, HIGH},
  {SLOT_SENSOR_PIN_4, "S4", false, false, 0, HIGH}
};
const int NUM_PARKING_SLOTS = sizeof(parkingSlots) / sizeof(ParkingSlotSensor);

// ======= BIẾN TOÀN CỤC ========
// Biến cho hàng đợi MQTT
MQTTMessage mqttQueue[MAX_MQTT_QUEUE];
int mqttQueueIndex = 0;

// Biến cho quản lý kết nối
unsigned long lastReconnectAttempt = 0;
int reconnectCount = 0;
String deviceMacAddress = "";

// Biến cho quản lý năng lượng
unsigned long lastActivityTime = 0;
bool lowPowerMode = false;

// Biến cho báo cáo trạng thái
unsigned long lastStatusReport = 0;
unsigned long lastStateSave = 0;
String firmwareVersion = "1.0.0";  // Phiên bản firmware

// Biến toàn cục theo dõi trạng thái
int totalOccupiedSlots = 0;
bool entryGateOpen = false;
bool exitGateOpen = false;
unsigned long systemStartTime = 0;

// ======= KHAI BÁO HÀM ========
// Các hàm kết nối và giao tiếp
void connectWiFi();
void syncNTPTime();
boolean reconnectAWSIoT();
void connectAWSIoT();
String getTimeStamp();

// Các hàm xử lý MQTT
void messageHandler(char* topic, byte* payload, unsigned int length);
bool queueMqttMessage(const char* topic, const char* payload, size_t length, int qos = 0, bool retained = false);
bool publishMqttMessage(const char* topic, const JsonDocument& doc, int qos = 0, bool retained = false);
void processQueuedMessages();

// Các hàm xuất bản trạng thái
void publishBarrierState(const char* barrierType, const char* state);
void publishGateSensorEvent(const char* sensorPinIdentifier, const char* gateArea, const char* eventType);
void publishParkingSlotStatus(const char* slotMqttId, bool isOccupied);
void publishInitialState();
void reportSystemStatus();

// Các hàm xử lý cảm biến
void handleGateSensors();
void handleParkingSlotSensors();

// Các hàm lưu trữ trạng thái
void saveState();
void restoreState();

// ======= TRIỂN KHAI CÁC HÀM ========

// ---- HÀM KẾT NỐI VÀ GIAO TIẾP ----

// Kết nối WiFi
void connectWiFi() {
  Serial.print("Đang kết nối WiFi đến ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  unsigned long startAttemptTime = millis();
  
  while (WiFi.status() != WL_CONNECTED && 
         millis() - startAttemptTime < 10000) {
    Serial.print(".");
    delay(100);
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nKhông thể kết nối WiFi. Đang khởi động lại...");
    ESP.restart();
  }
  
  Serial.println("\nĐã kết nối WiFi");
  Serial.print("Địa chỉ IP: ");
  Serial.println(WiFi.localIP());
  
  deviceMacAddress = WiFi.macAddress();
  Serial.print("Địa chỉ MAC: ");
  Serial.println(deviceMacAddress);
  
  // Đồng bộ thời gian sau khi kết nối WiFi
  syncNTPTime();
}

// Đồng bộ thời gian từ server NTP
void syncNTPTime() {
  Serial.println("Đang đồng bộ thời gian với NTP server...");
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  
  struct tm timeinfo;
  int retry = 0;
  while (!getLocalTime(&timeinfo) && retry < 5) {
    Serial.println("Đang chờ đồng bộ thời gian...");
    delay(1000);
    retry++;
  }
  
  if (retry >= 5) {
    Serial.println("Không thể đồng bộ thời gian. Tiếp tục...");
    return;
  }
  
  Serial.print("Thời gian hiện tại: ");
  Serial.println(getTimeStamp());
}

// Lấy timestamp hiện tại từ NTP
String getTimeStamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "Unknown";
  }
  char timeStringBuff[50];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(timeStringBuff);
}

// Thử kết nối lại AWS IoT
boolean reconnectAWSIoT() {
  Serial.println("Đang thử kết nối lại đến AWS IoT Core...");
  
  String clientId = "ESP32_" + deviceMacAddress;
  
  if (client.connect(clientId.c_str())) {
    Serial.println("Đã kết nối lại thành công đến AWS IoT!");
    reconnectCount = 0;
    
    // Đăng ký lại các topic cần thiết
    client.subscribe("$aws/things/" THINGNAME "/shadow/update/accepted");
    client.subscribe("$aws/things/" THINGNAME "/shadow/get/accepted");
    client.subscribe("smart_parking/command/barriers/#");
    
    // Báo cáo trạng thái kết nối
    DynamicJsonDocument statusDoc(256);
    statusDoc["state"]["reported"]["connection"] = "online";
    statusDoc["state"]["reported"]["last_reconnect"] = getTimeStamp();
    statusDoc["state"]["reported"]["ip"] = WiFi.localIP().toString();
    statusDoc["state"]["reported"]["rssi"] = WiFi.RSSI();
    
    char buffer[256];
    size_t n = serializeJson(statusDoc, buffer);
    client.publish("$aws/things/" THINGNAME "/shadow/update", buffer, n);
    
    return true;
  }
  
  return false;
}

// Kết nối đến AWS IoT
void connectAWSIoT() {
  // Cấu hình các thông số cho kết nối an toàn
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
  
  // Cấu hình client MQTT
  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setCallback(messageHandler);
  client.setKeepAlive(60); // Cấu hình keep-alive 60 giây
  
  Serial.println("Đang kết nối đến AWS IoT Core...");
  
  // Cấu hình timeout cho các thao tác mạng
  net.setTimeout(2000); // 2 giây timeout
  
  String clientId = "ESP32_" + deviceMacAddress;
  
  reconnectCount = 0;
  while (!client.connected() && reconnectCount < MAX_RECONNECT_ATTEMPTS) {
    Serial.print("Lần thử #");
    Serial.print(reconnectCount + 1);
    Serial.print("... ");
    
    if (client.connect(clientId.c_str())) {
      Serial.println("Đã kết nối!");
      
      // Đăng ký các topic
      client.subscribe("$aws/things/" THINGNAME "/shadow/update/accepted");
      client.subscribe("$aws/things/" THINGNAME "/shadow/get/accepted");
      client.subscribe("smart_parking/command/barriers/#");
      
      // Lấy trạng thái shadow hiện tại
      client.publish("$aws/things/" THINGNAME "/shadow/get", "", 0);
      
      // Xuất bản trạng thái ban đầu
      publishInitialState();
      
      return;
    }
    
    Serial.println("Thất bại.");
    reconnectCount++;
    delay(RECONNECT_INTERVAL);
  }
  
  if (reconnectCount >= MAX_RECONNECT_ATTEMPTS) {
    Serial.println("Đã vượt quá số lần thử kết nối. Khởi động lại ESP32...");
    ESP.restart();
  }
}

// ---- HÀM XỬ LÝ MQTT ----

// Xử lý thông điệp MQTT nhận được
void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("Nhận thông điệp từ topic: ");
  Serial.println(topic);
  
  // Chuyển đổi payload thành chuỗi kết thúc null
  char payloadStr[length + 1];
  memcpy(payloadStr, payload, length);
  payloadStr[length] = '\0';
  
  Serial.print("Nội dung: ");
  Serial.println(payloadStr);
  
  // Kiểm tra topic cho lệnh điều khiển rào chắn
  if (strstr(topic, "smart_parking/command/barriers/") != NULL) {
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, payloadStr);
    
    if (error) {
      Serial.print("deserializeJson() thất bại: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Xử lý lệnh điều khiển rào chắn
    if (strstr(topic, "entry") != NULL) {
      const char* action = doc["action"];
      if (strcmp(action, "open") == 0) {
        Serial.println("Lệnh mở rào vào");
        servoEntry.write(90);
        entryGateOpen = true;
        publishBarrierState("entry", "opened");
      } else if (strcmp(action, "close") == 0) {
        Serial.println("Lệnh đóng rào vào");
        servoEntry.write(0);
        entryGateOpen = false;
        publishBarrierState("entry", "closed");
      }
    } else if (strstr(topic, "exit") != NULL) {
      const char* action = doc["action"];
      if (strcmp(action, "open") == 0) {
        Serial.println("Lệnh mở rào ra");
        servoExit.write(0);
        exitGateOpen = true;
        publishBarrierState("exit", "opened");
      } else if (strcmp(action, "close") == 0) {
        Serial.println("Lệnh đóng rào ra");
        servoExit.write(90);
        exitGateOpen = false;
        publishBarrierState("exit", "closed");
      }
    }
    
    // Cập nhật thời gian hoạt động cuối cùng
    lastActivityTime = millis();
  }
  
  // Xử lý các topic khác nếu cần
}

// Thêm thông điệp vào hàng đợi MQTT
bool queueMqttMessage(const char* topic, const char* payload, size_t length, int qos, bool retained) {
  if (mqttQueueIndex >= MAX_MQTT_QUEUE) {
    Serial.println("Hàng đợi MQTT đầy, bỏ qua thông điệp");
    return false;
  }
  
  if (length > 0 && length < MAX_MQTT_PAYLOAD_SIZE) {
    strncpy(mqttQueue[mqttQueueIndex].topic, topic, 49);
    strncpy(mqttQueue[mqttQueueIndex].payload, payload, MAX_MQTT_PAYLOAD_SIZE - 1);
    mqttQueue[mqttQueueIndex].length = length;
    mqttQueue[mqttQueueIndex].qos = qos;
    mqttQueue[mqttQueueIndex].retained = retained;
    mqttQueue[mqttQueueIndex].pending = true;
    mqttQueueIndex++;
    return true;
  }
  
  Serial.println("Kích thước payload không hợp lệ");
  return false;
}

// Xuất bản thông điệp MQTT với hàng đợi
bool publishMqttMessage(const char* topic, const JsonDocument& doc, int qos, bool retained) {
  char buffer[MAX_MQTT_PAYLOAD_SIZE];
  size_t n = serializeJson(doc, buffer);
  
  if (n > 0 && n < MAX_MQTT_PAYLOAD_SIZE) {
    if (client.connected()) {
      return client.publish(topic, buffer, n, retained);
    } else {
      return queueMqttMessage(topic, buffer, n, qos, retained);
    }
  }
  
  Serial.println("Lỗi serialize JSON");
  return false;
}

// Xử lý các thông điệp trong hàng đợi MQTT
void processQueuedMessages() {
  if (!client.connected() || mqttQueueIndex == 0) {
    return;
  }
  
  int messagesProcessed = 0;
  for (int i = 0; i < mqttQueueIndex; i++) {
    if (mqttQueue[i].pending) {
      bool success = client.publish(
        mqttQueue[i].topic, 
        mqttQueue[i].payload, 
        mqttQueue[i].length, 
        mqttQueue[i].retained
      );
      
      if (success) {
        mqttQueue[i].pending = false;
        messagesProcessed++;
      } else {
        Serial.print("Không thể gửi thông điệp MQTT đến topic: ");
        Serial.println(mqttQueue[i].topic);
        break;
      }
    }
  }
  
  // Nếu đã xử lý tất cả thông điệp, làm trống hàng đợi
  if (messagesProcessed == mqttQueueIndex) {
    mqttQueueIndex = 0;
  }
}

// ---- HÀM XUẤT BẢN TRẠNG THÁI ----

// Xuất bản trạng thái rào chắn
void publishBarrierState(const char* barrierType, const char* state) {
  DynamicJsonDocument doc(128);
  doc["barrier_type"] = barrierType;
  doc["state"] = state;
  doc["timestamp"] = getTimeStamp();
  
  char topic[50];
  snprintf(topic, sizeof(topic), "smart_parking/barriers/%s/state", barrierType);
  
  publishMqttMessage(topic, doc);
}

// Xuất bản sự kiện cảm biến cổng
void publishGateSensorEvent(const char* sensorPinIdentifier, const char* gateArea, const char* eventType) {
  DynamicJsonDocument doc(256);
  doc["sensor"] = sensorPinIdentifier;
  doc["gate_area"] = gateArea;
  doc["event"] = eventType;
  doc["timestamp"] = getTimeStamp();
  
  char topic[50];
  snprintf(topic, sizeof(topic), "smart_parking/gates/%s/events", gateArea);
  
  publishMqttMessage(topic, doc);
}

// Xuất bản trạng thái chỗ đỗ
void publishParkingSlotStatus(const char* slotMqttId, bool isOccupied) {
  DynamicJsonDocument doc(128);
  doc["slot_id"] = slotMqttId;
  doc["occupied"] = isOccupied;
  doc["timestamp"] = getTimeStamp();
  
  char topic[50];
  snprintf(topic, sizeof(topic), "smart_parking/slots/%s/status", slotMqttId);
  
  publishMqttMessage(topic, doc);
  
  // Cập nhật tổng số chỗ đỗ đã sử dụng
  totalOccupiedSlots = 0;
  for (int i = 0; i < NUM_PARKING_SLOTS; i++) {
    if (parkingSlots[i].isOccupied) {
      totalOccupiedSlots++;
    }
  }
  
  // Xuất bản tổng trạng thái chỗ đỗ
  DynamicJsonDocument summaryDoc(256);
  summaryDoc["total_slots"] = NUM_PARKING_SLOTS;
  summaryDoc["occupied_slots"] = totalOccupiedSlots;
  summaryDoc["available_slots"] = NUM_PARKING_SLOTS - totalOccupiedSlots;
  summaryDoc["occupancy_percentage"] = (totalOccupiedSlots * 100) / NUM_PARKING_SLOTS;
  summaryDoc["timestamp"] = getTimeStamp();
  
  publishMqttMessage("smart_parking/slots/summary", summaryDoc);
}

// Xuất bản trạng thái ban đầu
void publishInitialState() {
  // Xuất bản trạng thái rào chắn
  publishBarrierState("entry", entryGateOpen ? "opened" : "closed");
  publishBarrierState("exit", exitGateOpen ? "opened" : "closed");
  
  // Xuất bản trạng thái chỗ đỗ
  for (int i = 0; i < NUM_PARKING_SLOTS; i++) {
    publishParkingSlotStatus(parkingSlots[i].slotMqttId, parkingSlots[i].isOccupied);
  }
  
  // Xuất bản thông tin thiết bị
  DynamicJsonDocument deviceDoc(256);
  deviceDoc["device"]["id"] = THINGNAME;
  deviceDoc["device"]["mac"] = deviceMacAddress;
  deviceDoc["device"]["ip"] = WiFi.localIP().toString();
  deviceDoc["device"]["rssi"] = WiFi.RSSI();
  deviceDoc["device"]["uptime"] = millis() / 1000;
  deviceDoc["device"]["firmware"] = firmwareVersion;
  deviceDoc["device"]["startup_time"] = getTimeStamp();
  
  publishMqttMessage("smart_parking/device/info", deviceDoc);
}

// Báo cáo trạng thái hệ thống
void reportSystemStatus() {
  if (client.connected()) {
    DynamicJsonDocument statusDoc(512);
    
    // Thông tin chung
    statusDoc["device"]["id"] = THINGNAME;
    statusDoc["device"]["mac"] = deviceMacAddress;
    statusDoc["device"]["uptime"] = millis() / 1000;
    statusDoc["device"]["free_heap"] = ESP.getFreeHeap();
    statusDoc["device"]["wifi"]["rssi"] = WiFi.RSSI();
    statusDoc["device"]["wifi"]["ip"] = WiFi.localIP().toString();
    statusDoc["device"]["power_mode"] = lowPowerMode ? "low" : "normal";
    statusDoc["device"]["firmware"] = firmwareVersion;
    
    // Thông tin chỗ đỗ
    JsonArray slots = statusDoc.createNestedArray("parking_slots");
    totalOccupiedSlots = 0;
    
    for (int i = 0; i < NUM_PARKING_SLOTS; i++) {
      JsonObject slot = slots.createNestedObject();
      slot["id"] = parkingSlots[i].slotMqttId;
      slot["occupied"] = parkingSlots[i].isOccupied;
      if (parkingSlots[i].isOccupied) totalOccupiedSlots++;
    }
    
    statusDoc["parking_summary"]["total"] = NUM_PARKING_SLOTS;
    statusDoc["parking_summary"]["occupied"] = totalOccupiedSlots;
    statusDoc["parking_summary"]["available"] = NUM_PARKING_SLOTS - totalOccupiedSlots;
    statusDoc["parking_summary"]["occupancy_percentage"] = (totalOccupiedSlots * 100) / NUM_PARKING_SLOTS;
    
    // Thông tin rào chắn
    statusDoc["barriers"]["entry"] = entryGateOpen ? "opened" : "closed";
    statusDoc["barriers"]["exit"] = exitGateOpen ? "opened" : "closed";
    
    // Thông tin thời gian
    statusDoc["timestamp"] = getTimeStamp();
    
    publishMqttMessage("smart_parking/system/status", statusDoc);
  }
}

// ---- HÀM XỬ LÝ CẢM BIẾN ----

// Xử lý cảm biến cổng
void handleGateSensors() {
  unsigned long currentTime = millis();
  bool activityDetected = false;
  
  for (int i = 0; i < NUM_GATE_SENSORS; i++) {
    bool currentPinState = digitalRead(gateSensors[i].pin);
    bool currentlyDetectedActive = (currentPinState == gateSensors[i].activeLogicLevel);

    // Phát hiện thay đổi trạng thái với debounce
    if (currentlyDetectedActive != gateSensors[i].isActive && 
        (currentTime - gateSensors[i].lastDebounceTime > GATE_SENSOR_DEBOUNCE_INTERVAL)) {
      
      gateSensors[i].isActive = currentlyDetectedActive;
      gateSensors[i].lastDebounceTime = currentTime;
      
      // Nếu phát hiện hoạt động, cập nhật thời gian hoạt động cuối
      if (gateSensors[i].isActive) {
        activityDetected = true;
        lastActivityTime = currentTime;
        
        // Xuất bản sự kiện cảm biến
        publishGateSensorEvent(gateSensors[i].sensorIdentifier, 
                              gateSensors[i].gateAreaMqttId, 
                              gateSensors[i].eventTypeOnActive);
        
        // Xử lý rào chắn tự động nếu cần
        if (gateSensors[i].autoCloseBarrier && gateSensors[i].associatedServo != NULL) {
          Serial.print("Cảm biến "); Serial.print(gateSensors[i].sensorIdentifier);
          Serial.println(" kích hoạt: Tự động đóng rào liên quan.");
          
          gateSensors[i].associatedServo->write(gateSensors[i].servoClosePosition);
          
          // Cập nhật trạng thái rào chắn
          if (strcmp(gateSensors[i].barrierTypeForState, "entry") == 0) {
            entryGateOpen = false;
          } else if (strcmp(gateSensors[i].barrierTypeForState, "exit") == 0) {
            exitGateOpen = false;
          }
          
          publishBarrierState(gateSensors[i].barrierTypeForState, "closed_auto");
        }
      }
      
      gateSensors[i].prevIsActive = gateSensors[i].isActive;
    }
  }
  
  // Nếu phát hiện hoạt động và đang ở chế độ tiết kiệm năng lượng, thoát chế độ đó
  if (activityDetected && lowPowerMode) {
    Serial.println("Hoạt động phát hiện: Trở lại chế độ năng lượng bình thường");
    lowPowerMode = false;
  }
}

// Xử lý cảm biến chỗ đỗ
void handleParkingSlotSensors() {
 unsigned long currentTime = millis();
 bool slotStateChanged = false;
 
 for (int i = 0; i < NUM_PARKING_SLOTS; i++) {
   bool currentPinState = digitalRead(parkingSlots[i].pin);
   bool currentlyDetectedOccupied = (currentPinState == parkingSlots[i].activeLogicLevel);

   // Phát hiện thay đổi trạng thái với debounce
   if (currentlyDetectedOccupied != parkingSlots[i].isOccupied && 
       (currentTime - parkingSlots[i].lastDebounceTime > SLOT_DEBOUNCE_DELAY)) {
     
     parkingSlots[i].isOccupied = currentlyDetectedOccupied;
     parkingSlots[i].lastDebounceTime = currentTime;
     
     // Xuất bản trạng thái mới
     publishParkingSlotStatus(parkingSlots[i].slotMqttId, parkingSlots[i].isOccupied);
     parkingSlots[i].prevIsOccupied = parkingSlots[i].isOccupied;
     
     slotStateChanged = true;
     lastActivityTime = currentTime; // Cập nhật thời gian hoạt động cuối
   }
 }
 
 // Nếu có thay đổi trạng thái, lưu trạng thái mới
 if (slotStateChanged) {
   saveState();
 }
}

// ---- HÀM LƯU TRỮ TRẠNG THÁI ----

// Lưu trạng thái vào bộ nhớ không bay hơi
void saveState() {
 // Lưu trạng thái của các chỗ đỗ
 for (int i = 0; i < NUM_PARKING_SLOTS; i++) {
   String key = "slot" + String(i);
   preferences.putBool(key.c_str(), parkingSlots[i].isOccupied);
 }
 
 // Lưu trạng thái rào chắn
 preferences.putBool("entry_open", entryGateOpen);
 preferences.putBool("exit_open", exitGateOpen);
 
 // Lưu thời gian
 preferences.putULong("lastSave", millis());
 
 Serial.println("Đã lưu trạng thái hệ thống");
}

// Khôi phục trạng thái từ bộ nhớ không bay hơi
void restoreState() {
 // Kiểm tra xem có trạng thái đã lưu không
 unsigned long lastSave = preferences.getULong("lastSave", 0);
 if (lastSave > 0) {
   Serial.println("Đang khôi phục trạng thái trước đó...");
   
   // Khôi phục trạng thái chỗ đỗ
   totalOccupiedSlots = 0;
   for (int i = 0; i < NUM_PARKING_SLOTS; i++) {
     String key = "slot" + String(i);
     parkingSlots[i].isOccupied = preferences.getBool(key.c_str(), false);
     parkingSlots[i].prevIsOccupied = parkingSlots[i].isOccupied;
     
     if (parkingSlots[i].isOccupied) {
       totalOccupiedSlots++;
     }
   }
   
   // Khôi phục trạng thái rào chắn
   entryGateOpen = preferences.getBool("entry_open", false);
   exitGateOpen = preferences.getBool("exit_open", false);
   
   Serial.println("Đã khôi phục trạng thái hệ thống");
 } else {
   Serial.println("Không tìm thấy trạng thái đã lưu trước đó");
 }
}

// ======= CÁC HÀM CHÍNH ========

// Hàm setup
void setup() {
 // Khởi tạo giao tiếp Serial
 Serial.begin(115200);
 while (!Serial && millis() < 5000); // Đợi tối đa 5 giây cho serial, rồi tiếp tục
 delay(1000);

 // Lưu thời gian khởi động
 systemStartTime = millis();

 Serial.println("\n\n--- ĐÃ KHỞI ĐỘNG HỆ THỐNG QUẢN LÝ BÃI ĐỖ XE ---");
 Serial.print("Phiên bản firmware: ");
 Serial.println(firmwareVersion);
 Serial.println("Ngày: " + getTimeStamp());

 // Khởi tạo watchdog
 esp_task_wdt_init(WDT_TIMEOUT, true);
 esp_task_wdt_add(NULL);
 Serial.println("Đã khởi tạo Watchdog Timer");
 
 // Khởi tạo Preferences cho lưu trữ
 preferences.begin("parking", false);
 Serial.println("Đã khởi tạo bộ nhớ không bay hơi");
 
 // Khôi phục trạng thái trước đó nếu có
 restoreState();

 Serial.println("Đang khởi tạo cảm biến và rào chắn...");
 
 // Khởi tạo pin cho cảm biến cổng
 for (int i = 0; i < NUM_GATE_SENSORS; i++) {
   pinMode(gateSensors[i].pin, INPUT);
   bool initialPinState = digitalRead(gateSensors[i].pin);
   gateSensors[i].isActive = (initialPinState == gateSensors[i].activeLogicLevel);
   gateSensors[i].prevIsActive = gateSensors[i].isActive;
   
   Serial.print("Cảm biến ");
   Serial.print(gateSensors[i].sensorIdentifier);
   Serial.print(": trạng thái ban đầu = ");
   Serial.println(gateSensors[i].isActive ? "ACTIVE" : "INACTIVE");
 }

 // Khởi tạo pin cho cảm biến chỗ đỗ
 for (int i = 0; i < NUM_PARKING_SLOTS; i++) {
   pinMode(parkingSlots[i].pin, INPUT);
   bool initialPinState = digitalRead(parkingSlots[i].pin);
   parkingSlots[i].isOccupied = (initialPinState == parkingSlots[i].activeLogicLevel);
   parkingSlots[i].prevIsOccupied = parkingSlots[i].isOccupied;
   
   Serial.print("Chỗ đỗ ");
   Serial.print(parkingSlots[i].slotMqttId);
   Serial.print(": trạng thái ban đầu = ");
   Serial.println(parkingSlots[i].isOccupied ? "OCCUPIED" : "AVAILABLE");
 }

 // Gắn và đặt vị trí ban đầu cho servo
 servoEntry.attach(SERVO_ENTRY_PIN);
 servoExit.attach(SERVO_EXIT_PIN);
 
 // Đặt vị trí servo dựa trên trạng thái đã khôi phục
 if (entryGateOpen) {
   servoEntry.write(90);
 } else {
   servoEntry.write(0);
 }
 
 if (exitGateOpen) {
   servoExit.write(0);
 } else {
   servoExit.write(90);
 }
 
 Serial.print("Rào vào: ");
 Serial.println(entryGateOpen ? "MỞ" : "ĐÓNG");
 Serial.print("Rào ra: ");
 Serial.println(exitGateOpen ? "MỞ" : "ĐÓNG");

 // Khởi tạo biến hoạt động
 lastActivityTime = millis();
 
 // Kết nối WiFi và AWS IoT
 connectWiFi();
 connectAWSIoT();
 
 Serial.println("--- Khởi động hoàn tất ---");
}

// Hàm loop
void loop() {
 unsigned long currentTime = millis();
 
 // Đặt lại watchdog
 esp_task_wdt_reset();
 
 // Kiểm tra kết nối WiFi và MQTT
 if (WiFi.status() != WL_CONNECTED) {
   Serial.println("WiFi bị ngắt kết nối. Đang kết nối lại...");
   connectWiFi();
 }
 
 if (!client.connected()) {
   // Thử kết nối lại nếu đã đến lúc
   if (currentTime - lastReconnectAttempt > RECONNECT_INTERVAL) {
     lastReconnectAttempt = currentTime;
     if (reconnectAWSIoT()) {
       Serial.println("Đã kết nối lại thành công đến AWS IoT!");
       lastReconnectAttempt = 0;
     } else {
       reconnectCount++;
       if (reconnectCount > MAX_RECONNECT_ATTEMPTS) {
         Serial.println("Quá nhiều lần thử kết nối thất bại. Đang khởi động lại...");
         ESP.restart();
       }
     }
   }
 } else {
   client.loop();
   processQueuedMessages(); // Xử lý các thông điệp trong hàng đợi
   reconnectCount = 0; // Đặt lại bộ đếm khi kết nối thành công
 }
 
 // Quản lý chế độ năng lượng dựa trên hoạt động
 if (!lowPowerMode && (currentTime - lastActivityTime > ACTIVITY_TIMEOUT)) {
   Serial.println("Không có hoạt động trong thời gian dài: Chuyển sang chế độ tiết kiệm năng lượng");
   lowPowerMode = true;
 }
 
 // Xử lý cảm biến 
 handleGateSensors();
 handleParkingSlotSensors();
 
 // Nếu ở chế độ tiết kiệm năng lượng, giảm tần số quét cảm biến
 if (lowPowerMode) {
   delay(500); // Thêm độ trễ để giảm tần số quét
 }
 
 // Báo cáo trạng thái định kỳ
 if (currentTime - lastStatusReport > STATUS_REPORT_INTERVAL) {
   lastStatusReport = currentTime;
   reportSystemStatus();
 }
 
 // Đồng bộ thời gian NTP định kỳ (mỗi 24 giờ)
 static unsigned long lastNTPSync = 0;
 if (currentTime - lastNTPSync > 86400000) { // 24 giờ
   lastNTPSync = currentTime;
   syncNTPTime();
 }
 
 // Lưu trạng thái định kỳ
 if (currentTime - lastStateSave > STATE_SAVE_INTERVAL) {
   lastStateSave = currentTime;
   saveState();
 }
}