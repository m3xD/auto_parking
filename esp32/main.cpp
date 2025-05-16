// Bao gồm các thư viện cần thiết
#include "secrets.h" // File chứa thông tin bí mật (WiFi, AWS credentials, certificates)
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h> // Thư viện Servo cho ESP32
#include <time.h>       // Thư viện thời gian cho NTP

// --- ĐỊNH NGHĨA PIN CHO CẢM BIẾN VÀ SERVO ---
// Cảm biến cổng
#define SENSOR_VAO_1_PIN 26
#define SENSOR_VAO_2_PIN 34 
#define SENSOR_RA_1_PIN  14
#define SENSOR_RA_2_PIN  23 

// Servo điều khiển rào chắn
#define SERVO_ENTRY_PIN 19 // Servo cho rào vào
#define SERVO_EXIT_PIN  18 // Servo cho rào ra

// Cảm biến chỗ đỗ 
#define SLOT_SENSOR_PIN_1 16 
#define SLOT_SENSOR_PIN_2 4
// #define SLOT_SENSOR_PIN_3 25
// #define SLOT_SENSOR_PIN_4 27 

// Khai báo đối tượng Servo
Servo servoEntry, servoExit;

// --- KHỞI TẠO BIẾN TOÀN CỤC CHO WIFI VÀ MQTT ---
WiFiClientSecure net = WiFiClientSecure(); // Đối tượng client an toàn cho WiFi
PubSubClient client(net);                  // Đối tượng client MQTT

// Cấu hình NTP (Network Time Protocol) để đồng bộ thời gian
const char* NTP_SERVER = "pool.ntp.org";
const long GMT_OFFSET_SEC = 7 * 3600; // Việt Nam GMT+7
const int DAYLIGHT_OFFSET_SEC = 0;    // Không sử dụng DST

// --- CẤU TRÚC VÀ BIẾN QUẢN LÝ CẢM BIẾN ---

// Cấu trúc quản lý cảm biến cổng
struct GateSensor {
  const int pin;
  const char* sensorIdentifier;   // ID định danh cảm biến, ví dụ: "SENSOR_VAO_1"
  const char* gateAreaMqttId;     // Phần ID cho topic MQTT, ví dụ: "entry_approach"
  const char* eventTypeOnActive;  // Sự kiện publish khi active, ví dụ: "presence_detected" hoặc "vehicle_passed"
  bool isActive;                  // Trạng thái logic hiện tại của cảm biến (true = active)
  bool prevIsActive;              // Trạng thái logic đã publish trước đó
  unsigned long lastDebounceTime; // Thời gian cuối cùng phát hiện thay đổi (cho debounce)
  const int activeLogicLevel;     // Mức logic được coi là active (HIGH hoặc LOW)
  bool autoCloseBarrier;          // Cờ xác định cảm biến này có tự động đóng rào không
  Servo* associatedServo;         // Servo liên quan (nếu có autoCloseBarrier)
  int servoClosePosition;         // Vị trí đóng của servo liên quan
  const char* barrierTypeForState; // "entry" hoặc "exit" để publish state
};

GateSensor gateSensors[] = {
  {SENSOR_VAO_1_PIN, "SENSOR_VAO_1", "entry_approach", "presence_detected", false, false, 0, HIGH, false, NULL, 0, ""},
  {SENSOR_VAO_2_PIN, "SENSOR_VAO_2", "entry_passed",   "vehicle_passed",    false, false, 0, HIGH, true, &servoEntry, 0, "entry"}, // Tự động đóng rào vào (vị trí 0)
  {SENSOR_RA_1_PIN,  "SENSOR_RA_1",  "exit_approach",  "presence_detected", false, false, 0, HIGH, false, NULL, 0, ""},
  {SENSOR_RA_2_PIN,  "SENSOR_RA_2",  "exit_passed",    "vehicle_passed",    false, false, 0, LOW,  true, &servoExit, 90, "exit"}  // Tự động đóng rào ra (vị trí 90), active LOW
};
const int NUM_GATE_SENSORS = sizeof(gateSensors) / sizeof(GateSensor);
const unsigned long GATE_SENSOR_DEBOUNCE_INTERVAL = 500; // Thời gian debounce cho cảm biến cổng (ms)


// Cấu trúc quản lý cảm biến chỗ đỗ (giữ nguyên như trước)
struct ParkingSlotSensor {
  const int pin;
  const char* slotMqttId;
  bool isOccupied;
  bool prevIsOccupied;
  unsigned long lastDebounceTime;
};

ParkingSlotSensor parkingSlots[] = {
  {SLOT_SENSOR_PIN_1, "S1", false, false, 0},
  {SLOT_SENSOR_PIN_2, "S2", false, false, 0},
  // {SLOT_SENSOR_PIN_3, "S3", false, false, 0},
  // {SLOT_SENSOR_PIN_4, "S4", false, false, 0}
};
const int NUM_PARKING_SLOTS = sizeof(parkingSlots) / sizeof(ParkingSlotSensor);
const unsigned long SLOT_DEBOUNCE_DELAY = 1000;


// --- CÁC HÀM TIỆN ÍCH (connectWiFi, syncNTPTime, publishMqttMessage, publishBarrierState, publishGateSensorEvent, publishParkingSlotStatus, messageHandler, connectAWSIoT) ---
// Giữ nguyên các hàm này như trong phiên bản "esp32_parking_controller_final"
// Lưu ý: publishGateSensorEvent sẽ được gọi từ hàm xử lý cảm biến cổng mới.

void connectWiFi() {
  Serial.print("Đang kết nối WiFi: ");
  Serial.println(SECRET_WIFI_SSID);
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nĐã kết nối WiFi!");
  Serial.print("Địa chỉ IP: ");
  Serial.println(WiFi.localIP());
}

void syncNTPTime() {
  Serial.print("Đang cài đặt thời gian qua NTP server: ");
  Serial.println(NTP_SERVER);
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Không thể lấy thời gian từ NTP server. Đang thử lại...");
    delay(1000);
    return; 
  }
  Serial.println("Đồng bộ thời gian thành công.");
  Serial.println(&timeinfo, "Thời gian hiện tại: %A, %B %d %Y %H:%M:%S");
}

void publishMqttMessage(const char* topic, const JsonDocument& doc) {
  if (!client.connected()) {
    Serial.println("MQTT chưa kết nối. Bỏ qua publish.");
    return;
  }
  char jsonBuffer[256];
  size_t n = serializeJson(doc, jsonBuffer);
  Serial.print("Đang publish tới topic: "); Serial.println(topic);
  Serial.print("Payload: "); Serial.println(jsonBuffer);
  if (client.publish(topic, jsonBuffer, n)) {
    Serial.println("Publish OK");
  } else {
    Serial.println("Publish THẤT BẠI");
  }
}

void publishBarrierState(const char* barrierType, const char* state) {
  StaticJsonDocument<128> doc;
  doc["thing_id"] = SECRET_AWS_THING_NAME;
  doc["barrier_id"] = (strcmp(barrierType, "entry") == 0) ? "entry_barrier_1" : "exit_barrier_1";
  doc["state"] = state;
  char timeBuffer[30];
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    doc["timestamp"] = timeBuffer;
  }
  char topicBuffer[128];
  sprintf(topicBuffer, "parking/barrier/%s/%s/state", SECRET_AWS_THING_NAME, barrierType);
  publishMqttMessage(topicBuffer, doc);
}

void publishGateSensorEvent(const char* sensorPinIdentifier, const char* gateArea, const char* eventType) {
  StaticJsonDocument<128> doc;
  doc["thing_id"] = SECRET_AWS_THING_NAME;
  doc["sensor_id"] = sensorPinIdentifier;
  doc["event"] = eventType;
  doc["gate_area"] = gateArea;
  char timeBuffer[30];
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    doc["timestamp"] = timeBuffer;
  }
  char topicBuffer[128];
  sprintf(topicBuffer, "parking/sensor/%s/%s/event", SECRET_AWS_THING_NAME, gateArea);
  publishMqttMessage(topicBuffer, doc);
}

void publishParkingSlotStatus(const char* slotMqttId, bool isOccupied) {
  StaticJsonDocument<128> doc;
  doc["thing_id"] = SECRET_AWS_THING_NAME;
  doc["slot_id"] = slotMqttId;
  doc["status"] = isOccupied ? "occupied" : "vacant";
  char timeBuffer[30];
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    doc["timestamp"] = timeBuffer;
  }
  char topicBuffer[128];
  sprintf(topicBuffer, "parking/slot/%s/%s/status", SECRET_AWS_THING_NAME, slotMqttId);
  publishMqttMessage(topicBuffer, doc);
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("Tin nhắn đến từ topic: ["); Serial.print(topic); Serial.print("], Payload: ");
  String messageTemp;
  for (int i = 0; i < length; i++) { messageTemp += (char)payload[i]; }
  Serial.println(messageTemp);

  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) { Serial.print(F("Lỗi deserializeJson(): ")); Serial.println(error.f_str()); return; }

  const char* receivedCommand = doc["command"];
  char entryBarrierCommandTopic[128]; char exitBarrierCommandTopic[128];
  sprintf(entryBarrierCommandTopic, "parking/barrier/%s/entry/command", SECRET_AWS_THING_NAME);
  sprintf(exitBarrierCommandTopic, "parking/barrier/%s/exit/command", SECRET_AWS_THING_NAME);

  if (strcmp(topic, entryBarrierCommandTopic) == 0) {
    if (strcmp(receivedCommand, "open") == 0) {
      servoEntry.write(90); Serial.println("Servo Rào Vào: MỞ (Lệnh từ server)");
      publishBarrierState("entry", "opened_command");
    } else if (strcmp(receivedCommand, "close") == 0) {
      servoEntry.write(0); Serial.println("Servo Rào Vào: ĐÓNG (Lệnh từ server)");
      publishBarrierState("entry", "closed_command");
    }
  } else if (strcmp(topic, exitBarrierCommandTopic) == 0) {
    if (strcmp(receivedCommand, "open") == 0) {
      servoExit.write(0); Serial.println("Servo Rào Ra: MỞ (Lệnh từ server)");
      publishBarrierState("exit", "opened_command");
    } else if (strcmp(receivedCommand, "close") == 0) {
      servoExit.write(90); Serial.println("Servo Rào Ra: ĐÓNG (Lệnh từ server)");
      publishBarrierState("exit", "closed_command");
    }
  }
}

void connectAWSIoT() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo) || timeinfo.tm_year < (2023 - 1900)) {
    Serial.println("Thời gian chưa được đồng bộ. Đang đồng bộ..."); syncNTPTime();
    if (!getLocalTime(&timeinfo) || timeinfo.tm_year < (2023 - 1900)) {
        Serial.println("Không thể đồng bộ thời gian sau khi thử lại. Kết nối MQTT có thể thất bại.");
    }
  }
  net.setCACert(SECRET_AWS_ROOT_CA_PEM);
  net.setCertificate(SECRET_AWS_CERT_CRT_PEM);
  net.setPrivateKey(SECRET_AWS_PRIVATE_KEY_PEM);
  client.setServer(SECRET_AWS_IOT_ENDPOINT, AWS_MQTT_PORT);
  client.setCallback(messageHandler);
  Serial.println("Đang thử kết nối AWS IoT MQTT...");
  if (client.connect(SECRET_AWS_THING_NAME)) {
    Serial.println("Đã kết nối AWS IoT MQTT!");
    char entryCmdTopic[128], exitCmdTopic[128];
    sprintf(entryCmdTopic, "parking/barrier/%s/entry/command", SECRET_AWS_THING_NAME);
    sprintf(exitCmdTopic, "parking/barrier/%s/exit/command", SECRET_AWS_THING_NAME);
    if(client.subscribe(entryCmdTopic)){ Serial.print("Đã subscribe tới: "); Serial.println(entryCmdTopic); } 
    else { Serial.print("Lỗi subscribe tới: "); Serial.println(entryCmdTopic); }
    if(client.subscribe(exitCmdTopic)){ Serial.print("Đã subscribe tới: "); Serial.println(exitCmdTopic); } 
    else { Serial.print("Lỗi subscribe tới: "); Serial.println(exitCmdTopic); }
  } else {
    Serial.print("Kết nối AWS IoT MQTT thất bại, mã lỗi rc="); Serial.print(client.state());
    char lastError[100]; net.getLastSSLError(lastError, 100);
    Serial.print(", Lỗi WiFiClientSecure: "); Serial.println(lastError);
    Serial.println(" Thử lại sau 5 giây...");
  }
}


// --- HÀM SETUP CHÍNH ---
void setup() {
  Serial.begin(115200);
  while (!Serial); 
  delay(1000);

  Serial.println("--- Bắt đầu chương trình điều khiển bãi đỗ xe ESP32 (Refactored) ---");

  // Khởi tạo pin cho cảm biến cổng
  for (int i = 0; i < NUM_GATE_SENSORS; i++) {
    pinMode(gateSensors[i].pin, INPUT_PULLUP); // Giả định tất cả dùng PULLUP ban đầu
    // Đọc trạng thái ban đầu cho cảm biến cổng
    bool initialPinState = digitalRead(gateSensors[i].pin);
    gateSensors[i].isActive = (initialPinState == gateSensors[i].activeLogicLevel);
    gateSensors[i].prevIsActive = gateSensors[i].isActive;
  }

  // Khởi tạo pin cho cảm biến chỗ đỗ (giữ nguyên)
  for (int i = 0; i < NUM_PARKING_SLOTS; i++) {
    pinMode(parkingSlots[i].pin, INPUT_PULLUP); 
    parkingSlots[i].isOccupied = (digitalRead(parkingSlots[i].pin) == LOW); // Giả định LOW = occupied
    parkingSlots[i].prevIsOccupied = parkingSlots[i].isOccupied;
  }

  // Gắn và đặt vị trí ban đầu cho servo
  servoEntry.attach(SERVO_ENTRY_PIN);
  servoExit.attach(SERVO_EXIT_PIN);
  servoEntry.write(0);  // Rào vào đóng
  servoExit.write(90); // Rào ra đóng
  Serial.println("Đã khởi tạo Servo.");

  connectWiFi();
}

// --- HÀM XỬ LÝ LOGIC CẢM BIẾN CỔNG (ĐÃ REFACTOR) ---
void handleGateSensors() {
  unsigned long currentTime = millis();
  for (int i = 0; i < NUM_GATE_SENSORS; i++) {
    bool currentPinState = digitalRead(gateSensors[i].pin);
    bool currentlyDetectedActive = (currentPinState == gateSensors[i].activeLogicLevel);

    if (currentlyDetectedActive != gateSensors[i].isActive) { // Nếu có sự thay đổi tiềm năng
      if (currentTime - gateSensors[i].lastDebounceTime > GATE_SENSOR_DEBOUNCE_INTERVAL) {
        // Trạng thái đã ổn định VÀ khác với trạng thái đã publish trước đó
        if (currentlyDetectedActive != gateSensors[i].prevIsActive) {
          gateSensors[i].isActive = currentlyDetectedActive; // Cập nhật trạng thái logic
          
          if (gateSensors[i].isActive) { // Chỉ publish khi cảm biến trở nên active
            publishGateSensorEvent(gateSensors[i].sensorIdentifier, gateSensors[i].gateAreaMqttId, gateSensors[i].eventTypeOnActive);
            
            // Xử lý tự động đóng rào nếu cảm biến này được cấu hình
            if (gateSensors[i].autoCloseBarrier && gateSensors[i].associatedServo != NULL) {
              Serial.print("Cảm biến "); Serial.print(gateSensors[i].sensorIdentifier);
              Serial.println(" kích hoạt: Tự động đóng rào liên quan.");
              gateSensors[i].associatedServo->write(gateSensors[i].servoClosePosition);
              publishBarrierState(gateSensors[i].barrierTypeForState, "closed_auto");
            }
          } else {
            // Tùy chọn: Publish sự kiện "cleared" khi cảm biến không còn active
            // publishGateSensorEvent(gateSensors[i].sensorIdentifier, gateSensors[i].gateAreaMqttId, "presence_cleared");
          }
          gateSensors[i].prevIsActive = gateSensors[i].isActive; // Cập nhật trạng thái đã publish
        }
      }
      gateSensors[i].lastDebounceTime = currentTime; // Reset thời gian debounce cho lần đọc thay đổi này
    } else {
      // Nếu không có thay đổi so với isActive, nhưng trạng thái đọc được đã ổn định và trước đó nó khác prevIsActive
      if (currentlyDetectedActive == gateSensors[i].isActive &&
          currentlyDetectedActive != gateSensors[i].prevIsActive &&
          (currentTime - gateSensors[i].lastDebounceTime > GATE_SENSOR_DEBOUNCE_INTERVAL)) {
        if (gateSensors[i].isActive) { // Chỉ publish nếu trạng thái active ổn định khác với prevIsActive
            publishGateSensorEvent(gateSensors[i].sensorIdentifier, gateSensors[i].gateAreaMqttId, gateSensors[i].eventTypeOnActive);
             // Xử lý tự động đóng rào nếu cần (lặp lại logic ở trên nếu trạng thái active được xác nhận lại)
            if (gateSensors[i].autoCloseBarrier && gateSensors[i].associatedServo != NULL) {
              Serial.print("Cảm biến "); Serial.print(gateSensors[i].sensorIdentifier);
              Serial.println(" xác nhận active: Tự động đóng rào liên quan.");
              gateSensors[i].associatedServo->write(gateSensors[i].servoClosePosition);
              publishBarrierState(gateSensors[i].barrierTypeForState, "closed_auto");
            }
        }
        gateSensors[i].prevIsActive = gateSensors[i].isActive;
      }
    }
  }
}


// --- HÀM XỬ LÝ LOGIC CẢM BIẾN CHỖ ĐỖ (Giữ nguyên như trước) ---
void handleParkingSlotSensors() {
  unsigned long currentTime = millis();
  for (int i = 0; i < NUM_PARKING_SLOTS; i++) {
    bool currentPinStateIsLow = (digitalRead(parkingSlots[i].pin) == LOW); 
    bool currentlyDetectedOccupied = currentPinStateIsLow; 

    if (currentlyDetectedOccupied != parkingSlots[i].isOccupied) { 
      if (currentTime - parkingSlots[i].lastDebounceTime > SLOT_DEBOUNCE_DELAY) {
        if (currentlyDetectedOccupied != parkingSlots[i].prevIsOccupied) { 
          parkingSlots[i].isOccupied = currentlyDetectedOccupied; 
          publishParkingSlotStatus(parkingSlots[i].slotMqttId, parkingSlots[i].isOccupied);
          parkingSlots[i].prevIsOccupied = parkingSlots[i].isOccupied; 
        }
      }
      parkingSlots[i].lastDebounceTime = currentTime; 
    } else {
      if (currentlyDetectedOccupied == parkingSlots[i].isOccupied && 
          currentlyDetectedOccupied != parkingSlots[i].prevIsOccupied && 
          (currentTime - parkingSlots[i].lastDebounceTime > SLOT_DEBOUNCE_DELAY)) {
        publishParkingSlotStatus(parkingSlots[i].slotMqttId, parkingSlots[i].isOccupied);
        parkingSlots[i].prevIsOccupied = parkingSlots[i].isOccupied;
      }
    }
  }
}

// --- HÀM LOOP CHÍNH ---
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi bị ngắt kết nối. Đang kết nối lại...");
    connectWiFi();
  }
  
  if (!client.connected()) {
    connectAWSIoT(); 
    delay(5000);     
    return;          
  }

  client.loop(); 

  // Xử lý logic cho các cảm biến cổng (đã refactor)
  handleGateSensors();

  // Xử lý logic cho 4 cảm biến chỗ đỗ
  handleParkingSlotSensors();
  
  // delay(10); // Một delay nhỏ có thể hữu ích để ổn định, nhưng client.loop() nên được gọi thường xuyên
}
