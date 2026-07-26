/**
 * Robot_Tu_Hanh.ino
 * Firmware Arduino/ESP32 cho Xe Robot Tự Hành Mecanum giao tiếp với ROS2 qua Text Protocol.
 *
 * Tương thích hoàn toàn với gói ROS2 `robot_serial` (file `serial_node.py` và `sensor_parser.py`).
 *
 * Giao thức nhận lệnh từ Pi:
 *   - Lệnh di chuyển: "<hướng> <tốc_độ>\n" (Ví dụ: "tien 150", "lui 120", "xoay_trai 150")
 *   - Lệnh dừng: "dung\n"
 *   - Lệnh bật dữ liệu cảm biến: "t on\n"
 *   - Lệnh tắt dữ liệu cảm biến: "t off\n"
 *
 * Giao thức gửi dữ liệu lên Pi (Telemetry):
 *   - Định dạng: [TELEMETRY] MODE: <mode>, STATUS: <status>, BATTERY: <V>V, FRONT: <cm>cm, REAR: <cm>cm, Yaw = <yaw>, Pitch = <pitch>, Roll = <roll>, Ax = <ax>, Ay = <ay>, Az = <az>, Gx = <gx>, Gy = <gy>, Gz = <gz>, Dist = <dist>
 */

#include <Arduino.h>
#include <Wire.h>

// ==========================================
// 1. ĐỊNH NGHĨA CHÂN PIN (Cấu hình tùy mạch của bạn)
// ==========================================
// Chân điều khiển Động cơ (Ví dụ cho mạch cầu H điều khiển PWM + DIR)
#define PIN_FL_DIR  12
#define PIN_FL_PWM  13
#define PIN_FR_DIR  14
#define PIN_FR_PWM  27
#define PIN_RL_DIR  26
#define PIN_RL_PWM  25
#define PIN_RR_DIR  33
#define PIN_RR_PWM  32

// Chân cảm biến Siêu âm (Ultrasonic HC-SR04)
#define PIN_FRONT_TRIG  5
#define PIN_FRONT_ECHO  18
#define PIN_REAR_TRIG   19
#define PIN_REAR_ECHO   23

// Chân đọc điện áp PIN (Battery voltage)
#define PIN_BATTERY     34

// ==========================================
// 2. BIẾN TOÀN CỤC & TRẠNG THÁI
// ==========================================
bool enableTelemetry = true;           // Mặc định luôn tự động gửi telemetry khi khởi động
unsigned long lastTelemetryTime = 0;   // Thời gian gửi telemetry lần cuối
const unsigned long TELEMETRY_INTERVAL = 50; // Gửi dữ liệu tần số 20Hz (mỗi 50ms)

// Biến lưu thông tin cảm biến để gửi lên Pi
float batteryVoltage = 12.4;
float frontDistance = 50.0;
float rearDistance = 50.0;
float yaw = 0.0, pitch = 0.0, roll = 0.0;
float ax = 0.0, ay = 0.0, az = 9.81;
float gx = 0.0, gy = 0.0, gz = 0.0;
float encoderDistance = 0.0;
String robotMode = "ROS2";
String robotStatus = "OK";

// ==========================================
// 3. ĐIỀU KHIỂN ĐỘNG CƠ (Mecanum Driver)
// ==========================================
// Hàm điều khiển từng động cơ (speed: -255 đến 255)
void setMotor(int pinDir, int pinPwm, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(pinDir, HIGH);
    analogWrite(pinPwm, speed);
  } else {
    digitalWrite(pinDir, LOW);
    analogWrite(pinPwm, abs(speed));
  }
}

void driveFL(int speed) { setMotor(PIN_FL_DIR, PIN_FL_PWM, speed); }
void driveFR(int speed) { setMotor(PIN_FR_DIR, PIN_FR_PWM, speed); }
void driveRL(int speed) { setMotor(PIN_RL_DIR, PIN_RL_PWM, speed); }
void driveRR(int speed) { setMotor(PIN_RR_DIR, PIN_RR_PWM, speed); }

// Hàm dừng toàn bộ động cơ
void stopAll() {
  driveFL(0);
  driveFR(0);
  driveRL(0);
  driveRR(0);
}

// Hàm di chuyển xe Mecanum theo hướng và tốc độ
void controlMecanum(String direction, int speed) {
  if (speed == 0 || direction == "dung") {
    stopAll();
    return;
  }
  
  if (direction == "tien") {
    driveFL(speed);  driveFR(speed);
    driveRL(speed);  driveRR(speed);
  } 
  else if (direction == "lui") {
    driveFL(-speed); driveFR(-speed);
    driveRL(-speed); driveRR(-speed);
  } 
  else if (direction == "trai") { // Dịch ngang sang trái
    driveFL(-speed); driveFR(speed);
    driveRL(speed);  driveRR(-speed);
  } 
  else if (direction == "phai") { // Dịch ngang sang phải
    driveFL(speed);  driveFR(-speed);
    driveRL(-speed); driveRR(speed);
  } 
  else if (direction == "xoay_trai") { // Xoay ngược chiều kim đồng hồ
    driveFL(-speed); driveFR(speed);
    driveRL(-speed); driveFR(speed);
  } 
  else if (direction == "xoay_phai") { // Xoay thuận chiều kim đồng hồ
    driveFL(speed);  driveFR(-speed);
    driveRL(speed);  driveRR(-speed);
  }
  else if (direction == "cheo_tp") { // Chéo tiến phải
    driveFL(speed);  driveFR(0);
    driveRL(0);      driveRR(speed);
  }
  else if (direction == "cheo_tt") { // Chéo tiến trái
    driveFL(0);      driveFR(speed);
    driveRL(speed);  driveRR(0);
  }
  else if (direction == "cheo_sp") { // Chéo lùi phải
    driveFL(0);      driveFR(-speed);
    driveRL(-speed); driveRR(0);
  }
  else if (direction == "cheo_st") { // Chéo lùi trái
    driveFL(-speed); driveFR(0);
    driveRL(0);      driveRR(-speed);
  }
  else {
    stopAll(); // Không nhận diện được hướng -> Dừng cho an toàn
  }
}

// ==========================================
// 4. ĐỌC DỮ LIỆU CẢM BIẾN
// ==========================================
float readUltrasonic(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);
  
  long duration = pulseIn(pinEcho, HIGH, 30000); // Timeout 30ms (~5m)
  if (duration == 0) return 400.0; // Nếu lỗi, trả về khoảng cách tối đa
  return duration * 0.034 / 2.0;
}

void readSensors() {
  // 4.1 Đọc khoảng cách siêu âm
  frontDistance = readUltrasonic(PIN_FRONT_TRIG, PIN_FRONT_ECHO);
  rearDistance = readUltrasonic(PIN_REAR_TRIG, PIN_REAR_ECHO);
  
  // 4.2 Đọc cảm biến Pin (Ước tính tỉ lệ phân áp)
  int rawBat = analogRead(PIN_BATTERY);
  batteryVoltage = (rawBat / 4095.0) * 3.3 * 5.0; // Hệ số nhân 5.0 cho mạch phân áp 10k/47k
  if (batteryVoltage < 1.0) batteryVoltage = 12.4; // Giả lập nếu không nối pin
  
  // 4.3 Đọc IMU (Dưới đây là mô phỏng, nếu bạn dùng MPU6050 thật, hãy viết code đọc thực tế ở đây)
  // yaw, pitch, roll = ...
  // ax, ay, az, gx, gy, gz = ...
  
  // Trạng thái an toàn: Nếu cảm biến siêu âm phát hiện vật cản quá gần, tự động dừng khẩn cấp
  if (frontDistance < 10.0 || rearDistance < 10.0) {
    robotStatus = "ESTOP_OBSTACLE";
    stopAll();
  } else {
    robotStatus = "OK";
  }
}

// ==========================================
// 5. PHÂN TÍCH LỆNH TỪ SERIAL (Parser)
// ==========================================
void processCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  
  // Kiểm tra lệnh bật/tắt luồng dữ liệu
  if (line == "t on") {
    enableTelemetry = true;
    return;
  }
  if (line == "t off") {
    enableTelemetry = false;
    return;
  }

  // Kiểm tra lệnh chuyển chế độ (từ teleop hoặc Pi)
  if (line == "mode_manual") {
    robotMode = "MANUAL";
    stopAll();
    return;
  }
  if (line == "mode_auto") {
    robotMode = "AUTO";
    return;
  }
  
  // Phân tách lệnh di chuyển (Ví dụ: "tien 150" -> hướng: "tien", tốc độ: 150)
  int spaceIndex = line.indexOf(' ');
  if (spaceIndex > 0) {
    String direction = line.substring(0, spaceIndex);
    String speedStr = line.substring(spaceIndex + 1);
    int speed = speedStr.toInt();
    
    // Khi nhận được lệnh di chuyển, tự động chuyển về chế độ ROS2
    robotMode = "ROS2";
    controlMecanum(direction, speed);
  } else {
    // Nếu chỉ gửi hướng không kèm tốc độ (ví dụ từ teleop: "tien")
    String direction = line;
    if (direction == "dung" || direction == "x") {
      controlMecanum("dung", 0);
    } else {
      // Khi nhận được lệnh di chuyển, tự động chuyển về chế độ ROS2
      robotMode = "ROS2";
      controlMecanum(direction, 150); // Tốc độ mặc định 150
    }
  }
}

// ==========================================
// 6. GỬI DỮ LIỆU TELEMETRY
// ==========================================
void sendTelemetry() {
  // Lưu ý: Định dạng chuỗi in ra phải tuân thủ chuẩn tuyệt đối để regex của Pi phân tích được
  Serial.print("[TELEMETRY] MODE: ");
  Serial.print(robotMode);
  Serial.print(", STATUS: ");
  Serial.print(robotStatus);
  Serial.print(", BATTERY: ");
  Serial.print(batteryVoltage, 2);
  Serial.print("V, FRONT: ");
  Serial.print(frontDistance, 1);
  Serial.print("cm, REAR: ");
  Serial.print(rearDistance, 1);
  Serial.print("cm, Yaw = ");
  Serial.print(yaw, 2);
  Serial.print(", Pitch = ");
  Serial.print(pitch, 2);
  Serial.print(", Roll = ");
  Serial.print(roll, 2);
  Serial.print(", Ax = ");
  Serial.print(ax, 3);
  Serial.print(", Ay = ");
  Serial.print(ay, 3);
  Serial.print(", Az = ");
  Serial.print(az, 3);
  Serial.print(", Gx = ");
  Serial.print(gx, 4);
  Serial.print(", Gy = ");
  Serial.print(gy, 4);
  Serial.print(", Gz = ");
  Serial.print(gz, 4);
  Serial.print(", Dist = ");
  Serial.print(encoderDistance, 3);
  Serial.println(); // Dấu xuống dòng \n bắt buộc ở cuối
}

// ==========================================
// 7. SETUP VÀ LOOP
// ==========================================
void setup() {
  // Cấu hình chân Động cơ
  pinMode(PIN_FL_DIR, OUTPUT); pinMode(PIN_FL_PWM, OUTPUT);
  pinMode(PIN_FR_DIR, OUTPUT); pinMode(PIN_FR_PWM, OUTPUT);
  pinMode(PIN_RL_DIR, OUTPUT); pinMode(PIN_RL_PWM, OUTPUT);
  pinMode(PIN_RR_DIR, OUTPUT); pinMode(PIN_RR_PWM, OUTPUT);
  
  // Cấu hình chân cảm biến siêu âm
  pinMode(PIN_FRONT_TRIG, OUTPUT);
  pinMode(PIN_FRONT_ECHO, INPUT);
  pinMode(PIN_REAR_TRIG, OUTPUT);
  pinMode(PIN_REAR_ECHO, INPUT);
  
  // Khởi động cổng Serial (Tốc độ 115200 trùng khớp với Pi)
  Serial.begin(115200);
  stopAll();
}

void loop() {
  // Đọc dữ liệu lệnh từ Pi gửi xuống
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    processCommand(input);
  }
  
  // Đọc dữ liệu từ cảm biến liên tục
  readSensors();

  // Xử lý di chuyển theo chế độ tự động (AUTO) tránh vật cản độc lập
  if (robotMode == "AUTO") {
    if (frontDistance < 20.0) {
      // Có vật cản phía trước -> Lùi lại và xoay trái để tránh
      controlMecanum("lui", 120);
      delay(300);
      controlMecanum("xoay_trai", 150);
      delay(500);
      stopAll();
    } else {
      // Đường thoáng -> Tiếp tục tiến lên phía trước ở tốc độ 120
      controlMecanum("tien", 120);
    }
  }
  
  // Gửi telemetry định kỳ nếu Pi yêu cầu bật ("t on")
  unsigned long now = millis();
  if (enableTelemetry && (now - lastTelemetryTime >= TELEMETRY_INTERVAL)) {
    lastTelemetryTime = now;
    sendTelemetry();
  }
}
