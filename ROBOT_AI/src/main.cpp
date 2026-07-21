/**
 * @file main.cpp
 * @brief Chương trình chính gom các phân hệ điều khiển robot Mecanum và quản lý chẩn đoán module.
 */

#include "robot_global.h"
#include "test_module.h"

// =============================================================================
// KHAI BÁO VÀ ĐỊNH NGHĨA CÁC ĐỐI TƯỢNG PHẦN CỨNG (GLOBAL INSTANCES)
// =============================================================================
#include "PinMap.h"
BTS7960 motorFL(MOTOR_FL_RPWM, MOTOR_FL_LPWM);
BTS7960 motorFR(MOTOR_FR_RPWM, MOTOR_FR_LPWM);
BTS7960 motorRL(MOTOR_RL_RPWM, MOTOR_RL_LPWM);
BTS7960 motorRR(MOTOR_RR_RPWM, MOTOR_RR_LPWM);
Motor car(motorFL, motorFR, motorRL, motorRR);

MPU6050Sensor mpu;
bool mpuOk = false;
unsigned long lastMpuUpdate = 0;
const unsigned long MPU_INTERVAL = 20;

ROS2BridgeManager ros2Bridge;

// Khai báo và định nghĩa các biến trạng thái vận hành của xe
OperatingMode currentMode = MODE_MANUAL;
AutoState currentAutoState = AUTO_STOP;
String currentMoveDir = "dung";
int currentSpeed = 0;
bool isAvoidanceActive = false;
unsigned long autoModeStartTime = 0;
bool bypassSensorCheck = false;

// Ngưỡng khoảng cách cảnh báo và hằng số thời gian chẩn đoán
const float OBSTACLE_TRIGGER_CM = 50.0f;
const float OBSTACLE_CLEAR_CM = 70.0f;

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    // 1. Khởi tạo cổng Serial chính
    Serial.begin(115200);
    
    // Chờ cổng CDC USB kết nối (tối đa 2.5 giây để tránh treo khi chạy pin)
    unsigned long start_wait = millis();
    while (!Serial && (millis() - start_wait < 2500)) {
        delay(10);
    }

    Serial.println(F("\n======================================================="));
    Serial.println(F("🤖 ROBOT MECANUM - HỆ THỐNG ĐIỀU KHIỂN & CHẨN ĐOÁN CẢM BIẾN"));
    Serial.println(F("======================================================="));

    // 2. Khởi tạo cảm biến siêu âm HC-SR04
    Serial.println(F("[1/7] Khoi tao cam bien sieu am HC-SR04..."));
    HC_SR04_Init();
    HC_SR04_SetWarningDistance(OBSTACLE_TRIGGER_CM);

    // 3. Khởi tạo cảm biến IMU MPU6050
    Serial.println(F("[2/7] Khoi tao MPU6050 (SDA=18, SCL=19)..."));
    mpuOk = mpu.begin(18, 19);
    if (mpuOk) {
        Serial.println(F("  MPU6050 OK"));
    } else {
        Serial.println(F("  MPU6050 THẤT BẠI! Xe vẫn hoạt động bình thường."));
    }

    // 4. Khởi tạo các Driver động cơ BTS7960
    Serial.printf("[3/7] Khoi tao motorFL (RPWM=%d, LPWM=%d)...\n", MOTOR_FL_RPWM, MOTOR_FL_LPWM);
    motorFL.begin();
    Serial.printf("[4/7] Khoi tao motorFR (RPWM=%d, LPWM=%d)...\n", MOTOR_FR_RPWM, MOTOR_FR_LPWM);
    motorFR.begin();
    Serial.printf("[5/7] Khoi tao motorRL (RPWM=%d, LPWM=%d)...\n", MOTOR_RL_RPWM, MOTOR_RL_LPWM);
    motorRL.begin();
    Serial.printf("[6/7] Khoi tao motorRR (RPWM=%d, LPWM=%d)...\n", MOTOR_RR_RPWM, MOTOR_RR_LPWM);
    motorRR.begin();

    // 5. Khởi tạo còi cảnh báo MH-FMD (Active Low, Pulsing beeps)
    Serial.println(F("[7/7] Khoi tao coi active buzzer MH-FMD (GPIO41)..."));
    MH_FMD_Init();
    MH_FMD_SetThreshold(OBSTACLE_TRIGGER_CM);

    // 6. Khởi tạo các phân hệ chức năng
    Serial.println(F("[System] Khoi tao phan he dieu khien bang tay..."));
    clien_dieukhien_Init();

    Serial.println(F("[System] Khoi tao phan he chay tu dong..."));
    auto_run_Init();
    autoModeStartTime = millis();

    Serial.println(F("[System] Khoi tao giao truyen thong ROS2 (50Hz)..."));
    ros2Bridge.begin(&Serial, 50);

    Serial.println(F("=== HỆ THỐNG SẴN SÀNG ==="));
    Serial.println(F("---------------------------------------------"));
    
    // Khởi tạo phân hệ kiểm tra module (hỏi chọn chế độ hoạt động ban đầu)
    test_module_Init();
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
    // 1. Cập nhật trạng thái cảm biến siêu âm (nếu không bị tạm dừng bởi test module)
    if (should_run_sensor_update()) {
        HC_SR04_Update();
    }

    // 2. Cập nhật góc IMU MPU6050 định kỳ không block
    if (mpuOk) {
        unsigned long now = millis();
        if (now - lastMpuUpdate >= MPU_INTERVAL) {
            mpu.update();
            lastMpuUpdate = now;
        }
    }

    // 3. Cập nhật phân hệ test module (đọc Serial và xử lý lệnh test)
    test_module_Update();

    // 4. Cập nhật phân hệ giao tiếp 2 chiều ROS2 (nhận cmd_vel và bắn Telemetry 50Hz)
    ros2Bridge.update();

    // 4. Cập nhật các luồng hoạt động chính dựa vào trạng thái chế độ
    float frontDist = HC_SR04_GetFrontDistance();

    if (!is_in_test_mode()) {
        // Trong chế độ hoạt động bình thường (Manual hoặc Auto):
        // Cập nhật còi cảnh báo dựa trên khoảng cách trước
        MH_FMD_Update(frontDist);

        // Cập nhật phân hệ chạy tự động tránh vật cản (auto_run.cpp)
        auto_run_Update();

        // Cập nhật tiến trình chạy thử động cơ (nếu có cuộc gọi từ phím tắt)
        updateMotorTest();
    } else {
        // Trong chế độ kiểm tra module:
        // Cập nhật còi cảnh báo để xử lý âm báo test/beep thủ công
        MH_FMD_Update(frontDist);

        // Cập nhật tiến trình test động cơ (cho phép test_motor chạy bình thường)
        updateMotorTest();
    }
}