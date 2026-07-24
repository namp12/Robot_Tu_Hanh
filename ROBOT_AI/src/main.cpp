/**
 * @file main.cpp
 * @brief Chương trình chính cho ESP32-S3 Hardware Controller cho ROS2 Humble + SLAM + Nav2.
 * Thiết kế theo kiến trúc Class Chuyên Nghiệp (Non-blocking, Millis(), Watchdog, Reconnect, Heartbeat).
 */

#include <Arduino.h>
#include "PinMap.h"
#include "Config.h"
#include "robot_global.h"

// Class-based Architecture Headers
#include "parameters.h"
#include "mode_manager.h"
#include "motion_controller.h"
#include "safety.h"
#include "SensorManager.h"
#include "SerialProtocol.h"
#include "bluetooth.h"
#include "test_module.h"

// Hardware Drivers
#include "Motor.h"
#include "Sensor_HC_SR04.h"
#include "EncoderManager.h"
#include "Mpu6050.h"
#include "MovementController.h"
#include "Managers/BuzzerManager.h"
#include "Managers/RobotStateManager.h"
#include "ROS2BridgeManager.h"

// =============================================================================
// KHAI BÁO ĐỐI TƯỢNG PHẦN CỨNG TOÀN CỤC
// =============================================================================

BTS7960 motorFL(MOTOR_FL_RPWM, MOTOR_FL_LPWM); // Front Left
BTS7960 motorFR(MOTOR_FR_RPWM, MOTOR_FR_LPWM); // Front Right
BTS7960 motorRL(MOTOR_RL_RPWM, MOTOR_RL_LPWM); // Rear Left
BTS7960 motorRR(MOTOR_RR_RPWM, MOTOR_RR_LPWM); // Rear Right

Motor car(motorFL, motorFR, motorRL, motorRR);
MovementController moveControl(car);
EncoderManager& encoderManager = EncoderManager::getInstance();
MPU6050Sensor mpu;
bool mpuOk = false;
unsigned long lastMpuUpdate = 0;
const unsigned long MPU_INTERVAL = 20;

ROS2BridgeManager ros2Bridge;

// Các biến trạng thái toàn cục từ robot_global.h
OperatingMode currentMode = MODE_MANUAL;
AutoState currentAutoState = AUTO_IDLE;
String currentMoveDir = "Dừng";
int currentSpeed = 0;
bool isAvoidanceActive = false;
unsigned long autoModeStartTime = 0;
bool bypassSensorCheck = false;

const float OBSTACLE_TRIGGER_CM = 50.0f;
const float OBSTACLE_CLEAR_CM = 70.0f;

// Instantiation cho các Controller Instance
SensorManager& sensorManager     = SensorManager::getInstance();
SafetyMonitor& safetyController   = SafetyMonitor::getInstance();
ModeManager& modeManager         = ModeManager::getInstance();
MotionController& motionController= MotionController::getInstance();
SerialProtocol& serialProtocol   = SerialProtocol::getInstance();

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);

    Serial.println(F("\n======================================================="));
    Serial.println(F("🤖 ROBOT MECANUM - ROS2 CLASS HARDWARE CONTROLLER"));
    Serial.println(F("======================================================="));

    // 1. Khởi tạo Motor Hardware Drivers
    motorFL.begin();
    motorFR.begin();
    motorRL.begin();
    motorRR.begin();

    // 2. Khởi tạo các Class Module
    ParameterManager::getInstance().initDefaults();
    modeManager.init(MODE_MANUAL);
    motionController.begin(&car);
    safetyController.init();
    sensorManager.begin();
    serialProtocol.begin(&Serial);
    BluetoothModule::getInstance().begin();

    // 3. Khởi tạo hỗ trợ legacy
    BuzzerManager::getInstance().begin();
    RobotStateManager::getInstance().begin();
    clien_dieukhien_Init();
    auto_run_Init();
    autoModeStartTime = millis();
    ros2Bridge.begin(&Serial, 50);

    RobotStateManager::getInstance().setState(STATE_READY);
    Serial.println(F("\n======================================================="));
    Serial.println(F("=== HỆ THỐNG HARDWARE CONTROLLER SẴN SÀNG ==="));
    Serial.println(F("⚙️ HƯỚNG DẪN CHỌN CHẾ ĐỘ (Gõ lệnh vào Serial Monitor):"));
    Serial.println(F("   1 hoặc 'manual' -> Bật Chế độ 1: Thủ Công (MANUAL)"));
    Serial.println(F("   2 hoặc 'auto'   -> Bật Chế độ 2: Tự Động Né Vật Cản (AUTO)"));
    Serial.println(F("   3 hoặc 'ros'    -> Bật Chế độ 3: Kết Nối ROS2 (ROS)"));
    Serial.println(F("   't on' / 't off'-> Bật/Tắt in màn hình Telemetry cảm biến"));
    Serial.println(F("=======================================================\n"));

    test_module_Init();
}

// =============================================================================
// MAIN LOOP - ĐÚNG THEO YÊU CẦU KIẾN TRÚC CHUẨN
// =============================================================================
void loop() {
    // 1. Cập nhật dữ liệu cảm biến
    sensorManager.update();

    // 2. Gửi dữ liệu cảm biến lên Raspberry Pi / Web / ROS2 (20Hz)
    sensorManager.sendData();

    // 3. Kiểm tra các quy tắc an toàn & ngắt khẩn cấp
    safetyController.update();

    // 4. Cập nhật trạng thái Mode (MANUAL, AUTO, ROS)
    // Mode chỉ quyết định nguồn lệnh lái, KHÔNG ảnh hưởng đọc cảm biến
    clien_dieukhien_Update();
    auto_run_Update();

    // 5. Cập nhật Motion Controller (Kiểm tra Safety trước khi xuất PWM)
    if (ModeManager::getInstance().getMode() == MODE_MANUAL) {
        moveControl.update();
    } else if (ModeManager::getInstance().getMode() == MODE_ROS) {
        motionController.update();
    }

    // 6. Cập nhật Serial Protocol (Heartbeat 1000ms, Watchdog & Reconnect)
    serialProtocol.update();
    BluetoothModule::getInstance().update();
    ros2Bridge.update();
}