/**
 * @file main.cpp
 * @brief Chương trình chính cho ESP32-S3 Hardware Controller cho ROS2 + SLAM + Navigation2.
 *        Khởi động BLE hoàn toàn tự động trên pin/nguồn ngoài, không cần cổng Serial hay bấm phím.
 */

#include <Arduino.h>
#include "PinMap.h"
#include "Config.h"
#include "robot_global.h"

// Hardware drivers
#include "Motor.h"
#include "Sensor_HC_SR04.h"
#include "EncoderManager.h"
#include "Mpu6050.h"

// Core Controllers & Managers
#include "MovementController.h"
#include "Managers/UltrasonicManager.h"
#include "Managers/IMUManager.h"
#include "Managers/BuzzerManager.h"
#include "Managers/RobotStateManager.h"
#include "Managers/CommandManager.h"
#include "Protocol/SerialProtocol.h"
#include "BLEManager.h"

// ROS2 Protocol Bridge
#include "ROS2Bridge.h"

// Simple Task Scheduler
#include "Core/TaskScheduler.h"

// =============================================================================
// KHAI BÁO CÁC ĐỐI TƯỢNG PHẦN CỨNG TOÀN CỤC
// =============================================================================

Motor motorFL(MOTOR1_PWM_L, MOTOR1_PWM_R); // Front Left
Motor motorFR(MOTOR2_PWM_L, MOTOR2_PWM_R); // Front Right
Motor motorRL(MOTOR3_PWM_L, MOTOR3_PWM_R); // Rear Left
Motor motorRR(MOTOR4_PWM_L, MOTOR4_PWM_R); // Rear Right

MovementController moveControl(&motorFL, &motorFR, &motorRL, &motorRR);
ROS2Bridge ros2Bridge;
TaskScheduler mainScheduler;

// Khai báo các biến trạng thái toàn cục từ robot_global.h
CarMode currentMode = MODE_MANUAL;
AutoState currentAutoState = AUTO_IDLE;
String currentMoveDir = "Dừng";
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
    // 1. Khởi tạo cổng Serial chính (Non-blocking)
    Serial.begin(115200);

    Serial.println(F("\n======================================================="));
    Serial.println(F("🤖 ROBOT MECANUM - HARDWARE CONTROLLER FOR ROS2 + SLAM"));
    Serial.println(F("======================================================="));

    // 2. Step 1: Initialize Hardware Drivers
    motorFL.begin();
    motorFR.begin();
    motorRL.begin();
    motorRR.begin();

#if ENCODER_ENABLED
    encoderManager.begin();
#endif

    UltrasonicManager::getInstance().begin();
    IMUManager::getInstance().begin(18, 19);
    BuzzerManager::getInstance().begin();
    RobotStateManager::getInstance().begin();

    // 3. Step 2: Initialize CommandManager
    CommandManager::getInstance().begin(&moveControl);

    // 4. Step 3: Initialize MovementController & SerialProtocol
    SerialProtocol::getInstance().begin(&Serial);

    // 5. Step 4 & 5: Initialize BLEManager & Start Advertising Immediately
    BLEManager::getInstance().begin();

    // 6. Khởi tạo các phân hệ chức năng
    clien_dieukhien_Init();
    auto_run_Init();
    autoModeStartTime = millis();
    ros2Bridge.begin(&Serial, 50);

    // 7. Đăng ký các task tuần hoàn cho Scheduler
    mainScheduler.registerTask(1, []() {
        test_module_Update();
    });

    mainScheduler.registerTask(10, []() {
        if (!is_in_test_mode()) {
            moveControl.update();
            updateMotorTest();
        } else {
            updateMotorTest();
        }
    });

    mainScheduler.registerTask(20, []() {
#if ENCODER_ENABLED
        encoderManager.update();
#endif
        if (should_run_sensor_update()) {
            bool frontActive = true;
            bool rearActive = true;
            if (is_in_test_mode()) {
                frontActive = is_sensor_front_test_active();
                rearActive = is_sensor_rear_test_active();
            }
            UltrasonicManager::getInstance().update(frontActive, rearActive);
        }

        if (!is_sensor_isolated_mode()) {
            IMUManager::getInstance().update();
            BuzzerManager::getInstance().update();
        }
    });

    // Task 50ms (20Hz): Phát tín hiệu Telemetry thống nhất lên ROS2 / Pi / BLE
    mainScheduler.registerTask(50, []() {
        SerialProtocol::getInstance().sendTelemetry();
    });

    // 8. Step 6: Robot Ready
    RobotStateManager::getInstance().setState(STATE_READY);
    Serial.println(F("=== HỆ THỐNG HARDWARE CONTROLLER SẴN SÀNG ==="));
    Serial.println(F("🤖 [System] Robot Ready. BLE Advertising: ESP32_Robot"));
    
    test_module_Init();
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
    // Nhận & Parse lệnh Serial và BLE non-blocking
    SerialProtocol::getInstance().update();
    BLEManager::getInstance().update();
    ros2Bridge.update();

    // Thực thi cyclic task
    mainScheduler.tick();
}