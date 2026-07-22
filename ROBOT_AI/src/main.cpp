/**
 * @file main.cpp
 * @brief Chương trình chính gom các phân hệ điều khiển robot Mecanum và quản lý chẩn đoán module.
 */

#include "robot_global.h"
#include "test_module.h"
#include "MovementController.h"
#include "Scheduler/Scheduler.h"
#include "SensorManager/SensorManager.h"
#include "EventBus/EventBus.h"
#include "EncoderManager.h"

EncoderManager& encoderManager = EncoderManager::getInstance();

// =============================================================================
// KHAI BÁO VÀ ĐỊNH NGHĨA CÁC ĐỐI TƯỢNG PHẦN CỨNG (GLOBAL INSTANCES)
// =============================================================================
#include "PinMap.h"
BTS7960 motorFL(MOTOR_FL_RPWM, MOTOR_FL_LPWM);
BTS7960 motorFR(MOTOR_FR_RPWM, MOTOR_FR_LPWM);
BTS7960 motorRL(MOTOR_RL_RPWM, MOTOR_RL_LPWM);
BTS7960 motorRR(MOTOR_RR_RPWM, MOTOR_RR_LPWM);
Motor car(motorFL, motorFR, motorRL, motorRR);

MovementController moveControl(car);

MPU6050Sensor mpu;
bool mpuOk = false;
unsigned long lastMpuUpdate = 0;
const unsigned long MPU_INTERVAL = 20;

ROS2BridgeManager ros2Bridge;
Scheduler mainScheduler;

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
    Serial.printf("[7/7] Khoi tao coi active buzzer MH-FMD (GPIO%d)...\n", MH_FMD_PIN);
    MH_FMD_Init();
    MH_FMD_SetThreshold(OBSTACLE_TRIGGER_CM);

#if ENCODER_ENABLED
    Serial.println(F("[System] Khoi tao module Encoder..."));
    encoderManager.begin();
#endif

    // 6. Khởi tạo các phân hệ chức năng
    Serial.println(F("[System] Khoi tao phan he dieu khien bang tay..."));
    clien_dieukhien_Init();

    Serial.println(F("[System] Khoi tao phan he chay tu dong..."));
    auto_run_Init();
    autoModeStartTime = millis();

    Serial.println(F("[System] Khoi tao giao truyen thong ROS2 (50Hz)..."));
    ros2Bridge.begin(&Serial, 50);

    // 7. Đăng ký các task tuần hoàn cho Scheduler
    // Task 1ms: Cập nhật luồng chẩn đoán
    mainScheduler.registerTask(1, []() {
        test_module_Update();
    });

    // Task 10ms: Điều khiển chuyển động và chu kỳ test motor
    mainScheduler.registerTask(10, []() {
        if (!is_in_test_mode()) {
            moveControl.update();
            updateMotorTest();
        } else {
            updateMotorTest();
        }
    });

    // Task 20ms: Đo cảm biến, cập nhật SensorManager và kiểm tra khoảng cách an toàn
    mainScheduler.registerTask(20, []() {
#if ENCODER_ENABLED
        // Cập nhật bộ đếm và trạng thái Encoder
        encoderManager.update();
        SensorManager::getInstance().publishEncoders(
            encoderManager.getPulse(0),
            encoderManager.getPulse(1),
            encoderManager.getPulse(2),
            encoderManager.getPulse(3)
        );
#endif

        if (should_run_sensor_update()) {
            HC_SR04_Update();
        }

        if (mpuOk) {
            mpu.update();
            SensorManager::getInstance().publishIMU(
                mpu.getRoll(), mpu.getPitch(), mpu.getYaw(),
                mpu.getAccelX(), mpu.getAccelY(), mpu.getAccelZ(),
                mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ()
            );
        }

        float frontDist = HC_SR04_GetFrontDistance();
        float rearDist = HC_SR04_GetRearDistance();
        SensorManager::getInstance().publishUltrasonic(frontDist, rearDist);

        // Phát tín hiệu cảnh báo trên còi
        MH_FMD_Update(frontDist);

        // Bắn sự kiện lên EventBus nếu có vật cản trước
        if (frontDist > 0.0f && frontDist < OBSTACLE_TRIGGER_CM && !bypassSensorCheck) {
            Event obstacleEvent;
            obstacleEvent.type = EVENT_OBSTACLE_DETECTED;
            obstacleEvent.timestamp = millis();
            obstacleEvent.data.distance = frontDist;
            EventBus::getInstance().publish(obstacleEvent);
        }
    });

    // Task 50ms: Chạy chế độ tự động tránh vật cản
    mainScheduler.registerTask(50, []() {
        if (!is_in_test_mode()) {
            auto_run_Update();
        }
    });

    // Task 10000ms (10s): Xuất thông tin chẩn đoán
    mainScheduler.registerTask(10000, []() {
        const SensorData& data = SensorManager::getInstance().getSensorData();
        Serial.printf(
            "ESP32_DATA front=%.2f rear=%.2f ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f roll=%.2f pitch=%.2f yaw=%.2f\n",
            data.front_distance, data.rear_distance, data.accel_x, data.accel_y, data.accel_z,
            data.gyro_x, data.gyro_y, data.gyro_z,
            data.roll, data.pitch, data.yaw);
    });

    Serial.println(F("=== HỆ THỐNG SẴN SÀNG ==="));
    Serial.println(F("---------------------------------------------"));
    
    // Khởi tạo phân hệ kiểm tra module (hỏi chọn chế độ hoạt động ban đầu)
    test_module_Init();
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
    // Luôn nhận Serial từ RPi nhanh nhất có thể để tránh tràn bộ đệm UART
    ros2Bridge.update();

    // Thực thi cyclic task
    mainScheduler.tick();
}