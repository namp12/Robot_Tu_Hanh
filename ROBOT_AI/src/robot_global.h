/**
 * @file robot_global.h
 * @brief Khai báo các đối tượng phần cứng và trạng thái hoạt động dùng chung.
 */

#ifndef ROBOT_GLOBAL_H
#define ROBOT_GLOBAL_H

#include <Arduino.h>
#include "BTS7960.h"
#include "Motor.h"
#include "Sensor_HC_SR04.h"
#include "Mpu6050.h"
#include "MH_FMD.h"
#include "ROS2BridgeManager.h"

// =============================================================================
// ĐỊNH NGHĨA CHẾ ĐỘ HOẠT ĐỘNG VÀ TRẠNG THÁI AUTO
// =============================================================================
enum OperatingMode {
    MODE_MANUAL, ///< Chế độ điều khiển bằng tay (Thủ công)
    MODE_AUTO,   ///< Chế độ chạy tự động tránh vật cản
    MODE_ROS2    ///< Chế độ nhận lệnh từ Raspberry Pi (ROS2 Mode)
};

enum AutoState {
    AUTO_IDLE,       ///< Trạng thái chờ / tạm dừng trước khi chạy hoặc khi bị Pause
    AUTO_FORWARD,    ///< Chạy tiến thẳng tự động
    AUTO_STOP,       ///< Dừng tạm thời
    AUTO_BACKWARD,   ///< Lùi lại ngắn khi gặp vật cản / recovery
    AUTO_SCAN,       ///< Dừng xe, chờ cảm biến ổn định và xác nhận khoảng cách
    AUTO_TURN_LEFT,  ///< Quay trái theo góc chỉ định
    AUTO_TURN_RIGHT, ///< Quay phải theo góc chỉ định
    AUTO_RECOVER     ///< Quy trình phục hồi khi bị kẹt hoặc đường bị chặn
};

// =============================================================================
// CHIA SẺ ĐỐI TƯỢNG PHẦN CỨNG VÀ TRẠNG THÁI GIAO TIẾP
// =============================================================================
extern BTS7960 motorFL;
extern BTS7960 motorFR;
extern BTS7960 motorRL;
extern BTS7960 motorRR;
extern Motor car;

extern MPU6050Sensor mpu;
extern bool mpuOk;
extern unsigned long lastMpuUpdate;
extern const unsigned long MPU_INTERVAL;

class MovementController;
extern MovementController moveControl;

class EncoderManager;
extern EncoderManager& encoderManager;

extern ROS2BridgeManager ros2Bridge;

// Trạng thái vận hành của xe
extern OperatingMode currentMode;
extern AutoState currentAutoState;
extern String currentMoveDir;
extern int currentSpeed;
extern bool isAvoidanceActive;
extern unsigned long autoModeStartTime;
extern bool bypassSensorCheck;

// Ngưỡng khoảng cách cảnh báo
extern const float OBSTACLE_TRIGGER_CM;
extern const float OBSTACLE_CLEAR_CM;

// =============================================================================
// GIAO TIẾP CÁC PHÂN HỆ CHỨC NĂNG (APIs)
// =============================================================================

// Phân hệ điều khiển bằng tay (clien_dieukhien.cpp)
void clien_dieukhien_Init();
void clien_dieukhien_Update();
void processCommand(String cmd);
void printHelp();
void printMotorDebug();
void startMotorTest();
void updateMotorTest();

// Phân hệ chạy tự động (auto_run.cpp)
void auto_run_Init();
void auto_run_Update();
void auto_run_ResetState();
void auto_run_ProcessPiCommand(const String& cmd);
const char* auto_run_GetStateName(AutoState state);

void printStatus();

#endif // ROBOT_GLOBAL_H
