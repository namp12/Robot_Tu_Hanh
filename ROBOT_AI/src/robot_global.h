/**
 * @file robot_global.h
 * @brief Khai báo các đối tượng phần cứng và trạng thái hoạt động dùng chung.
 */

#ifndef ROBOT_GLOBAL_H
#define ROBOT_GLOBAL_H

#include <Arduino.h>
#include "BTS7960.h"
#include "Motor.h"
#include "Mpu6050.h"
#include "Sensor_HC_SR04.h"
#include "MH_FMD.h"

// =============================================================================
// ĐỊNH NGHĨA CHẾ ĐỘ HOẠT ĐỘNG
// =============================================================================
enum OperatingMode {
    MODE_MANUAL, ///< Chế độ điều khiển bằng tay (Thủ công)
    MODE_AUTO    ///< Chế độ chạy tự động tránh vật cản
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

// Trạng thái vận hành của xe
extern OperatingMode currentMode;
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

void printStatus();

#endif // ROBOT_GLOBAL_H
