#include "TelemetryManager.h"
#include "RobotStateManager.h"
#include "EncoderManager.h"
#include "IMUManager.h"
#include "UltrasonicManager.h"
#include "Config.h"

TelemetryManager::TelemetryManager() {
    memset(&_telemetry, 0, sizeof(UnifiedTelemetry));
}

void TelemetryManager::update() {
    _telemetry.timestamp = millis();
    _telemetry.robotState = RobotStateManager::getInstance().getStateName();

#if ENCODER_ENABLED
    EncoderManager& enc = EncoderManager::getInstance();
    for (int i = 0; i < 4; i++) {
        _telemetry.encoderPulse[i] = enc.getPulse(i);
        _telemetry.encoderRPM[i] = enc.getRPM(i);
    }
    _telemetry.odomX = enc.getWheelDistance(); // Approximate forward displacement
    _telemetry.vx = enc.getLinearVelocity();
    _telemetry.wz = enc.getAngularVelocity();
#endif

    IMUManager& imu = IMUManager::getInstance();
    _telemetry.yaw = imu.getYaw();
    _telemetry.pitch = imu.getPitch();
    _telemetry.roll = imu.getRoll();
    Vector3 gyro = imu.getGyro();
    _telemetry.gyroX = gyro.x;
    _telemetry.gyroY = gyro.y;
    _telemetry.gyroZ = gyro.z;
    Vector3 accel = imu.getAccel();
    _telemetry.accelX = accel.x;
    _telemetry.accelY = accel.y;
    _telemetry.accelZ = accel.z;

    UltrasonicManager& us = UltrasonicManager::getInstance();
    _telemetry.frontDistance = us.getFrontDistance();
    _telemetry.rearDistance = us.getRearDistance();

    _telemetry.batteryVoltage = 12.6f; // Simulated / ADC voltage sensor
}
