#ifndef MPU6050SENSOR_H
#define MPU6050SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

class MPU6050Sensor
{
private:
    Adafruit_MPU6050 mpu;

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    float accelX;
    float accelY;
    float accelZ;

    float gyroX;
    float gyroY;
    float gyroZ;
    float gyroZOffset;

    float temperature;

    float roll;
    float pitch;
    float yaw;

    unsigned long previousTime;
    bool _ready;   // true sau khi begin() thành công

public:

    MPU6050Sensor();

    bool begin(uint8_t sda = 18, uint8_t scl = 19);

    void update();

    void calibrate();

    void resetAngle();

    bool isConnected();

    float getAccelX();
    float getAccelY();
    float getAccelZ();

    float getGyroX();
    float getGyroY();
    float getGyroZ();

    float getTemperature();

    float getRoll();
    float getPitch();
    float getYaw();
};

#endif