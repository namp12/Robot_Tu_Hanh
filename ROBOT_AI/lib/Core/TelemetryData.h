#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <Arduino.h>

struct TelemetryData {
    unsigned long timestamp_ms;
    float roll, pitch, yaw;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float front_distance, rear_distance;
    int current_mode;
    int auto_state;
    int16_t motor_fl_speed;
    int16_t motor_fr_speed;
    int16_t motor_rl_speed;
    int16_t motor_rr_speed;
    uint32_t flags;
};

#endif // TELEMETRY_DATA_H
