#ifndef PIN_MAP_H
#define PIN_MAP_H

/**
 * 📌 ĐỊNH NGHĨA CHÂN KẾT NỐI (PIN MAPPING)
 * Cấu hình chân GPIO vật lý của ESP32-S3 kết nối tới Driver.
 */

// --- Động cơ Front Left (FL) ---
#define MOTOR_FL_RPWM 6
#define MOTOR_FL_LPWM 7

// --- Động cơ Front Right (FR) ---
#define MOTOR_FR_RPWM 4
#define MOTOR_FR_LPWM 5

// --- Động cơ Rear Left (RL) ---
#define MOTOR_RL_RPWM 8
#define MOTOR_RL_LPWM 9

// --- Động cơ Rear Right (RR) ---
#define MOTOR_RR_RPWM 10
#define MOTOR_RR_LPWM 11

//================ HC-SR04 =================
#define HC_FRONT_TRIG 12
#define HC_FRONT_ECHO 13
#define HC_REAR_TRIG  14
#define HC_REAR_ECHO  15

//================ BUZZER ==================
#define MH_FMD_PIN    21

#endif // PIN_MAP_H
