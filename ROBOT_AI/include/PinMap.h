#ifndef PIN_MAP_H
#define PIN_MAP_H

/**
 * 📌 ĐỊNH NGHĨA CHÂN KẾT NỐI (PIN MAPPING)
 * Cấu hình chân GPIO vật lý của ESP32-S3 kết nối tới Driver.
 */

// --- Động cơ Front Left (FL) ---
#define MOTOR_FL_RPWM 7
#define MOTOR_FL_LPWM 6

// --- Động cơ Front Right (FR) ---
#define MOTOR_FR_RPWM 5
#define MOTOR_FR_LPWM 4

// --- Động cơ Rear Left (RL) ---
#define MOTOR_RL_RPWM 9
#define MOTOR_RL_LPWM 8

// --- Động cơ Rear Right (RR) ---
#define MOTOR_RR_RPWM 11
#define MOTOR_RR_LPWM 10

//================ HC-SR04 =================
#define HC_FRONT_TRIG 16
#define HC_FRONT_ECHO 17
#define HC_REAR_TRIG  14
#define HC_REAR_ECHO  15

//================ ENCODER =================
#define ENC_FL_A      41
#define ENC_FL_B      42
#define ENC_FR_A      39
#define ENC_FR_B      40
#define ENC_RL_A      37
#define ENC_RL_B      38
#define ENC_RR_A      35
#define ENC_RR_B      36

//================ BUZZER & BATTERY =========
#define MH_FMD_PIN      2
#define BATTERY_ADC_PIN 1

#endif // PIN_MAP_H
