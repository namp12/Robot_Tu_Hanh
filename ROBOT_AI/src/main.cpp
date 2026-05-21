/**
 * 🤖 HỆ THỐNG ROBOT TỰ HÀNH ĐA NHIỆM (AUTONOMOUS MULTI-PURPOSE ROBOT)
 * 🧠 ESP32-S3 Low-Level Firmware (Entrypoint chính)
 * 
 * Sơ đồ cấu trúc phần mềm dạng hướng đối tượng (OOP):
 * - PinMap.h / Config.h: Cấu hình phần cứng và hằng số toàn cục.
 * - MotorDriver (lib): Quản lý driver BTS7960 băm xung PWM.
 * - EncoderReader (lib): Quản lý ngắt đọc encoder và tính RPM/vận tốc.
 * - Kinematics (lib): Thực hiện động học Mecanum ngược.
 */

#include <Arduino.h>
#include "PinMap.h"
#include "Config.h"
#include "MotorDriver.h"
#include "EncoderReader.h"
#include "Kinematics.h"

// Tạo các đối tượng điều khiển
MotorDriver motor;
EncoderReader encoder;
Kinematics kinematics;

// Biến lưu trạng thái Serial Command
String inputString = "";
bool stringComplete = false;

// Khai báo các chương trình con điều khiển di chuyển nhanh
void tien(uint8_t speed) {
    WheelSpeeds ws = kinematics.getWheelSpeeds(speed, 0, 0);
    motor.setSpeed(ws.fl, ws.fr, ws.rl, ws.rr);
}

void lui(uint8_t speed) {
    WheelSpeeds ws = kinematics.getWheelSpeeds(-speed, 0, 0);
    motor.setSpeed(ws.fl, ws.fr, ws.rl, ws.rr);
}

void sangTrai(uint8_t speed) {
    WheelSpeeds ws = kinematics.getWheelSpeeds(0, -speed, 0);
    motor.setSpeed(ws.fl, ws.fr, ws.rl, ws.rr);
}

void sangPhai(uint8_t speed) {
    WheelSpeeds ws = kinematics.getWheelSpeeds(0, speed, 0);
    motor.setSpeed(ws.fl, ws.fr, ws.rl, ws.rr);
}

void xoayTaiCho(int16_t speed) {
    WheelSpeeds ws = kinematics.getWheelSpeeds(0, 0, speed);
    motor.setSpeed(ws.fl, ws.fr, ws.rl, ws.rr);
}

void dungYen() {
    motor.stop();
}

/**
 * Đọc dữ liệu từ Serial (UART)
 */
void checkSerial() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n' || inChar == '\r') {
            if (inputString.length() > 0) {
                stringComplete = true;
            }
        } else {
            inputString += inChar;
        }
    }

    if (stringComplete) {
        inputString.trim();
        inputString.toLowerCase();

        Serial.print("-> Recv: ");
        Serial.println(inputString);

        if (inputString.startsWith("tien ")) {
            int val = inputString.substring(5).toInt();
            tien(val);
            Serial.printf("Action: FORWARD | Speed: %d\n", val);
        } 
        else if (inputString.startsWith("lui ")) {
            int val = inputString.substring(4).toInt();
            lui(val);
            Serial.printf("Action: BACKWARD | Speed: %d\n", val);
        } 
        else if (inputString.startsWith("trai ")) {
            int val = inputString.substring(5).toInt();
            sangTrai(val);
            Serial.printf("Action: STRAFE LEFT | Speed: %d\n", val);
        } 
        else if (inputString.startsWith("phai ")) {
            int val = inputString.substring(5).toInt();
            sangPhai(val);
            Serial.printf("Action: STRAFE RIGHT | Speed: %d\n", val);
        } 
        else if (inputString.startsWith("xoay ")) {
            int val = inputString.substring(5).toInt();
            xoayTaiCho(val);
            Serial.printf("Action: ROTATE | Speed: %d\n", val);
        } 
        else if (inputString == "dung") {
            dungYen();
            Serial.println("Action: STOPPED");
        } 
        else if (inputString.startsWith("move ")) {
            // Cú pháp: move <vx> <vy> <omega> (ví dụ: move 150 0 50)
            int firstSpace = inputString.indexOf(' ', 5);
            int secondSpace = inputString.indexOf(' ', firstSpace + 1);
            
            if (firstSpace != -1 && secondSpace != -1) {
                float vx = inputString.substring(5, firstSpace).toFloat();
                float vy = inputString.substring(firstSpace + 1, secondSpace).toFloat();
                float omega = inputString.substring(secondSpace + 1).toFloat();
                
                WheelSpeeds ws = kinematics.getWheelSpeeds(vx, vy, omega);
                motor.setSpeed(ws.fl, ws.fr, ws.rl, ws.rr);
                Serial.printf("Action: KINEMATICS | vx: %.1f, vy: %.1f, w: %.1f\n", vx, vy, omega);
            } else {
                Serial.println("ERR: Use: move <vx> <vy> <omega>");
            }
        } 
        else if (inputString == "status") {
            long fl_t, fr_t, rl_t, rr_t;
            float fl_r, fr_r, rl_r, rr_r;
            float fl_s, fr_s, rl_s, rr_s;

            encoder.getTicks(fl_t, fr_t, rl_t, rr_t);
            encoder.getRPM(fl_r, fr_r, rl_r, rr_r);
            encoder.getSpeeds(fl_s, fr_s, rl_s, rr_s);

            Serial.println("====== SYSTEM STATUS ======");
            Serial.printf("Ticks - FL: %ld | FR: %ld | RL: %ld | RR: %ld\n", fl_t, fr_t, rl_t, rr_t);
            Serial.printf("RPM   - FL: %.1f | FR: %.1f | RL: %.1f | RR: %.1f\n", fl_r, fr_r, rl_r, rr_r);
            Serial.printf("Speed - FL: %.3f m/s | FR: %.3f m/s | RL: %.3f m/s | RR: %.3f m/s\n", fl_s, fr_s, rl_s, rr_s);
            Serial.println("===========================");
        } 
        else if (inputString == "reset") {
            encoder.resetTicks();
            Serial.println("Encoder ticks reset.");
        }
        else {
            Serial.println("ERR: Unknown Command.");
        }

        inputString = "";
        stringComplete = false;
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(1000);
    Serial.println("=== SYSTEM INITIALIZING ===");

    // Khởi tạo các linh kiện/phân hệ
    motor.begin();
    encoder.begin();

    Serial.println("=== SYSTEM READY ===");
}

void loop() {
    // Đọc và tính toán vận tốc định kỳ
    encoder.update();

    // Xử lý dữ liệu điều khiển từ cổng Serial
    checkSerial();
}