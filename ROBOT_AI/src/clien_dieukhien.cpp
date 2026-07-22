/**
 * @file clien_dieukhien.cpp
 * @brief Phân hệ điều khiển thủ công và phân tích lệnh Serial Terminal.
 */

#include "robot_global.h"
#include "PinMap.h"
#include <unordered_map>
#include <functional>
#include "MovementController.h"

// =============================================================================
// BIẾN NỘI BỘ (INTERNAL VARIABLES)
// =============================================================================
static String inputString = "";
static bool stringComplete = false;

// Trạng thái chạy thử động cơ
static bool isTestingMotors = false;
static uint8_t testStep = 0;
static unsigned long lastTestStepTime = 0;

// Khai báo hàm checkSerial nội bộ
static void checkSerial();

// =============================================================================
// APIS IMPLEMENTATION
// =============================================================================

void clien_dieukhien_Init() {
    inputString.reserve(30);
    stringComplete = false;
    isTestingMotors = false;
}

void clien_dieukhien_Update() {
    // 1. Kiểm tra và nhận diện lệnh từ Serial
    checkSerial();
    
    // 2. Cập nhật tiến trình chạy thử động cơ nếu được kích hoạt
    updateMotorTest();
}

void printHelp() {
    Serial.println(F("\n========================================================"));
    Serial.println(F("              BẢNG LỆNH ĐIỀU KHIỂN ROBOT"));
    Serial.println(F("========================================================"));
    Serial.println(F(" 🎮 CHẾ ĐỘ THỦ CÔNG - PHÍM TẮT KÈM TỐC ĐỘ (VD: w 180, a 200, d):"));
    Serial.println(F("       [Q] Xoay Trái  -  [W] Tiến Thẳng  -  [E] Xoay Phải"));
    Serial.println(F("       [A] Đi Sang Trái - [S] Lùi Lại    -  [D] Đi Sang Phải"));
    Serial.println(F("                         [X] DỪNG XE & THOÁT AUTO"));
    Serial.println(F(" ------------------------------------------------------"));
    Serial.println(F(" ⚙️ LỆNH CHUYỂN CHẾ ĐỘ:"));
    Serial.println(F("   auto / run / 2     -> Bật Chế độ 2: Tự Động (Xe tự tránh vật cản)"));
    Serial.println(F("   manual / man / m / 1 -> Bật Chế độ 1: Thủ Công (Người dùng lái)"));
    Serial.println(F(" ------------------------------------------------------"));
    Serial.println(F(" 📝 CÁC LỆNH ĐẦY ĐỦ (Nhập dạng: <lệnh> <tốc độ 0-255>):"));
    Serial.println(F("   tien <speed>  -  lui <speed>  -  trai <speed>  -  phai <speed>"));
    Serial.println(F("   xoay_trai <speed>  -  xoay_phai <speed>  -  dung"));
    Serial.println(F(" ------------------------------------------------------"));
    Serial.println(F(" 🛠️ CÁC LỆNH HỆ THỐNG / CHẨN ĐOÁN:"));
    Serial.println(F("   status / st        -> Xem thông tin trạng thái xe (chỉ in 1 lần)"));
    Serial.println(F("   mpu                -> Xem thông số IMU MPU6050 chi tiết"));
    Serial.println(F("   reset_goc          -> Thiết lập lại góc Yaw về 0"));
    Serial.println(F("   debug              -> Xem tần số PWM và trạng thái pin"));
    Serial.println(F("   test_motor         -> Chạy tuần tự test động cơ (không block)"));
    Serial.println(F("   telemetry on/off   -> Bật/Tắt truyền telemetry nhị phân (t on/off)"));
    Serial.println(F("   bypass / bp        -> Bật/Tắt bỏ qua lỗi cảm biến siêu âm (để test AUTO)"));
    Serial.println(F("   pi <lệnh>          -> Gửi lệnh Raspberry Pi mở rộng (forward, backward, left, right, rotate_left, rotate_right, stop, recover, set_speed <v>, set_target_angle <a>)"));
    Serial.println(F("   help / h           -> In lại bảng hướng dẫn này"));
    Serial.println(F("========================================================\n"));
}

void printMotorDebug() {
    Serial.println(F("\n--- DEBUG LEDC/GPIO MOTOR ---"));
    Serial.printf("  motorFL: RPWM=GPIO%-2d ch=%-2d | LPWM=GPIO%-2d ch=%-2d\n", 
                  MOTOR_FL_RPWM, motorFL.getRpwmChannel(), MOTOR_FL_LPWM, motorFL.getLpwmChannel());
    Serial.printf("           duty RPWM=%lu | duty LPWM=%lu | freq=%.0f Hz\n",
                  ledcRead(motorFL.getRpwmChannel()), ledcRead(motorFL.getLpwmChannel()), (float)ledcReadFreq(motorFL.getRpwmChannel()));

    Serial.printf("  motorFR: RPWM=GPIO%-2d ch=%-2d | LPWM=GPIO%-2d ch=%-2d\n", 
                  MOTOR_FR_RPWM, motorFR.getRpwmChannel(), MOTOR_FR_LPWM, motorFR.getLpwmChannel());
    Serial.printf("           duty RPWM=%lu | duty LPWM=%lu | freq=%.0f Hz\n",
                  ledcRead(motorFR.getRpwmChannel()), ledcRead(motorFR.getLpwmChannel()), (float)ledcReadFreq(motorFR.getRpwmChannel()));

    Serial.printf("  motorRL: RPWM=GPIO%-2d ch=%-2d | LPWM=GPIO%-2d ch=%-2d\n", 
                  MOTOR_RL_RPWM, motorRL.getRpwmChannel(), MOTOR_RL_LPWM, motorRL.getLpwmChannel());
    Serial.printf("           duty RPWM=%lu | duty LPWM=%lu | freq=%.0f Hz\n",
                  ledcRead(motorRL.getRpwmChannel()), ledcRead(motorRL.getLpwmChannel()), (float)ledcReadFreq(motorRL.getRpwmChannel()));

    Serial.printf("  motorRR: RPWM=GPIO%-2d ch=%-2d | LPWM=GPIO%-2d ch=%-2d\n", 
                  MOTOR_RR_RPWM, motorRR.getRpwmChannel(), MOTOR_RR_LPWM, motorRR.getLpwmChannel());
    Serial.printf("           duty RPWM=%lu | duty LPWM=%lu | freq=%.0f Hz\n",
                  ledcRead(motorRR.getRpwmChannel()), ledcRead(motorRR.getLpwmChannel()), (float)ledcReadFreq(motorRR.getRpwmChannel()));

    Serial.printf("  GPIO logic level: pin%d=%d pin%d=%d pin%d=%d pin%d=%d\n",
                  MOTOR_FR_RPWM, digitalRead(MOTOR_FR_RPWM), MOTOR_FR_LPWM, digitalRead(MOTOR_FR_LPWM),
                  MOTOR_FL_RPWM, digitalRead(MOTOR_FL_RPWM), MOTOR_FL_LPWM, digitalRead(MOTOR_FL_LPWM));
    Serial.printf("  GPIO logic level: pin%d=%d pin%d=%d pin%d=%d pin%d=%d\n",
                  MOTOR_RL_RPWM, digitalRead(MOTOR_RL_RPWM), MOTOR_RL_LPWM, digitalRead(MOTOR_RL_LPWM),
                  MOTOR_RR_RPWM, digitalRead(MOTOR_RR_RPWM), MOTOR_RR_LPWM, digitalRead(MOTOR_RR_LPWM));
    Serial.println(F("----------------------------\n"));
}

void startMotorTest() {
    isTestingMotors = true;
    testStep = 0;
    lastTestStepTime = millis();
    Serial.println(F("\n--- TEST MOTOR (mỗi motor chạy 1.5s - KHÔNG BLOCK) ---"));
}

void updateMotorTest() {
    if (!isTestingMotors) return;

    unsigned long now = millis();
    unsigned long elapsed = now - lastTestStepTime;
    const uint8_t SPD = 180;

    switch (testStep) {
        case 0:
            Serial.println(F("[1/4] Motor FL tien..."));
            motorFL.forward(SPD);
            testStep = 1;
            lastTestStepTime = now;
            break;
        case 1:
            if (elapsed >= 1500) {
                motorFL.stop();
                testStep = 2;
                lastTestStepTime = now;
            }
            break;
        case 2:
            if (elapsed >= 300) {
                Serial.println(F("[2/4] Motor FR tien..."));
                motorFR.forward(SPD);
                testStep = 3;
                lastTestStepTime = now;
            }
            break;
        case 3:
            if (elapsed >= 1500) {
                motorFR.stop();
                testStep = 4;
                lastTestStepTime = now;
            }
            break;
        case 4:
            if (elapsed >= 300) {
                Serial.println(F("[3/4] Motor RL tien..."));
                motorRL.forward(SPD);
                testStep = 5;
                lastTestStepTime = now;
            }
            break;
        case 5:
            if (elapsed >= 1500) {
                motorRL.stop();
                testStep = 6;
                lastTestStepTime = now;
            }
            break;
        case 6:
            if (elapsed >= 300) {
                Serial.println(F("[4/4] Motor RR tien..."));
                motorRR.forward(SPD);
                testStep = 7;
                lastTestStepTime = now;
            }
            break;
        case 7:
            if (elapsed >= 1500) {
                motorRR.stop();
                testStep = 8;
                lastTestStepTime = now;
            }
            break;
        case 8:
            if (elapsed >= 300) {
                Serial.println(F("  Hoan tat test motor!"));
                isTestingMotors = false;
            }
            break;
    }
}

// =============================================================================
// CÁC HÀM NỘI BỘ (INTERNAL HELPERS)
// =============================================================================

static void checkSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (inputString.length() > 0) stringComplete = true;
        } else {
            inputString += c;
        }
    }

    if (stringComplete) {
        processCommand(inputString);
        inputString = "";
        stringComplete = false;
    }
}

struct ManualCommand {
    String actionName;
    std::function<void(int, const String&)> handler;
};

static std::unordered_map<std::string, ManualCommand> commandRegistry;

static void initCommandRegistry() {
    if (!commandRegistry.empty()) return;

    commandRegistry["tien"] = {"Forward", [](int speed, const String&) {
        if (currentMode == MODE_AUTO) {
            Serial.println(F("   [Lỗi] Đang ở chế độ AUTO, hãy chuyển sang chế độ MANUAL trước."));
            return;
        }
        currentMoveDir = "tien";
        currentSpeed = speed;
        moveControl.forward(speed);
        Serial.printf("   TIEN | speed=%d\n", speed);
    }};

    commandRegistry["lui"] = {"Backward", [](int speed, const String&) {
        if (currentMode == MODE_AUTO) {
            Serial.println(F("   [Lỗi] Đang ở chế độ AUTO, hãy chuyển sang chế độ MANUAL trước."));
            return;
        }
        currentMoveDir = "lui";
        currentSpeed = speed;
        moveControl.backward(speed);
        Serial.printf("   LUI | speed=%d\n", speed);
    }};

    commandRegistry["trai"] = {"Strafe Left", [](int speed, const String&) {
        if (currentMode == MODE_AUTO) {
            Serial.println(F("   [Lỗi] Đang ở chế độ AUTO, hãy chuyển sang chế độ MANUAL trước."));
            return;
        }
        currentMoveDir = "trai";
        currentSpeed = speed;
        moveControl.strafeLeft(speed);
        Serial.printf("   TRAI | speed=%d\n", speed);
    }};

    commandRegistry["phai"] = {"Strafe Right", [](int speed, const String&) {
        if (currentMode == MODE_AUTO) {
            Serial.println(F("   [Lỗi] Đang ở chế độ AUTO, hãy chuyển sang chế độ MANUAL trước."));
            return;
        }
        currentMoveDir = "phai";
        currentSpeed = speed;
        moveControl.strafeRight(speed);
        Serial.printf("   PHAI | speed=%d\n", speed);
    }};

    commandRegistry["xoay_trai"] = {"Rotate Left", [](int speed, const String&) {
        if (currentMode == MODE_AUTO) {
            Serial.println(F("   [Lỗi] Đang ở chế độ AUTO, hãy chuyển sang chế độ MANUAL trước."));
            return;
        }
        currentMoveDir = "xoay_trai";
        currentSpeed = speed;
        moveControl.rotateLeft(speed);
        Serial.printf("   XOAY_TRAI | speed=%d\n", speed);
    }};

    commandRegistry["xoay_phai"] = {"Rotate Right", [](int speed, const String&) {
        if (currentMode == MODE_AUTO) {
            Serial.println(F("   [Lỗi] Đang ở chế độ AUTO, hãy chuyển sang chế độ MANUAL trước."));
            return;
        }
        currentMoveDir = "xoay_phai";
        currentSpeed = speed;
        moveControl.rotateRight(speed);
        Serial.printf("   XOAY_PHAI | speed=%d\n", speed);
    }};

    commandRegistry["cheo_tt"] = {"Diagonal Front Left", [](int speed, const String&) {
        if (currentMode == MODE_AUTO) {
            Serial.println(F("   [Lỗi] Đang ở chế độ AUTO, hãy chuyển sang chế độ MANUAL trước."));
            return;
        }
        currentMoveDir = "cheo_tt";
        currentSpeed = speed;
        moveControl.diagonalFrontLeft(speed);
        Serial.printf("   CHEO_TT | speed=%d\n", speed);
    }};

    commandRegistry["cheo_tp"] = {"Diagonal Front Right", [](int speed, const String&) {
        if (currentMode == MODE_AUTO) {
            Serial.println(F("   [Lỗi] Đang ở chế độ AUTO, hãy chuyển sang chế độ MANUAL trước."));
            return;
        }
        currentMoveDir = "cheo_tp";
        currentSpeed = speed;
        moveControl.diagonalFrontRight(speed);
        Serial.printf("   CHEO_TP | speed=%d\n", speed);
    }};

    commandRegistry["cheo_st"] = {"Diagonal Back Left", [](int speed, const String&) {
        if (currentMode == MODE_AUTO) {
            Serial.println(F("   [Lỗi] Đang ở chế độ AUTO, hãy chuyển sang chế độ MANUAL trước."));
            return;
        }
        currentMoveDir = "cheo_st";
        currentSpeed = speed;
        moveControl.diagonalBackLeft(speed);
        Serial.printf("   CHEO_ST | speed=%d\n", speed);
    }};

    commandRegistry["cheo_sp"] = {"Diagonal Back Right", [](int speed, const String&) {
        if (currentMode == MODE_AUTO) {
            Serial.println(F("   [Lỗi] Đang ở chế độ AUTO, hãy chuyển sang chế độ MANUAL trước."));
            return;
        }
        currentMoveDir = "cheo_sp";
        currentSpeed = speed;
        moveControl.diagonalBackRight(speed);
        Serial.printf("   CHEO_SP | speed=%d\n", speed);
    }};

    commandRegistry["dung"] = {"Stop", [](int, const String&) {
        currentMoveDir = "dung";
        currentSpeed = 0;
        isAvoidanceActive = false;
        moveControl.stop();
        if (currentMode == MODE_AUTO) {
            currentMode = MODE_MANUAL;
            Serial.println(F("   [AUTO] Đã dừng xe và tự động thoát về chế độ MANUAL."));
        }
        Serial.println(F("   DỪNG XE"));
    }};

    commandRegistry["mode_manual"] = {"Set Mode Manual", [](int, const String&) {
        currentMode = MODE_MANUAL;
        isAvoidanceActive = false;
        currentMoveDir = "dung";
        currentSpeed = 0;
        moveControl.stop();
        Serial.println(F("   [System] Đã chuyển sang chế độ MANUAL (THỦ CÔNG). Đã dừng xe."));
    }};

    commandRegistry["mode_auto"] = {"Set Mode Auto", [](int, const String&) {
        currentMode = MODE_AUTO;
        isAvoidanceActive = false;
        autoModeStartTime = millis();
        auto_run_ResetState();
        Serial.println(F("   [System] Đã chuyển sang chế độ AUTO. Đang đồng bộ cảm biến trong 1.5s..."));
    }};

    commandRegistry["mode_ros2"] = {"Set Mode ROS2", [](int, const String&) {
        currentMode = MODE_ROS2;
        isAvoidanceActive = false;
        currentMoveDir = "dung";
        currentSpeed = 0;
        moveControl.stop();
        Serial.println(F("   [System] Đã chuyển sang chế độ ROS2 MODE (MÁY TÍNH LÁI)."));
    }};

    commandRegistry["pi"] = {"Raspberry Pi Command Interface", [](int, const String& rawParam) {
        auto_run_ProcessPiCommand(rawParam);
    }};

    commandRegistry["mpu"] = {"Print IMU Data", [](int, const String&) {
        if (!mpuOk) {
            Serial.println(F("   MPU6050 chưa khởi tạo! Kiểm tra kết nối I2C."));
        } else {
            Serial.println(F("--- DỮ LIỆU CẢM BIẾN IMU MPU6050 ---"));
            Serial.printf("  Roll : %.2f deg\n", mpu.getRoll());
            Serial.printf("  Pitch: %.2f deg\n", mpu.getPitch());
            Serial.printf("  Yaw  : %.2f deg\n", mpu.getYaw());
            Serial.printf("  Ax=%.3f  Ay=%.3f  Az=%.3f (m/s2)\n",
                          mpu.getAccelX(), mpu.getAccelY(), mpu.getAccelZ());
            Serial.printf("  Gx=%.3f  Gy=%.3f  Gz=%.3f (rad/s)\n",
                          mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ());
            Serial.printf("  Nhiệt độ: %.2f C\n", mpu.getTemperature());
            Serial.println(F("---------------------------"));
        }
    }};

    commandRegistry["reset_goc"] = {"Reset Yaw Angle", [](int, const String&) {
        mpu.resetAngle();
        Serial.println(F("   Đã reset góc Roll/Pitch/Yaw về 0"));
    }};

    commandRegistry["debug"] = {"Print Motor Debug", [](int, const String&) {
        printMotorDebug();
    }};

    commandRegistry["test_motor"] = {"Run Motor Test Sequence", [](int, const String&) {
        startMotorTest();
    }};

    commandRegistry["print_status"] = {"Print Status Report", [](int, const String&) {
        printStatus();
    }};

    commandRegistry["toggle_bypass"] = {"Toggle Bypass Sensor", [](int, const String&) {
        bypassSensorCheck = !bypassSensorCheck;
        Serial.printf("   [System] Tự động bỏ qua lỗi cảm biến (Bypass Sensor Check): %s\n",
                      bypassSensorCheck ? "ĐANG BẬT (ON)" : "ĐANG TẮT (OFF)");
    }};

    commandRegistry["telemetry"] = {"Toggle Telemetry Stream", [](int, const String& param) {
        bool enable = true;
        if (param.equalsIgnoreCase("off") || param.equalsIgnoreCase("0") || param == "disable" || param == "f") {
            enable = false;
        }
        ros2Bridge.setTelemetryEnabled(enable);
        Serial.printf("   [Telemetry] Binary telemetry streaming is now: %s\n",
                      enable ? "ENABLED (ON)" : "DISABLED (OFF)");
    }};

    commandRegistry["help"] = {"Print Help Menu", [](int, const String&) {
        printHelp();
    }};
}

static std::string getNormalizedAction(const std::string& rawAction) {
    static std::unordered_map<std::string, std::string> aliasMap = {
        {"w", "tien"}, {"s", "lui"}, {"a", "trai"}, {"d", "phai"},
        {"q", "xoay_trai"}, {"e", "xoay_phai"}, {"x", "dung"},
        {"h", "help"},
        {"m", "mode_manual"}, {"man", "mode_manual"}, {"manual", "mode_manual"}, {"1", "mode_manual"},
        {"run", "mode_auto"}, {"auto", "mode_auto"}, {"2", "mode_auto"},
        {"ros2", "mode_ros2"}, {"mode_ros2", "mode_ros2"}, {"4", "mode_ros2"},
        {"st", "print_status"}, {"status", "print_status"},
        {"bp", "toggle_bypass"}, {"bypass", "toggle_bypass"},
        {"t", "telemetry"}, {"telemetry", "telemetry"}
    };
    auto it = aliasMap.find(rawAction);
    if (it != aliasMap.end()) {
        return it->second;
    }
    return rawAction;
}

void processCommand(String cmd) {
    cmd.trim();
    String origCmd = cmd;
    cmd.toLowerCase();

    int spaceIndex = cmd.indexOf(' ');
    String action = (spaceIndex == -1) ? cmd : cmd.substring(0, spaceIndex);
    int speed = (spaceIndex == -1)
                    ? 150 
                    : constrain(cmd.substring(spaceIndex + 1).toInt(), 0, 255);

    std::string actionStr = std::string(action.c_str());
    std::string normalizedAction = getNormalizedAction(actionStr);
    initCommandRegistry();

    auto it = commandRegistry.find(normalizedAction);
    if (it != commandRegistry.end()) {
        // Output Serial Log in required format
        Serial.println(F("=========================="));
        Serial.print(F("RX: "));
        Serial.println(origCmd);
        Serial.print(F("ACTION: "));
        Serial.println(it->second.actionName);
        Serial.println(F("=========================="));

        String rawParam = "";
        if (spaceIndex != -1) {
            rawParam = origCmd.substring(spaceIndex + 1);
            rawParam.trim();
        }
        it->second.handler(speed, rawParam);
    } else {
        Serial.println(F("   ERR: Lệnh không hợp lệ. Gõ 'help' để xem bảng lệnh."));
    }
}
