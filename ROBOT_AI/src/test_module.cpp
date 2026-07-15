/**
 * @file test_module.cpp
 * @brief Implementations cho phân hệ chẩn đoán và kiểm tra các module.
 */

#include "test_module.h"
#include "robot_global.h"

// Cấu hình chế độ chạy chính và kiểm tra chẩn đoán module
enum MainMode {
    MAIN_MODE_MANUAL,
    MAIN_MODE_AUTO,
    MAIN_MODE_TEST
};
static MainMode currentMainMode = MAIN_MODE_MANUAL;

enum TestModule {
    TEST_NONE,
    TEST_SENSOR_HC_SR04,
    TEST_MOTOR,
    TEST_MPU6050,
    TEST_BUZZER
};
static TestModule activeTestModule = TEST_NONE;

static bool runSensorUpdate = true;           // Cho phép bật/tắt cập nhật cảm biến siêu âm tránh ngập lụt log
static bool waitingForMainMenuChoice = false;  // Biến trạng thái để xử lý menu chính

// =============================================================================
// CÁC HÀM IN MENU TRỢ GIÚP NỘI BỘ
// =============================================================================

static void printModuleTestMenu() {
    Serial.println(F("\n============================================================="));
    Serial.println(F("🛠️ CHẾ ĐỘ 3: CHẨN ĐOÁN & KIỂM TRA CÁC MODULE"));
    Serial.println(F("============================================================="));
    Serial.println(F("Vui lòng chọn module để kiểm tra (gõ số 1-5 rồi nhấn Enter):"));
    Serial.println(F("  [1] Cảm biến siêu âm HC-SR04"));
    Serial.println(F("  [2] Động cơ & Di chuyển (BTS7960 / Mecanum Car)"));
    Serial.println(F("  [3] Cảm biến góc nghiêng IMU MPU6050"));
    Serial.println(F("  [4] Còi cảnh báo MH-FMD (Active Buzzer)"));
    Serial.println(F("  [5] Quay lại Menu chính (Chọn Chế độ 1 / Chế độ 2)"));
    Serial.println(F("=============================================================\n"));
}

static void printSensorTestHelp() {
    Serial.println(F("\n============================================================="));
    Serial.println(F("🔍 CẤU HÌNH KIỂM TRA CẢM BIẾN SIÊU ÂM HC-SR04"));
    Serial.println(F("============================================================="));
    Serial.println(F(" Gõ các lệnh chẩn đoán sau rồi nhấn Enter:"));
    Serial.println(F("   0             -> Cấu hình quét đo luân phiên cả 2 cảm biến"));
    Serial.println(F("   1             -> Chỉ đo cảm biến TRƯỚC (Front)"));
    Serial.println(F("   2             -> Chỉ đo cảm biến SAU (Rear)"));
    Serial.println(F("   pause         -> Tạm dừng cập nhật (tránh trôi log khi bị lỗi Echo LOW)"));
    Serial.println(F("   resume        -> Tiếp tục cập nhật khoảng cách cảm biến"));
    Serial.println(F("   print         -> Gọi HC_SR04_Print() in báo cáo chi tiết"));
    Serial.println(F("   gpio          -> Gọi HC_SR04_TestGPIO() xem mức logic tĩnh các chân"));
    Serial.println(F("   trigger       -> Gọi HC_SR04_TestTrigger() tạo xung kích hoạt thủ công"));
    Serial.println(F("   conflict      -> Gọi HC_SR04_CheckConflicts() kiểm tra xung đột chân"));
    Serial.println(F("   warn <số cm>  -> Đặt khoảng cách cảnh báo cảm biến (VD: warn 45)"));
    Serial.println(F("   back          -> Quay lại Menu chọn Module"));
    Serial.println(F("=============================================================\n"));
}

static void printMotorTestHelp() {
    Serial.println(F("\n============================================================="));
    Serial.println(F("⚙️ CẤU HÌNH KIỂM TRA ĐỘNG CƠ (BTS7960 / MECANUM CAR)"));
    Serial.println(F("============================================================="));
    Serial.println(F(" Gõ các lệnh kiểm tra sau rồi nhấn Enter:"));
    Serial.println(F("   test_motor    -> Chạy quy trình quay thử lần lượt 4 bánh xe"));
    Serial.println(F("   tien <tốc độ> -> Cho xe tiến thẳng (tốc độ 0-255, mặc định 150)"));
    Serial.println(F("   lui <tốc độ>  -> Cho xe lùi lại"));
    Serial.println(F("   trai <tốc độ> -> Cho xe đi sang trái (Strafe Left)"));
    Serial.println(F("   phai <tốc độ> -> Cho xe đi sang phải (Strafe Right)"));
    Serial.println(F("   xoay_trai <tốc độ> -> Xoay trái tại chỗ"));
    Serial.println(F("   xoay_phai <tốc độ> -> Xoay phải tại chỗ"));
    Serial.println(F("   dung          -> Dừng xe ngay lập tức"));
    Serial.println(F("   debug         -> In chi tiết tần số/chu kỳ PWM và logic chân động cơ"));
    Serial.println(F("   back          -> Quay lại Menu chọn Module"));
    Serial.println(F("=============================================================\n"));
}

static void printMpuTestHelp() {
    Serial.println(F("\n============================================================="));
    Serial.println(F("🧭 CẤU HÌNH KIỂM TRA CẢM BIẾN GÓC IMU MPU6050"));
    Serial.println(F("============================================================="));
    Serial.println(F(" Gõ các lệnh kiểm tra sau rồi nhấn Enter:"));
    Serial.println(F("   mpu           -> In chi tiết góc Roll/Pitch/Yaw và gia tốc/gyro hiện tại"));
    Serial.println(F("   reset_goc     -> Hiệu chuẩn lại góc hiện tại về 0"));
    Serial.println(F("   back          -> Quay lại Menu chọn Module"));
    Serial.println(F("=============================================================\n"));
}

static void printBuzzerTestHelp() {
    Serial.println(F("\n============================================================="));
    Serial.println(F("🔔 CẤU HÌNH KIỂM TRA CÒI CẢNH BÁO MH-FMD"));
    Serial.println(F("============================================================="));
    Serial.println(F(" Gõ các lệnh kiểm tra sau rồi nhấn Enter:"));
    Serial.println(F("   on            -> Bật còi kêu liên tục (kêu to)"));
    Serial.println(F("   off           -> Tắt còi hoàn toàn"));
    Serial.println(F("   beep <ms>     -> Bíp còi ngắn theo thời gian (ms), mặc định 300ms"));
    Serial.println(F("   test          -> Chạy thử chu trình âm báo (chậm -> nhanh -> khẩn cấp)"));
    Serial.println(F("   threshold <cm>-> Cài đặt khoảng cách kích hoạt còi kêu tự động"));
    Serial.println(F("   status        -> Xem trạng thái còi và khoảng cách hiện tại"));
    Serial.println(F("   back          -> Quay lại Menu chọn Module"));
    Serial.println(F("=============================================================\n"));
}

static void promptMainMenuSelection() {
    Serial.println(F("\n======================================================="));
    Serial.println(F("Vui lòng chọn Chế độ hoạt động chính:"));
    Serial.println(F("  [1] Chế độ 1: Người dùng điều khiển bằng tay (Manual Mode)"));
    Serial.println(F("  [2] Chế độ 2: Xe chạy tự động tránh vật cản (Auto Mode)"));
    Serial.println(F("======================================================="));
    waitingForMainMenuChoice = true;
}

// =============================================================================
// IMPLEMENTATION CÁC HÀM API GIAO TIẾP
// =============================================================================

bool is_in_test_mode() {
    return (currentMainMode == MAIN_MODE_TEST);
}

bool should_run_sensor_update() {
    // Nếu không ở chế độ Test, luôn cập nhật cảm biến siêu âm (nếu không bị pause)
    if (currentMainMode != MAIN_MODE_TEST) {
        return runSensorUpdate;
    }
    // Nếu ở chế độ Test, chỉ cập nhật khi đang chọn test module cảm biến siêu âm và không bị pause
    return (activeTestModule == TEST_SENSOR_HC_SR04 && runSensorUpdate);
}

void test_module_Init() {
    Serial.println(F("Vui lòng chọn chế độ hoạt động ban đầu bằng cách gõ '1', '2' hoặc '3' rồi nhấn Enter:"));
    Serial.println(F("  [1] Chế độ 1: Người dùng điều khiển bằng tay (Manual Mode)"));
    Serial.println(F("  [2] Chế độ 2: Xe chạy tự động tránh vật cản (Auto Mode)"));
    Serial.println(F("  [3] Chế độ 3: Chế độ kiểm tra chẩn đoán các Module (Module Test Mode)"));
    Serial.println(F("* Đang chờ lựa chọn (Mặc định tự chọn [1] sau 10 giây nếu không nhập)..."));
    Serial.println(F("-------------------------------------------------------"));

    unsigned long boot_wait_start = millis();
    bool modeSelected = false;

    while (millis() - boot_wait_start < 10000) {
        if (Serial.available()) {
            String choice = Serial.readStringUntil('\n');
            choice.trim();
            if (choice == "1") {
                currentMainMode = MAIN_MODE_MANUAL;
                currentMode = MODE_MANUAL;
                modeSelected = true;
                Serial.println(F("\n📢 [System] Đã chọn Chế độ 1: ĐIỀU KHIỂN BẰNG TAY (MANUAL)"));
                break;
            } else if (choice == "2") {
                currentMainMode = MAIN_MODE_AUTO;
                currentMode = MODE_AUTO;
                modeSelected = true;
                Serial.println(F("\n📢 [System] Đã chọn Chế độ 2: CHẠY TỰ ĐỘNG TRÁNH VẬT CẢN (AUTO)"));
                break;
            } else if (choice == "3") {
                currentMainMode = MAIN_MODE_TEST;
                currentMode = MODE_MANUAL; // Tạm thời đưa về MANUAL
                activeTestModule = TEST_NONE;
                modeSelected = true;
                Serial.println(F("\n📢 [System] Đã chọn Chế độ 3: CHẨN ĐOÁN & KIỂM TRA CÁC MODULE"));
                break;
            }
        }
        delay(10);
    }

    if (!modeSelected) {
        currentMainMode = MAIN_MODE_MANUAL;
        currentMode = MODE_MANUAL;
        Serial.println(F("\n📢 [System] Quá thời gian chờ! Mặc định chọn Chế độ 1: ĐIỀU KHIỂN BẰNG TAY (MANUAL)."));
    }

    if (currentMainMode == MAIN_MODE_TEST) {
        printModuleTestMenu();
    } else {
        printHelp();
    }
}

// Xử lý lệnh
static void processMainCommand(String cmd);

static String mainInputString = "";
void test_module_Update() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (mainInputString.length() > 0) {
                mainInputString.trim();
                processMainCommand(mainInputString);
                mainInputString = "";
            }
        } else {
            mainInputString += c;
        }
    }
}

static void processMainCommand(String cmd) {
    String origCmd = cmd;
    cmd.trim();
    cmd.toLowerCase();

    // 1. Phân tích tham số của lệnh
    int spaceIndex = cmd.indexOf(' ');
    String action = (spaceIndex == -1) ? cmd : cmd.substring(0, spaceIndex);
    String param = (spaceIndex == -1) ? "" : cmd.substring(spaceIndex + 1);

    // Chuyển đổi nhanh chế độ chính bất cứ lúc nào nếu gõ đúng từ khóa toàn cục
    if (action == "manual" || action == "man") {
        currentMainMode = MAIN_MODE_MANUAL;
        currentMode = MODE_MANUAL;
        activeTestModule = TEST_NONE;
        waitingForMainMenuChoice = false;
        HC_SR04_SetDebug(false);
        car.stop();
        Serial.println(F("\n📢 [System] Đã chuyển sang Chế độ 1: ĐIỀU KHIỂN BẰNG TAY (MANUAL)"));
        printHelp();
        return;
    }
    if (action == "auto" || action == "run") {
        currentMainMode = MAIN_MODE_AUTO;
        currentMode = MODE_AUTO;
        activeTestModule = TEST_NONE;
        waitingForMainMenuChoice = false;
        HC_SR04_SetDebug(false);
        autoModeStartTime = millis();
        car.stop();
        Serial.println(F("\n📢 [System] Đã chuyển sang Chế độ 2: CHẠY TỰ ĐỘNG TRÁNH VẬT CẢN (AUTO)"));
        printHelp();
        return;
    }
    if (action == "diagnose" || action == "sensor" || action == "test_module") {
        currentMainMode = MAIN_MODE_TEST;
        currentMode = MODE_MANUAL;
        activeTestModule = TEST_NONE;
        waitingForMainMenuChoice = false;
        HC_SR04_SetDebug(false);
        car.stop();
        Serial.println(F("\n📢 [System] Đã chuyển sang Chế độ 3: CHẨN ĐOÁN & KIỂM TRA CÁC MODULE"));
        printModuleTestMenu();
        return;
    }

    // 2. Nếu đang chờ chọn chế độ chính (khi quay lại từ Menu test)
    if (waitingForMainMenuChoice) {
        if (action == "1" || action == "manual" || action == "man" || action == "m") {
            currentMainMode = MAIN_MODE_MANUAL;
            currentMode = MODE_MANUAL;
            waitingForMainMenuChoice = false;
            Serial.println(F("\n📢 [System] Đã chuyển sang Chế độ 1: ĐIỀU KHIỂN BẰNG TAY (MANUAL)"));
            printHelp();
        } else if (action == "2" || action == "auto" || action == "run") {
            currentMainMode = MAIN_MODE_AUTO;
            currentMode = MODE_AUTO;
            waitingForMainMenuChoice = false;
            autoModeStartTime = millis();
            Serial.println(F("\n📢 [System] Đã chuyển sang Chế độ 2: CHẠY TỰ ĐỘNG TRÁNH VẬT CẢN (AUTO)"));
            printHelp();
        } else {
            Serial.println(F("❌ Vui lòng nhập '1' (chế độ Manual) hoặc '2' (chế độ Auto) để tiếp tục:"));
        }
        return;
    }

    // 3. Nếu ở chế độ TEST MODULES (MAIN_MODE_TEST)
    if (currentMainMode == MAIN_MODE_TEST) {
        // Lệnh quay lại menu chọn module hoặc Menu chính
        if (action == "back") {
            if (activeTestModule != TEST_NONE) {
                if (activeTestModule == TEST_SENSOR_HC_SR04) {
                    HC_SR04_SetDebug(false); // Tắt debug log khi thoát bài test cảm biến
                }
                Serial.println(F("\nQuay lại Menu chọn Module..."));
                activeTestModule = TEST_NONE;
                printModuleTestMenu();
            } else {
                promptMainMenuSelection();
            }
            return;
        }

        // Lệnh trợ giúp
        if (action == "h" || action == "help") {
            if (activeTestModule == TEST_NONE) {
                printModuleTestMenu();
            } else if (activeTestModule == TEST_SENSOR_HC_SR04) {
                printSensorTestHelp();
            } else if (activeTestModule == TEST_MOTOR) {
                printMotorTestHelp();
            } else if (activeTestModule == TEST_MPU6050) {
                printMpuTestHelp();
            } else if (activeTestModule == TEST_BUZZER) {
                printBuzzerTestHelp();
            }
            return;
        }

        // Khi chưa chọn module nào (TEST_NONE)
        if (activeTestModule == TEST_NONE) {
            if (action == "1") {
                activeTestModule = TEST_SENSOR_HC_SR04;
                HC_SR04_SetDebug(true); // Bật in debug chi tiết của readSensor khi test cảm biến
                Serial.println(F("\n📢 [Module Test] Đang kiểm tra CẢM BIẾN SIÊU ÂM HC-SR04"));
                printSensorTestHelp();
            } else if (action == "2") {
                activeTestModule = TEST_MOTOR;
                Serial.println(F("\n📢 [Module Test] Đang kiểm tra ĐỘNG CƠ / DI CHUYỂN"));
                printMotorTestHelp();
            } else if (action == "3") {
                activeTestModule = TEST_MPU6050;
                Serial.println(F("\n📢 [Module Test] Đang kiểm tra IMU MPU6050"));
                printMpuTestHelp();
            } else if (action == "4") {
                activeTestModule = TEST_BUZZER;
                Serial.println(F("\n📢 [Module Test] Đang kiểm tra CÒI CẢNH BÁO MH-FMD"));
                printBuzzerTestHelp();
            } else if (action == "5") {
                promptMainMenuSelection();
            } else {
                Serial.println(F("❌ Lựa chọn không hợp lệ! Vui lòng nhập từ 1 đến 5."));
            }
            return;
        }

        // --- SUBMENU: Test Cảm biến HC-SR04 ---
        if (activeTestModule == TEST_SENSOR_HC_SR04) {
            if (action == "0" || action == "alt") {
                HC_SR04_TestMode(0);
            } else if (action == "1" || action == "front") {
                HC_SR04_TestMode(1);
            } else if (action == "2" || action == "rear") {
                HC_SR04_TestMode(2);
            } else if (action == "pause") {
                runSensorUpdate = false;
                HC_SR04_SetDebug(false); // Tạm dừng thì tắt luôn debug log
                Serial.println(F("⏸️ [Sensor] Đã TẠM DỪNG tự động cập nhật cảm biến (Ngưng ngập lụt Log)"));
            } else if (action == "resume") {
                runSensorUpdate = true;
                HC_SR04_SetDebug(true); // Tiếp tục thì bật lại debug log
                Serial.println(F("▶️ [Sensor] Đã TIẾP TỤC tự động cập nhật cảm biến"));
            } else if (action == "print") {
                HC_SR04_Print();
            } else if (action == "gpio") {
                HC_SR04_TestGPIO();
            } else if (action == "trigger") {
                HC_SR04_TestTrigger();
            } else if (action == "conflict") {
                HC_SR04_CheckConflicts();
            } else if (action == "warn") {
                if (param.length() > 0) {
                    float val = param.toFloat();
                    if (val > 0.0f) {
                        HC_SR04_SetWarningDistance(val);
                        Serial.printf("⚙️ [Sensor] Đã cấu hình ngưỡng cảnh báo cảm biến: %.1f cm\n", val);
                    } else {
                        Serial.println(F("❌ Lỗi: Giá trị không hợp lệ."));
                    }
                } else {
                    Serial.printf("ℹ️ Ngưỡng cảnh báo hiện tại: %.1f cm\n", HC_SR04_GetWarningDistance());
                }
            } else {
                Serial.println(F("❌ Lệnh không hợp lệ trong Test Sensor. Gõ 'help' hoặc 'back' để quay lại."));
            }
            return;
        }

        // --- SUBMENU: Test Động cơ ---
        if (activeTestModule == TEST_MOTOR) {
            if (action == "tien" || action == "lui" || action == "trai" || action == "phai" || 
                action == "xoay_trai" || action == "xoay_phai" || action == "dung" || 
                action == "debug" || action == "test_motor") {
                processCommand(origCmd);
            } else {
                Serial.println(F("❌ Lệnh không hợp lệ trong Test Motor. Gõ 'help' hoặc 'back' để quay lại."));
            }
            return;
        }

        // --- SUBMENU: Test MPU6050 ---
        if (activeTestModule == TEST_MPU6050) {
            if (action == "mpu" || action == "reset_goc") {
                processCommand(origCmd);
            } else {
                Serial.println(F("❌ Lệnh không hợp lệ trong Test IMU. Gõ 'help' hoặc 'back' để quay lại."));
            }
            return;
        }

        // --- SUBMENU: Test Buzzer ---
        if (activeTestModule == TEST_BUZZER) {
            if (action == "on") {
                MH_FMD_On();
                Serial.println(F("🔊 [Buzzer] Đã BẬT còi kêu liên tục."));
            } else if (action == "off") {
                MH_FMD_Off();
                Serial.println(F("🔇 [Buzzer] Đã TẮT còi."));
            } else if (action == "beep") {
                int duration = (param.length() > 0) ? param.toInt() : 300;
                MH_FMD_Beep(duration);
                Serial.printf("🔊 [Buzzer] Beep %d ms...\n", duration);
            } else if (action == "test") {
                MH_FMD_Test();
                Serial.println(F("🔊 [Buzzer] Đang chạy chu trình tự động test..."));
            } else if (action == "threshold") {
                if (param.length() > 0) {
                    float val = param.toFloat();
                    if (val > 0.0f) {
                        MH_FMD_SetThreshold(val);
                        Serial.printf("⚙️ [Buzzer] Đã đặt ngưỡng cảnh báo còi: %.1f cm\n", val);
                    } else {
                        Serial.println(F("❌ Lỗi: Giá trị không hợp lệ."));
                    }
                } else {
                    Serial.printf("ℹ️ Ngưỡng cảnh báo còi hiện tại: %.1f cm\n", MH_FMD_GetThreshold());
                }
            } else if (action == "status") {
                MH_FMD_PrintStatus();
            } else {
                Serial.println(F("❌ Lệnh không hợp lệ trong Test Buzzer. Gõ 'help' hoặc 'back' để quay lại."));
            }
            return;
        }
    }

    // 4. Nếu ở chế độ VẬN HÀNH CHÍNH (Manual hoặc Auto)
    else {
        // Hỗ trợ đổi nhanh chế độ bằng các phím số 1, 2, 3
        if (action == "3") {
            currentMainMode = MAIN_MODE_TEST;
            currentMode = MODE_MANUAL;
            activeTestModule = TEST_NONE;
            car.stop();
            Serial.println(F("\n📢 [System] Đã chuyển sang Chế độ 3: CHẨN ĐOÁN & KIỂM TRA CÁC MODULE"));
            printModuleTestMenu();
            return;
        }
        if (action == "1") {
            currentMainMode = MAIN_MODE_MANUAL;
            currentMode = MODE_MANUAL;
            isAvoidanceActive = false;
            car.stop();
            Serial.println(F("\n📢 [System] Đã chuyển sang Chế độ 1: ĐIỀU KHIỂN BẰNG TAY (MANUAL)"));
            printHelp();
            return;
        }
        if (action == "2") {
            currentMainMode = MAIN_MODE_AUTO;
            currentMode = MODE_AUTO;
            isAvoidanceActive = false;
            autoModeStartTime = millis();
            Serial.println(F("\n📢 [System] Đã chuyển sang Chế độ 2: CHẠY TỰ ĐỘNG TRÁNH VẬT CẢN (AUTO)"));
            printHelp();
            return;
        }

        // Chuyển lệnh còn lại đến clien_dieukhien.cpp
        processCommand(origCmd);
    }
}

// =============================================================================
// IN TRẠNG THÁI CHI TIẾT THEO YÊU CẦU (ON DEMAND STATUS)
// =============================================================================
void printStatus() {
    Serial.println(F("--------------------------------------------------------------------------------"));
    Serial.print(F("[Mode] "));
    Serial.print(currentMode == MODE_AUTO ? F("AUTO (TỰ ĐỘNG TRÁNH VẬT CẢN)") : F("MANUAL (THỦ CÔNG)"));
    Serial.print(F(" | Active Dir: "));
    Serial.print(currentMoveDir);
    Serial.print(F(" | Speed: "));
    Serial.print(currentSpeed);
    Serial.print(F(" | Avoidance Status: "));
    Serial.println(isAvoidanceActive ? F("RUNNING") : F("IDLE"));
    
    if (mpuOk) {
        Serial.printf("[MPU6050] Angle Roll: %.1f deg | Pitch: %.1f deg | Yaw: %.1f deg\n", 
                      mpu.getRoll(), mpu.getPitch(), mpu.getYaw());
    }
    
    HC_SR04_Print();
    MH_FMD_PrintStatus();
    Serial.println(F("--------------------------------------------------------------------------------"));
}
