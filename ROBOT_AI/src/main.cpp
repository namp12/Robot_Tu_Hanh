/**
 * 🤖 ROBOT MECANUM - HỆ THỐNG ĐIỀU KHIỂN ĐỘNG CƠ
 * Điều khiển hướng và tốc độ qua Serial Terminal (COM5 / USB)
 *
 * === BẢNG LỆNH ĐIỀU KHIỂN ===
 * tien <speed>      -> Tiến thẳng        (VD: tien 180)
 * lui <speed>       -> Lùi thẳng         (VD: lui 150)
 * trai <speed>      -> Dịch ngang Trái   (VD: trai 200)
 * phai <speed>      -> Dịch ngang Phải   (VD: phai 200)
 * xoay_trai <speed> -> Xoay Trái tại chỗ (VD: xoay_trai 120)
 * xoay_phai <speed> -> Xoay Phải tại chỗ (VD: xoay_phai 120)
 * cheo_tt <speed>   -> Chéo Trước-Trái   (VD: cheo_tt 150)
 * cheo_tp <speed>   -> Chéo Trước-Phải   (VD: cheo_tp 150)
 * cheo_st <speed>   -> Chéo Sau-Trái     (VD: cheo_st 150)
 * cheo_sp <speed>   -> Chéo Sau-Phải     (VD: cheo_sp 150)
 * dung              -> Dừng tất cả động cơ
 * mpu               -> Hiển thị dữ liệu IMU (góc, gia tốc, con quay)
 * reset_goc         -> Reset góc Roll/Pitch/Yaw về 0
 * debug             -> In trạng thái LEDC/GPIO của tất cả motor
 * test_motor        -> Test từng motor FL/FR/RL/RR liên tiếp
 * help              -> Hiển thị bảng lệnh
 * =============================================
 * Tốc độ hợp lệ: 0 - 255
 */

#include <Arduino.h>
#include "BTS7960.h"
#include "Motor.h"
#include "Mpu6050.h"

// -------------------------------------------
// Cấu hình 4 Driver BTS7960 (RPWM, LPWM)
// EN/R-EN/L-EN nối thẳng VCC → không cần khai báo
// -------------------------------------------
// Core 2.x: channel tự động gán theo thứ tự tạo object:
//   motorFL: ch0(RPWM=6),  ch1(LPWM=7)
//   motorFR: ch2(RPWM=4),  ch3(LPWM=5)
//   motorRL: ch4(RPWM=8),  ch5(LPWM=9)
//   motorRR: ch6(RPWM=10), ch7(LPWM=11)
BTS7960 motorFL(6, 7);
BTS7960 motorFR(4, 5);
BTS7960 motorRL(8, 9);
BTS7960 motorRR(10, 11);

Motor car(motorFL, motorFR, motorRL, motorRR);

// ---- MPU6050 ----
MPU6050Sensor mpu;
bool mpuOk = false;
unsigned long lastMpuUpdate = 0;
const unsigned long MPU_INTERVAL = 20; // cập nhật MPU mỗi 20ms (~50Hz)

// Biến đệm nhận lệnh Serial
String inputString    = "";
bool   stringComplete = false;

// =============================================
// In bảng lệnh ra Serial0
// =============================================
void printHelp() {
  Serial0.println(F("\n====================================="));
  Serial0.println(F("      BẢNG LỆNH ĐIỀU KHIỂN ROBOT"));
  Serial0.println(F("====================================="));
  Serial0.println(F("  tien <0-255>       -> Tiến thẳng"));
  Serial0.println(F("  lui <0-255>        -> Lùi thẳng"));
  Serial0.println(F("  trai <0-255>       -> Dịch ngang Trái"));
  Serial0.println(F("  phai <0-255>       -> Dịch ngang Phải"));
  Serial0.println(F("  xoay_trai <0-255>  -> Xoay Trái tại chỗ"));
  Serial0.println(F("  xoay_phai <0-255>  -> Xoay Phải tại chỗ"));
  Serial0.println(F("  cheo_tt <0-255>    -> Chéo Trước-Trái"));
  Serial0.println(F("  cheo_tp <0-255>    -> Chéo Trước-Phải"));
  Serial0.println(F("  cheo_st <0-255>    -> Chéo Sau-Trái"));
  Serial0.println(F("  cheo_sp <0-255>    -> Chéo Sau-Phải"));
  Serial0.println(F("  dung               -> DỪNG"));
  Serial0.println(F("  mpu                -> Xem dữ liệu IMU"));
  Serial0.println(F("  reset_goc          -> Reset goc Roll/Pitch/Yaw"));
  Serial0.println(F("  debug              -> Debug LEDC/GPIO motor"));
  Serial0.println(F("  test_motor         -> Test từng motor"));
  Serial0.println(F("  help               -> Bảng lệnh này"));
  Serial0.println(F("=====================================\n"));
}

// =============================================
// DEBUG: In trạng thái LEDC của tất cả motor
// Dùng ledcRead (Core 2.x) để đọc duty đang set
// =============================================
void printMotorDebug() {
  Serial0.println(F("\n--- DEBUG LEDC/GPIO MOTOR ---"));
  Serial0.println(F("  Core 2.x: channel = (object_index * 2)"));
  Serial0.printf("  motorFL: RPWM=GPIO%-2d ch=0  | LPWM=GPIO%-2d ch=1\n", 6, 7);
  Serial0.printf("           duty RPWM=%lu | duty LPWM=%lu\n",
                 ledcRead(0), ledcRead(1));
  Serial0.printf("           freq ch0=%.0f Hz | freq ch1=%.0f Hz\n",
                 (float)ledcReadFreq(0), (float)ledcReadFreq(1));

  Serial0.printf("  motorFR: RPWM=GPIO%-2d ch=2  | LPWM=GPIO%-2d ch=3\n", 4, 5);
  Serial0.printf("           duty RPWM=%lu | duty LPWM=%lu\n",
                 ledcRead(2), ledcRead(3));
  Serial0.printf("           freq ch2=%.0f Hz | freq ch3=%.0f Hz\n",
                 (float)ledcReadFreq(2), (float)ledcReadFreq(3));

  Serial0.printf("  motorRL: RPWM=GPIO%-2d ch=4  | LPWM=GPIO%-2d ch=5\n", 8, 9);
  Serial0.printf("           duty RPWM=%lu | duty LPWM=%lu\n",
                 ledcRead(4), ledcRead(5));
  Serial0.printf("           freq ch4=%.0f Hz | freq ch5=%.0f Hz\n",
                 (float)ledcReadFreq(4), (float)ledcReadFreq(5));

  Serial0.printf("  motorRR: RPWM=GPIO%-2d ch=6  | LPWM=GPIO%-2d ch=7\n", 10, 11);
  Serial0.printf("           duty RPWM=%lu | duty LPWM=%lu\n",
                 ledcRead(6), ledcRead(7));
  Serial0.printf("           freq ch6=%.0f Hz | freq ch7=%.0f Hz\n",
                 (float)ledcReadFreq(6), (float)ledcReadFreq(7));

  Serial0.printf("  motorFL enabled=%d | motorFR enabled=%d\n",
                 motorFL.isEnabled(), motorFR.isEnabled());
  Serial0.printf("  motorRL enabled=%d | motorRR enabled=%d\n",
                 motorRL.isEnabled(), motorRR.isEnabled());

  // Đọc mode GPIO hiện tại (INPUT=1, OUTPUT=2, INPUT_PULLUP=5)
  // gpio_get_level chỉ đọc logic level
  Serial0.printf("  GPIO logic level: pin4=%d pin5=%d pin6=%d pin7=%d\n",
                 digitalRead(4), digitalRead(5), digitalRead(6), digitalRead(7));
  Serial0.printf("  GPIO logic level: pin8=%d pin9=%d pin10=%d pin11=%d\n",
                 digitalRead(8), digitalRead(9), digitalRead(10), digitalRead(11));
  Serial0.println(F("----------------------------\n"));
}

// =============================================
// TEST MOTOR: chạy từng motor 1.5 giây
// =============================================
void testMotor() {
  Serial0.println(F("\n--- TEST MOTOR (moi motor 1.5s) ---"));
  const uint8_t SPD = 180;

  Serial0.println(F("[1/4] Motor FL tien..."));
  motorFL.forward(SPD);
  delay(1500);
  motorFL.stop();
  delay(300);

  Serial0.println(F("[2/4] Motor FR tien..."));
  motorFR.forward(SPD);
  delay(1500);
  motorFR.stop();
  delay(300);

  Serial0.println(F("[3/4] Motor RL tien..."));
  motorRL.forward(SPD);
  delay(1500);
  motorRL.stop();
  delay(300);

  Serial0.println(F("[4/4] Motor RR tien..."));
  motorRR.forward(SPD);
  delay(1500);
  motorRR.stop();
  delay(300);

  Serial0.println(F("  Test xong!"));
}

// =============================================
// Xử lý một lệnh đã nhận
// =============================================
void processCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  Serial0.print(F(">> Nhan lenh: "));
  Serial0.println(cmd);

  int    spaceIndex = cmd.indexOf(' ');
  String action     = (spaceIndex == -1) ? cmd : cmd.substring(0, spaceIndex);
  int    speed      = (spaceIndex == -1)
                        ? 150
                        : constrain(cmd.substring(spaceIndex + 1).toInt(), 0, 255);

  if (action == "tien") {
    car.forward(speed);
    Serial0.printf("   TIEN | speed=%d | FL+%d FR+%d RL+%d RR+%d\n",
                   speed, speed, speed, speed, speed);

  } else if (action == "lui") {
    car.backward(speed);
    Serial0.printf("   LUI | speed=%d | FL-%d FR-%d RL-%d RR-%d\n",
                   speed, speed, speed, speed, speed);

  } else if (action == "trai") {
    car.strafeLeft(speed);
    Serial0.printf("   TRAI | speed=%d\n", speed);

  } else if (action == "phai") {
    car.strafeRight(speed);
    Serial0.printf("   PHAI | speed=%d\n", speed);

  } else if (action == "xoay_trai") {
    car.rotateLeft(speed);
    Serial0.printf("   XOAY_TRAI | speed=%d\n", speed);

  } else if (action == "xoay_phai") {
    car.rotateRight(speed);
    Serial0.printf("   XOAY_PHAI | speed=%d\n", speed);

  } else if (action == "cheo_tt") {
    car.diagonalFrontLeft(speed);
    Serial0.printf("   CHEO_TT | speed=%d\n", speed);

  } else if (action == "cheo_tp") {
    car.diagonalFrontRight(speed);
    Serial0.printf("   CHEO_TP | speed=%d\n", speed);

  } else if (action == "cheo_st") {
    car.diagonalBackLeft(speed);
    Serial0.printf("   CHEO_ST | speed=%d\n", speed);

  } else if (action == "cheo_sp") {
    car.diagonalBackRight(speed);
    Serial0.printf("   CHEO_SP | speed=%d\n", speed);

  } else if (action == "dung") {
    car.stop();
    Serial0.println(F("   DUNG"));

  } else if (action == "mpu") {
    if (!mpuOk) {
      Serial0.println(F("   MPU6050 chua init! Kiem tra ket noi."));
    } else {
      Serial0.println(F("--- DU LIEU IMU MPU6050 ---"));
      Serial0.printf("  Roll : %.2f deg\n", mpu.getRoll());
      Serial0.printf("  Pitch: %.2f deg\n", mpu.getPitch());
      Serial0.printf("  Yaw  : %.2f deg\n", mpu.getYaw());
      Serial0.printf("  Ax=%.3f  Ay=%.3f  Az=%.3f (m/s2)\n",
                     mpu.getAccelX(), mpu.getAccelY(), mpu.getAccelZ());
      Serial0.printf("  Gx=%.3f  Gy=%.3f  Gz=%.3f (rad/s)\n",
                     mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ());
      Serial0.printf("  Nhiet do: %.2f C\n", mpu.getTemperature());
      Serial0.println(F("---------------------------"));
    }

  } else if (action == "reset_goc") {
    mpu.resetAngle();
    Serial0.println(F("   Da reset goc Roll/Pitch/Yaw ve 0"));
  } else if (action == "debug") {
    printMotorDebug();

  } else if (action == "test_motor") {
    testMotor();

  } else if (action == "help") {
    printHelp();

  } else {
    Serial0.println(F("   ERR: Lenh khong hop le. Go 'help' de xem bang lenh."));
  }
}

// =============================================
// Đọc từ cổng Serial0 (UART0 / COM5)
// =============================================
void checkSerial() {
  while (Serial0.available()) {
    char c = (char)Serial0.read();
    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) stringComplete = true;
    } else {
      inputString += c;
    }
  }

  if (stringComplete) {
    processCommand(inputString);
    inputString    = "";
    stringComplete = false;
  }
}

// =============================================
// SETUP
// =============================================
void setup() {
  // Khởi Serial0 = UART0 phần cứng (TX=GPIO43, RX=GPIO44)
  Serial0.begin(115200);
  delay(1500);

  Serial0.println(F("=== KHOI DONG HE THONG ==="));
#if defined(ESP_ARDUINO_VERSION_MAJOR)
  Serial0.printf("Arduino ESP32 Core: %d.%d.%d\n",
                 ESP_ARDUINO_VERSION_MAJOR,
                 ESP_ARDUINO_VERSION_MINOR,
                 ESP_ARDUINO_VERSION_PATCH);
#else
  Serial0.println(F("Arduino ESP32 Core: (khong xac dinh duoc phien ban)"));
#endif

  // ==========================================================
  // BƯỚC 1: Khởi tạo MPU6050 (Wire.begin) TRƯỚC LEDC
  // ----------------------------------------------------------
  // LÝ DO: Adafruit_I2CDevice::begin() gọi _wire->begin()
  // (không tham số → SDA=GPIO8, SCL=GPIO9 là default trên S3).
  // Nếu motorRL đã dùng GPIO8/9 làm LEDC, Wire sẽ tranh chấp.
  //
  // Giải pháp: Gọi Wire.begin(18,19) TRƯỚC → i2cIsInit()=true
  // → Wire.begin() ngầm của BusIO bị bỏ qua, GPIO8/9 an toàn.
  // ==========================================================
  Serial0.println(F("[1/5] Khoi tao MPU6050 (SDA=18, SCL=19)..."));
  mpuOk = mpu.begin(18, 19);
  if (mpuOk) {
    Serial0.println(F("  MPU6050 OK"));
  } else {
    Serial0.println(F("  MPU6050 KHONG TIM THAY! Motor van hoat dong binh thuong."));
  }

  // ==========================================================
  // BƯỚC 2: Khởi tạo LEDC Motor SAU Wire.begin()
  // ==========================================================
  Serial0.println(F("[2/5] Khoi tao motorFL (RPWM=6 ch=0, LPWM=7 ch=1)..."));
  motorFL.begin();
  Serial0.printf("   freq ch0=%.0f Hz, freq ch1=%.0f Hz\n",
                 (float)ledcReadFreq(0), (float)ledcReadFreq(1));

  Serial0.println(F("[3/5] Khoi tao motorFR (RPWM=4 ch=2, LPWM=5 ch=3)..."));
  motorFR.begin();
  Serial0.printf("   freq ch2=%.0f Hz, freq ch3=%.0f Hz\n",
                 (float)ledcReadFreq(2), (float)ledcReadFreq(3));

  Serial0.println(F("[4/5] Khoi tao motorRL (RPWM=8 ch=4, LPWM=9 ch=5)..."));
  motorRL.begin();
  Serial0.printf("   freq ch4=%.0f Hz, freq ch5=%.0f Hz\n",
                 (float)ledcReadFreq(4), (float)ledcReadFreq(5));

  Serial0.println(F("[5/5] Khoi tao motorRR (RPWM=10 ch=6, LPWM=11 ch=7)..."));
  motorRR.begin();
  Serial0.printf("   freq ch6=%.0f Hz, freq ch7=%.0f Hz\n",
                 (float)ledcReadFreq(6), (float)ledcReadFreq(7));

  // Kiểm tra nhanh: nếu freq = 0 → ledcSetup thất bại
  bool ledcOk = (ledcReadFreq(0) > 0) && (ledcReadFreq(2) > 0) &&
                (ledcReadFreq(4) > 0) && (ledcReadFreq(6) > 0);
  if (ledcOk) {
    Serial0.println(F("  LEDC OK - tat ca channel co tan so hop le"));
  } else {
    Serial0.println(F("  [!!!] LEDC LOI - mot so channel co tan so = 0!"));
    Serial0.println(F("        Goi lenh 'debug' de xem chi tiet."));
  }

  Serial0.println(F("=== HE THONG SAN SANG ==="));
  printHelp();
}

// =============================================
// LOOP
// =============================================
void loop() {
  checkSerial();

  // Cập nhật MPU6050 định kỳ (non-blocking)
  if (mpuOk) {
    unsigned long now = millis();
    if (now - lastMpuUpdate >= MPU_INTERVAL) {
      mpu.update();
      lastMpuUpdate = now;
    }
  }
}