# BÁO CÁO PHÂN TÍCH KIẾN TRÚC & KẾ HOẠCH TÍCH HỢP ROS2 CHO FIRMWARE ESP32
**Vai trò:** Tech Lead (Robotics, ROS2 Humble, Raspberry Pi 4, ESP32, PlatformIO)  
**Dự án:** Robot Tự Hành Mecanum (Robot AI)  
**Ngày lập kế hoạch:** 21/07/2026  

---

## 1. DỰ ÁN HIỆN TẠI ĐÃ ĐỦ TỐT CHƯA? CÓ CẦN REFACTOR KHÔNG?

**"Giữ nguyên."**

### Phân tích chi tiết:
- Project hiện tại được cấu trúc theo chuẩn PlatformIO rất xuất sắc: tách biệt rõ ràng giữa tầng phần cứng Driver (`lib/`), tầng ứng dụng (`src/`), tầng cấu hình (`include/`).
- Các module đã thiết kế theo mô hình **Non-blocking Event-Driven** dùng `millis()`, hoàn toàn không dùng `delay()` ngắt dòng execution trong `loop()`.
- Việc tổ chức lớp `Motor` làm Coordinator và `Kinematics` xử lý toán học độc lập cho phép tích hợp giao tiếp ROS2 một cách tự nhiên mà không cần phải thay đổi cấu trúc nền móng.

---

## 2. NHỮNG MODULE KHÔNG ĐƯỢC SỬA & LÝ DO

| Thư mục / Module | Lý do KHÔNG ĐƯỢC SỬA |
| :--- | :--- |
| `lib/BTS7960` | Driver điều khiển PWM H-Bridge công suất thấp nhất. Đã hoạt động ổn định và tương thích cả ESP32 Core 2.x & 3.x. Sửa đổi dễ gây chập PWM hoặc mất điều khiển motor. |
| `lib/Motor` | Lớp Coordinator điều khiển hướng chạy Mecanum 4 bánh. Đã cung cấp đầy đủ các API di chuyển (`setAllMotor`, `forward`, `strafeLeft`, `rotateLeft`). |
| `lib/Kinematics` | Công thức toán học động học ngược Mecanum đã chính xác. Chuyển đổi $(v_x, v_y, \omega) \to \text{PWM } 4 \text{ bánh}$. |
| `lib/EncoderReader` | Đọc ngắt phần cứng ISR sườn lên `RISING` của 4 kênh đĩa encoder. Đã có đầy đủ logic tính RPM, vận tốc $m/s$, tích lũy xung `getTicks()`. |
| `lib/Mpu6050` | Module giao tiếp I2C tùy chỉnh chân SDA/SCL đọc gia tốc, gyro, góc nghiêng Euler MPU6050. |
| `lib/Sensor_HC_SR04` | Phân hệ đọc siêu âm luân phiên 60ms non-blocking + bộ lọc Median 5 mẫu + tự động phát hiện hỏng chân/ngắt kết nối. |
| `lib/MH_FMD` | Phân hệ còi cảnh báo Active Buzzer (Active Low). |
| `src/auto_run.cpp` | Phân hệ chạy tự động né vật cản độc lập ở cấp Firmware (Standalone Auto Mode). |
| `src/clien_dieukhien.cpp` | Phân hệ giao diện Terminal CLI hỗ trợ kỹ sư điều khiển và debug trực tiếp từ Serial Monitor. |
| `src/test_module.cpp` | Phân hệ chẩn đoán module phần cứng độc lập. |

---

## 3. NHỮNG MODULE NÊN MỞ RỘNG & LÝ DO

1. **`include/Config.h`**:
   - *Nội dung mở rộng*: Thêm cấu hình Baudrate truyền Serial dành cho ROS2 (ví dụ: `SERIAL_ROS2_BAUD 115200` hoặc `921600`), thêm hằng số Watchdog Timeout (`ROS2_WATCHDOG_MS 500`).
   - *Lý do*: Đảm bảo khả năng cấu hình tập trung các thông số giao tiếp cao cấp.
2. **`src/robot_global.h`**:
   - *Nội dung mở rộng*: Thêm `MODE_ROS2` vào enum `OperatingMode`, khai báo `extern` cho đối tượng `EncoderReader encoder;` và đối tượng `ROS2BridgeManager ros2Bridge;`.
   - *Lý do*: Chia sẻ trạng thái hoạt động ROS2 và đối tượng Encoder toàn cục cho toàn hệ thống.
3. **`src/main.cpp`**:
   - *Nội dung mở rộng*: Khởi tạo instance `EncoderReader encoder;`, khởi tạo `ros2Bridge`, gọi `encoder.begin()` và `ros2Bridge.begin()` trong `setup()`, và chèn điểm gọi `encoder.update()` cùng `ros2Bridge.update()` trong `loop()`.
   - *Lý do*: `main.cpp` là tầng Orchestrator kết nối các subsystem lại với nhau.

---

## 4. PHƯƠNG ÁN THÊM GIAO TIẾP SERIAL VỚI RASPBERRY PI

### **NÊN THÊM MODULE MỚI.**

- **Tên module mới**: `lib/ROS2Protocol` (bao gồm `ROS2Protocol.h`, `PacketBuilder`, `PacketParser`, và `ROS2BridgeManager`).
- **Lý do**:
  - **Tách biệt vai trò (Separation of Concerns)**: Giao tiếp CLI Terminal ở `clien_dieukhien.cpp` hướng tới **Người dùng (Human)** nhập ký tự chữ ASCII (`w, a, s, d`). Trong khi đó, giao tiếp với Raspberry Pi / ROS2 hướng tới **Máy tính / Thuật toán (Machine)** truyền gói tin nhị phân Binary Frame hoặc micro-ROS tốc độ cao.
  - **Giữ an toàn cho hệ thống hiện tại**: Việc thêm module mới giúp giữ nguyên 100% tính năng CLI debug hiện tại, cho phép chuyển đổi qua lại giữa mode CLI và mode ROS2 mà không gây xung đột.

---

## 5. THIẾT KẾ CÁC CLASS TRONG MODULE MỚI (`lib/ROS2Protocol`)

*(CHỈ THIẾT KẾ KIẾN TRÚC - KHÔNG CODE)*

```text
lib/ROS2Protocol/
├── ROS2Protocol.h        # Định nghĩa Cấu trúc Frame Nhị phân, Struct Telemetry, Struct Command, Msg ID
├── PacketBuilder.h/.cpp  # Đóng gói dữ liệu Telemetry thành Mảng Byte Nhị phân + CRC16
├── PacketParser.h/.cpp   # Máy trạng thái giải mã dòng Byte Serial từ RPi + Kiểm tra CRC16
└── ROS2BridgeManager.h/.cpp # Manager quản lý luồng gửi/nhận, xử lý Watchdog ngắt khẩn cấp
```

### Chi tiết nhiệm vụ từng Lớp (Class):

1. **`struct ROS2TelemetryData` & `struct ROS2CommandData` (`ROS2Protocol.h`)**:
   - Định nghĩa chính xác cấu trúc dữ liệu nhị phân truyền qua lại giữa ESP32 và Raspberry Pi:
     - Header (`0xFF 0xFE`), Message ID, Length, Payload, CRC16 Checksum, Tail (`0xFD`).
2. **`Class PacketBuilder`**:
   - Cung cấp hàm `buildTelemetryPacket(...)`: Đóng gói dữ liệu số xung 4 Encoder, Vận tốc 4 bánh, IMU Roll/Pitch/Yaw, Gia tốc, Gyro, Siêu âm, Điện áp pin, Trạng thái hệ thống thành mảng byte chuẩn bị xuất ra `Serial.write()`.
   - Tính toán và gán CRC16 vào cuối gói tin.
3. **`Class PacketParser`**:
   - Triển khai **State Machine nhận byte (BYTE_BY_BYTE_PARSER)**:
     - `STATE_WAIT_HEADER1` -> `STATE_WAIT_HEADER2` -> `STATE_WAIT_MSG_ID` -> `STATE_WAIT_LEN` -> `STATE_READ_PAYLOAD` -> `STATE_CHECK_CRC`.
   - Đảm bảo loại bỏ hoàn toàn nhiễu tín hiệu đường truyền Serial và trích xuất chính xác lệnh vận tốc `cmd_vel` ($v_x, v_y, \omega$).
4. **`Class ROS2BridgeManager`**:
   - Sở hữu đối tượng `PacketBuilder` và `PacketParser`.
   - **Tích hợp Hardware Safety Watchdog**: Nếu ở `MODE_ROS2` mà không nhận được gói tin từ Raspberry Pi quá 500ms, tự động ngắt PWM toàn bộ 4 bánh xe để tránh xe "chạy điên" khi RPi bị crash hoặc đứt cáp USB.

---

## 6. THIẾT KẾ LUỒNG TRONG `loop()`

### Sơ đồ luồng xử lý trong `loop()`:

```text
               ┌────────────────────────┐
               │         loop()         │
               └───────────┬────────────┘
                           │
  ┌────────────────────────┼────────────────────────┐
  ▼                        ▼                        ▼
[1. ReadIMU]         [2. ReadSensors]         [3. ReadEncoder]
 (mpu.update)        (HC_SR04_Update)         (encoder.update)
  │                        │                        │
  └────────────────────────┼────────────────────────┘
                           │
                           ▼
                 [4. ReceiveCommand]
             (ros2Bridge.updateFromSerial)
                           │
                           ▼
                    [5. CheckWatchdog]
        (Kiểm tra timeout Serial ngắt khẩn nếu mất kết nối)
                           │
                           ▼
                    [6. RunRobot]
             ┌─────────────┴─────────────┐
             ▼                           ▼
     currentMode == MODE_ROS2?    currentMode != MODE_ROS2?
     ├── TRUE: Lấy vx, vy, w       └── FALSE: Giữ nguyên logic cũ
     │   từ RPi Command               (Manual CLI / Auto Run)
     │   -> Kinematics
     │   -> Motor.setAllMotor()
             │                           │
             └─────────────┬─────────────┘
                           │
                           ▼
                    [7. SendStatus]
          (ros2Bridge.sendTelemetry 20ms/50ms)
```

### Đánh giá ảnh hưởng đến logic cũ:
- **KHÔNG ẢNH HƯỞNG ĐẾN LOGIC CŨ.**
- Hệ thống chỉ bổ sung thêm một chế độ hoạt động mới là `MODE_ROS2`.
- Khi `currentMode == MODE_MANUAL` hoặc `MODE_AUTO`, toàn bộ luồng cũ chạy chính xác 100% như hiện tại.
- Khi người dùng hoặc RPi gửi lệnh bật `MODE_ROS2`, xe mới chuyển sang nhận lệnh lái từ ROS2.

---

## 7. DỮ LIỆU GIAO TIẾP ESP32 ↔ RASPBERRY PI

### A. Dữ liệu ESP32 gửi lên Raspberry Pi (Telemetry / Status)

| Tên dữ liệu | Mô tả chi tiết | Mức độ ưu tiên | Vai trò trong ROS2 |
| :--- | :--- | :--- | :--- |
| **Encoder Ticks** | Ticks tích lũy 4 bánh (`FL`, `FR`, `RL`, `RR`) | **BẮT BUỘC** | ROS2 Node tính Wheel Odometry (`nav_msgs/msg/Odometry`) & Joint States |
| **Encoder Speeds** | Vận tốc $m/s$ hoặc RPM 4 bánh | **BẮT BUỘC** | Kiểm tra phản hồi vận tốc thực tế bánh xe |
| **IMU Data** | Accel (X,Y,Z), Gyro (X,Y,Z), Roll, Pitch, Yaw | **BẮT BUỘC** | ROS2 EKF Node hợp nhất Odom + IMU (`robot_localization`) |
| **Current Mode & Status** | Trạng thái mode hiện tại, E-Stop flag | **BẮT BUỘC** | Giám sát an toàn hệ thống |
| **Ultrasonic Distance** | Khoảng cách siêu âm Trước & Sau ($cm$) | Secondary (Thêm sau) | ROS2 `sensor_msgs/msg/Range` phục vụ né vật cản khẩn cấp |
| **Battery Voltage** | Điện áp pin đọc từ ADC ESP32 | Secondary (Thêm sau) | ROS2 `sensor_msgs/msg/BatteryState` cảnh báo pin |
| **Motor PWM Feedback** | Giá trị PWM thực tế xuất ra 4 motor | Secondary (Thêm sau) | Debug chẩn đoán driver |

### B. Lệnh Raspberry Pi gửi xuống ESP32 (Control Commands)

1. **`CMD_VEL` (Bắt buộc)**: Chứa $linear.x$ ($m/s$), $linear.y$ ($m/s$), $angular.z$ ($rad/s$) phát ra từ ROS2 Nav2 hoặc Teleop.
2. **`CMD_SET_MODE` (Bắt buộc)**: Chuyển đổi mode hoạt động (`MODE_ROS2`, `MODE_MANUAL`, `MODE_AUTO`, `MODE_TEST`).
3. **`CMD_EMERGENCY_STOP` (Bắt buộc)**: Lệnh dừng khẩn cấp lập tức ngắt toàn bộ PWM motor về 0.
4. **`CMD_RESET_ODOM` (Khuyên dùng)**: Đặt lại số xung tích lũy 4 encoder về 0 khi bắt đầu Map SLAM mới.
5. **`CMD_RESET_IMU` (Khuyên dùng)**: Hiệu chỉnh lại góc Yaw IMU về 0.

---

## 8. ĐỀ XUẤT GIAO THỨC TRUYỀN THÔNG (SERIAL PROTOCOL)

### So sánh các dạng giao thức:

| Tiêu chí | JSON Protocol | CSV Protocol | Binary Packet Frame (COBS + CRC16) |
| :--- | :--- | :--- | :--- |
| **Kích thước gói tin** | Rất lớn (~150 - 200 bytes) | Trung bình (~50 - 80 bytes) | **Cực nhỏ (~24 - 32 bytes)** |
| **Tải CPU ESP32** | Cao (Tốn thời gian Parse String) | Trung bình (Tốn String split) | **Gần như bằng 0 (Memory Copy)** |
| **Tần số đáp ứng** | Thấp (< 10-20 Hz) | Khá (20-30 Hz) | **Rất cao (50Hz - 100Hz mượt mà)** |
| **Độ tin cậy chống nhiễu**| Kém (Dễ trôi vỡ chuỗi ASCII) | Kém (Thiếu checksum mạnh) | **Cực cao (Header Sync + CRC16 Checksum)** |

### **KHUYÊN DÙNG: BINARY PACKET FRAMEWORK KÈM CRC16.**

#### Cấu trúc Gói tin Nhị phân (Binary Frame Standard):

```text
 ┌──────────────┬───────────┬───────────┬─────────┬──────────────────────┬───────────────┬──────────────┐
 │ Header (2B)  │ MsgID(1B) │ Length(1B)│ Data... │ Payload (N Bytes)    │ CRC16 (2B)    │ Tail (1B)    │
 │ 0xFF  0xFE   │ 0x01/0x02 │    N      │         │ Telemetry/Cmd Struct │ Checksum Low/H│     0xFD     │
 └──────────────┴───────────┴───────────┴─────────┴──────────────────────┴───────────────┴──────────────┘
```

---

## 9. CÓ CẦN SỬA MOTOR, PID, ENCODER, MPU6050 KHÔNG?

### **"KHÔNG CẦN SỬA."**

- **Motor (`lib/Motor`)**: Đã hoàn chỉnh API điều phối 4 bánh.
- **Kinematics (`lib/Kinematics`)**: Đã hoàn chỉnh công thức đổi $v_x, v_y, \omega \to \text{PWM}$.
- **EncoderReader (`lib/EncoderReader`)**: Đã hoàn chỉnh logic đọc ngắt ISR 4 đĩa encoder và tính RPM.
- **MPU6050 (`lib/Mpu6050`)**: Đã hoàn chỉnh logic đọc IMU 6 trục và tích phân góc nghiêng.
- **PID Controller**: Khi tích hợp closed-loop PID sau này, ta chỉ cần tạo module riêng `lib/PIDController` nằm trung gian giữa `Kinematics` và `Motor` mà **KHÔNG CẦN SỬA** các module driver cũ.

---

## 10. DANH SÁCH FILE SẼ SỬA VÀ RỦI RO

| Tên File | Các thay đổi chi tiết | Mục đích | Rủi ro & Cách khắc phục |
| :--- | :--- | :--- | :--- |
| [include/Config.h](file:///e:/robot_AI/ROBOT_AI/include/Config.h) | Bổ sung `#define ROS2_SERIAL_BAUD 115200`<br>Bổ sung `#define ROS2_WATCHDOG_MS 500` | Định nghĩa tham số truyền thông ROS2 | **Rủi ro: Không có.** (Chỉ thêm hằng số compile-time). |
| [src/robot_global.h](file:///e:/robot_AI/ROBOT_AI/src/robot_global.h) | Bổ sung `MODE_ROS2` vào `enum OperatingMode`.<br>Bổ sung `extern EncoderReader encoder;`<br>Bổ sung `extern ROS2BridgeManager ros2Bridge;` | Chia sẻ trạng thái ROS2 & encoder toàn cục | **Rủi ro: Rất thấp.** Giữ nguyên giá trị enum cũ (`MODE_MANUAL=0`, `MODE_AUTO=1`). |
| [src/main.cpp](file:///e:/robot_AI/ROBOT_AI/src/main.cpp) | 1. Tạo instance `EncoderReader encoder;`<br>2. Gọi `encoder.begin()` trong `setup()`<br>3. Gọi `ros2Bridge.begin()` trong `setup()`<br>4. Gọi `encoder.update()` trong `loop()`<br>5. Gọi `ros2Bridge.update()` trong `loop()` | Điểm chèn khởi tạo và chạy máy trạng thái ROS2 | **Rủi ro: Thấp.** Đảm bảo các hàm gọi trong `loop()` hoàn toàn non-blocking để tránh làm chậm vòng lặp. |

---

## 11. DANH SÁCH FILE SẼ TẠO MỚI

1. `lib/ROS2Protocol/ROS2Protocol.h`: Định nghĩa cấu trúc gói nhị phân, Message ID (`CMD_VEL`, `TELEMETRY`, `ESTOP`), Struct dữ liệu Telemetry & Struct Command, CRC16.
2. `lib/ROS2Protocol/PacketBuilder.h` & `PacketBuilder.cpp`: Đóng gói dữ liệu Telemetry thành mảng byte kèm CRC16.
3. `lib/ROS2Protocol/PacketParser.h` & `PacketParser.cpp`: Máy trạng thái giải mã dòng byte Serial đọc từ Raspberry Pi.
4. `lib/ROS2Protocol/ROS2BridgeManager.h` & `ROS2BridgeManager.cpp`: Class quản lý luồng gửi/nhận ROS2 telemetry, tích hợp Watchdog ngắt khẩn cấp khi mất kết nối.

---

## 12. KIỂM TRA QUY CHUẨN KIẾN TRÚC THIẾT KẾ

### Đánh giá các nguyên lý phần mềm:
- **SOLID - Single Responsibility Principle (SRP)**: **TUÂN THỦ HOÀN HẢO**. Mỗi module trong `lib/` đảm nhận 1 nhiệm vụ riêng biệt. Việc tách `lib/ROS2Protocol` riêng giúp không vi phạm SRP.
- **Open/Closed Principle (OCP)**: **TUÂN THỦ HOÀN HẢO**. Hệ thống sẵn sàng mở rộng tính năng giao tiếp ROS2 mới mà không cần sửa đổi bất kỳ code driver cũ nào.
- **Layered Architecture (Kiến trúc phân tầng)**: **TUÂN THỦ HOÀN HẢO**. Tầng Driver Hardware $\to$ Tầng Subsystem/Kinematics $\to$ Tầng Application Logic $\to$ Tầng Orchestrator (`main.cpp`).
- **Modular Design**: **TUÂN THỦ HOÀN HẢO**. Tách biệt 100% giữa ứng dụng điều khiển xe và giao thức truyền thông.

---

## 13. KIẾN TRÚC TỔỔNG THỂ CUỐI CÙNG & LUỒNG DỮ LIỆU

### Sơ đồ Kiến trúc Đề xuất:

```text
ESP32 FIRMWARE
├── Hardware Drivers (lib/)
│   ├── BTS7960          <── Điều khiển PWM Cầu H Motor
│   ├── EncoderReader    <── Đọc ngắt phần cứng ISR Encoder 4 bánh
│   ├── Mpu6050          <── Đọc IMU 6 trục (I2C)
│   ├── Sensor_HC_SR04   <── Đọc khoảng cách Siêu âm
│   └── MH_FMD           <── Còi báo động Active Buzzer
├── Kinematics & Coordinator (lib/)
│   ├── Kinematics       <── Động học ngược Mecanum (vx, vy, w -> PWM 4 bánh)
│   └── Motor            <── Phối hợp điều khiển 4 bánh Mecanum
├── Communication Layer (lib/ - MODULE MỚI)
│   └── ROS2Protocol     <── PacketBuilder, PacketParser, ROS2BridgeManager, Watchdog
└── Applications & Orchestration (src/)
    ├── clien_dieukhien  <── Terminal CLI Debug cho Người dùng
    ├── auto_run         <── Standalone Auto Mode né vật cản
    ├── test_module      <── Menu Chẩn đoán Phần cứng
    └── main.cpp         <── Orchestrator điều phối loop() & setup()
```

### Luồng Dữ liệu (Data Flow):
1. **Luồng Lệnh (Raspberry Pi $\to$ ESP32)**:
   - ROS2 Node trên RPi gửi Byte Stream qua USB Serial.
   - `ros2Bridge.update()` trong `loop()` đọc Serial byte, `PacketParser` kiểm tra CRC16 trích xuất $v_x, v_y, \omega$.
   - Nếu `currentMode == MODE_ROS2`, `Kinematics::getWheelSpeeds(vx, vy, omega)` tính ra PWM 4 bánh.
   - Truyền PWM sang `car.setAllMotor(fl, fr, rl, rr)` để quay 4 bánh.
2. **Luồng Phản hồi (ESP32 $\to$ Raspberry Pi)**:
   - Trong `loop()`, `encoder.update()` và `mpu.update()` đọc xung và góc nghiêng.
   - `ros2Bridge` lấy Ticks/Speeds 4 bánh, IMU Roll/Pitch/Yaw/Accel/Gyro, Khoảng cách Siêu âm.
   - `PacketBuilder` đóng gói nhị phân + CRC16 và gửi lên RPi qua `Serial.write()` định kỳ 50Hz (mỗi 20ms).

---

## 14. KẾ HOẠCH TRIỂN KHAI THEO TỪNG BƯỚC

*(KHÔNG CODE - CHỈ LẬP KẾ HOẠCH)*

- **Bước 1: Thiết kế Giao thức Truyền thông Nhị phân (`lib/ROS2Protocol`)**
  - Tạo `ROS2Protocol.h` định nghĩa Cấu trúc Frame, Struct Telemetry, Struct Command, và CRC16 Checksum.
- **Bước 2: Xây dựng Module `PacketBuilder` & `PacketParser`**
  - Viết logic đóng gói struct telemetry thành byte stream kèm CRC16.
  - Viết máy trạng thái Parse byte stream từ Serial để trích xuất `cmd_vel`.
- **Bước 3: Xây dựng Class `ROS2BridgeManager` & Safety Watchdog**
  - Tích hợp Watchdog tự động ngắt motor dừng khẩn cấp nếu mất kết nối Serial quá 500ms khi ở `MODE_ROS2`.
- **Bước 4: Cập nhật `src/robot_global.h` & `include/Config.h`**
  - Khai báo thêm `MODE_ROS2` vào enum `OperatingMode`, thêm cấu hình baudrate & watchdog timeout.
- **Bước 5: Khởi tạo & Tích hợp trong `src/main.cpp`**
  - Instantiate `EncoderReader encoder;` và `ROS2BridgeManager ros2Bridge;`.
  - Khởi tạo trong `setup()` và chèn các điểm cập nhật non-blocking vào `loop()`.
- **Bước 6: Viết Script Test ROS2 trên Laptop / Raspberry Pi**
  - Viết Python script gửi `cmd_vel` nhị phân và đọc Telemetry từ ESP32 để kiểm tra độ trễ và độ chính xác dữ liệu trước khi kết nối với ROS2 Nodes chính thức.

---

## 15. ĐÁNH GIÁ RỦI RO VẬN HÀNH

- **Có làm hỏng Auto Run không?** -> **KHÔNG.** Chế độ Auto Run hoạt động độc lập khi `currentMode == MODE_AUTO`. Khi không ở `MODE_ROS2`, Auto Run chạy 100% như cũ.
- **Có làm hỏng Manual không?** -> **KHÔNG.** Chế độ Manual CLI qua Serial Monitor chạy khi `currentMode == MODE_MANUAL`. Phím tắt `w,a,s,d` hoạt động hoàn toàn bình thường.
- **Có làm hỏng PID không?** -> **KHÔNG.** Hiện tại project chưa dùng PID. Khi thêm PID sau này, nó sẽ đóng vai trò tầng trung gian độc lập.
- **Có làm hỏng Encoder không?** -> **KHÔNG.** Encoder đọc qua ngắt phần cứng ISR độc lập ở tầng thấp, không bị ảnh hưởng bởi tầng truyền thông.
- **Có làm hỏng Motor không?** -> **KHÔNG.** Tầng driver BTS7960 giữ nguyên 100%. Watchdog mới thậm chí còn giúp bảo vệ motor dừng ngay khi RPi bị đứt cáp hoặc crash.

---

## 16. BẢNG MA TRẬN FILE & RỦI RO (MATRIX)

| File / Module | Action | Risk Level | Priority | Nội dung thay đổi / Ghi chú |
| :--- | :--- | :--- | :--- | :--- |
| `lib/BTS7960/*` | **Không sửa** | None | - | Giữ nguyên 100% Driver PWM phần cứng |
| `lib/Motor/*` | **Không sửa** | None | - | Giữ nguyên 100% Coordinator 4 bánh Mecanum |
| `lib/Kinematics/*` | **Không sửa** | None | - | Giữ nguyên 100% Toán học động học nghịch |
| `lib/EncoderReader/*` | **Không sửa** | None | - | Giữ nguyên 100% Đọc ngắt phần cứng ISR Encoder |
| `lib/Mpu6050/*` | **Không sửa** | None | - | Giữ nguyên 100% Đọc IMU 6-DOF qua I2C |
| `lib/Sensor_HC_SR04/*`| **Không sửa** | None | - | Giữ nguyên 100% Đọc siêu âm luân phiên & Lọc Median |
| `lib/MH_FMD/*` | **Không sửa** | None | - | Giữ nguyên 100% Còi báo động Active Low |
| `src/auto_run.cpp` | **Không sửa** | None | - | Giữ nguyên 100% Logic Auto Run né vật cản |
| `src/clien_dieukhien.cpp`| **Không sửa** | None | - | Giữ nguyên 100% Terminal CLI Debug |
| `src/test_module.cpp` | **Không sửa** | None | - | Giữ nguyên 100% Menu Chẩn đoán Module |
| `include/Config.h` | **Sửa (Thêm)** | Low | Medium | Thêm hằng số Serial Baudrate & Watchdog Timeout |
| `src/robot_global.h` | **Sửa (Thêm)** | Low | High | Thêm `MODE_ROS2` vào enum, khai báo extern encoder & bridge |
| `src/main.cpp` | **Sửa (Thêm)** | Low | High | Instantiate encoder & bridge, chèn điểm gọi trong `setup` & `loop` |
| `lib/ROS2Protocol/*` | **TẠO MỚI** | Low | High | Module giao tiếp nhị phân ROS2 (Builder, Parser, Manager, Watchdog) |

---

## 17. KẾT LUẬN

1. **Có nên giữ nguyên project không?** $\to$ **CÓ. NÊN GIỮ NGUYÊN ARCHITECTURE.**
2. **Có cần refactor không?** $\to$ **KHÔNG CẦN REFACTOR.**
3. **Có nên thêm module mới không?** $\to$ **CÓ. NÊN THÊM MODULE MỚI `lib/ROS2Protocol`.**
4. **Có nên sửa module cũ không?** $\to$ **KHÔNG SỬA MODULE CŨ**, chỉ bổ sung điểm chèn khởi tạo và gọi máy trạng thái tại `main.cpp` và `robot_global.h`.
5. **Kiến trúc này có đủ tốt để phát triển tiếp ROS2, SLAM, Nav2 và Web không?** $\to$ **CỰC KỲ ĐỦ TỐT.** Kiến trúc phân tầng non-blocking nhị phân này đảm bảo độ trễ thấp (< 20ms), độ tin cậy cao, cấp đầy đủ dữ liệu Odometry + IMU cho ROS2 EKF Node (`robot_localization`), giúp Raspberry Pi 4 chạy mượt mà SLAM (Cartographer/Slam Toolbox) và Nav2.
