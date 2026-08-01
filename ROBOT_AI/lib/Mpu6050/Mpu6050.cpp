#include "Mpu6050.h"

MPU6050Sensor::MPU6050Sensor()
{
    accelX = 0;
    accelY = 0;
    accelZ = 0;

    gyroX = 0;
    gyroY = 0;
    gyroZ = 0;

    temperature = 0;

    roll = 0;
    pitch = 0;
    yaw = 0;

    previousTime = 0;
    _ready = false;
}

bool MPU6050Sensor::begin(uint8_t sda, uint8_t scl)
{
    // ----------------------------------------------------------------
    // Bước 1: Gọi Wire.begin() tường minh với đúng pin SDA/SCL
    //         TRƯỚC khi Adafruit_MPU6050::begin() gọi lại Wire.begin()
    //         (không tham số → default SDA=GPIO8, SCL=GPIO9 trên S3).
    //         Sau khi Wire.begin(sda, scl) → i2cIsInit() = true
    //         → Wire.begin() không tham số trong BusIO sẽ bị bỏ qua.
    // ----------------------------------------------------------------
    Wire.begin(sda, scl);
    delay(10); // ổn định I2C

    // Bước 2: Truyền &Wire vào Adafruit, thử cả 2 địa chỉ I2C 0x68 và 0x69
    if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire) && !mpu.begin(0x69, &Wire))
    {
        return false;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Bước 3: Tự động calib Gyro Z offset khi đứng yên lúc khởi động
    float sumZ = 0;
    for (int i = 0; i < 40; i++) {
        mpu.getEvent(&accel, &gyro, &temp);
        sumZ += gyro.gyro.z;
        delay(5);
    }
    gyroZOffset = sumZ / 40.0f;

    previousTime = millis();
    _ready = true;

    return true;
}

void MPU6050Sensor::update()
{
    if (!_ready) return;

    mpu.getEvent(&accel, &gyro, &temp);

    accelX = accel.acceleration.x;
    accelY = accel.acceleration.y;
    accelZ = accel.acceleration.z;

    gyroX = gyro.gyro.x;
    gyroY = gyro.gyro.y;
    gyroZ = gyro.gyro.z;

    temperature = temp.temperature;

    unsigned long currentTime = millis();
    float dt = (currentTime - previousTime) / 1000.0f;
    previousTime = currentTime;

    // Giới hạn dt tránh nhảy vọt dữ liệu nếu vòng lặp bị trễ hoặc khi khởi động
    if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;

    // Trừ offset tĩnh và áp dụng lọc vùng chết (Deadband Filter)
    float gZ = gyroZ - gyroZOffset;
    if (fabs(gZ) < 0.02f) gZ = 0.0f;

    // Tính toán góc nghiêng từ Gia tốc kế (Accelerometer) làm mốc tham chiếu
    float rollAccel = atan2(accelY, accelZ) * 57.2958f;
    float pitchAccel = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958f;

    // Bộ lọc bù (Complementary Filter): Kết hợp 98% Con quay hồi chuyển (tính phản ứng nhanh)
    // và 2% Gia tốc kế (ổn định lâu dài, không trôi) để tính Roll và Pitch
    roll  = 0.98f * (roll + gyroX * 57.2958f * dt) + 0.02f * rollAccel;
    pitch = 0.98f * (pitch + gyroY * 57.2958f * dt) + 0.02f * pitchAccel;

    // Trục Yaw chỉ có thể tích lũy từ Gyro (được khử trôi bằng deadband ở trên)
    yaw   += gZ * 57.2958f * dt;
}

void MPU6050Sensor::calibrate()
{
    roll = 0;
    pitch = 0;
    yaw = 0;
}

void MPU6050Sensor::resetAngle()
{
    roll = 0;
    pitch = 0;
    yaw = 0;
}

bool MPU6050Sensor::isConnected()
{
    return _ready;
}

float MPU6050Sensor::getAccelX() { return accelX; }
float MPU6050Sensor::getAccelY() { return accelY; }
float MPU6050Sensor::getAccelZ() { return accelZ; }

float MPU6050Sensor::getGyroX() { return gyroX; }
float MPU6050Sensor::getGyroY() { return gyroY; }
float MPU6050Sensor::getGyroZ() { return gyroZ; }

float MPU6050Sensor::getTemperature() { return temperature; }

float MPU6050Sensor::getRoll()  { return roll; }
float MPU6050Sensor::getPitch() { return pitch; }
float MPU6050Sensor::getYaw()   { return yaw; }