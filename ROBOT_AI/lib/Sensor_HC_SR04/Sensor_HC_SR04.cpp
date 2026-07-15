/**
 * @file Sensor_HC_SR04.cpp
 * @brief Implementations for the Sensor_HC_SR04 module with advanced diagnostic functions.
 */

#include "Sensor_HC_SR04.h"
#include "driver/gpio.h"
#include "PinMap.h"

#define FILTER_SIZE 5

// =============================================================================
// KHAI BÁO BIẾN NỘI BỘ (INTERNAL VARIABLES)
// =============================================================================
static bool front_online = false;
static bool rear_online = false;
static float warning_distance = 50.0f;
static bool hc_sr04_debug = false; // Flag kiểm soát in debug của readSensor

// Lưu trữ các lý do offline cụ thể để hiển thị
static String front_offline_reason = "No Echo";
static String rear_offline_reason = "No Echo";

// 0: Alternating (Cả hai), 1: Only Front (Chỉ trước), 2: Only Rear (Chỉ sau)
static int current_test_mode = 0; 

struct RollingBuffer {
    float samples[FILTER_SIZE];
    uint8_t index;
    bool has_valid_data;
};

static RollingBuffer front_buffer;
static RollingBuffer rear_buffer;

// =============================================================================
// CÁC HÀM TIỆN ÍCH NỘI BỘ
// =============================================================================
static void initBuffer(RollingBuffer &buf, float defaultValue) {
    for (int i = 0; i < FILTER_SIZE; i++) {
        buf.samples[i] = defaultValue;
    }
    buf.index = 0;
    buf.has_valid_data = false;
}

static void addSample(RollingBuffer &buf, float val) {
    buf.samples[buf.index] = val;
    buf.index = (buf.index + 1) % FILTER_SIZE;
    buf.has_valid_data = true;
}

static float getMedian(RollingBuffer &buf) {
    if (!buf.has_valid_data) {
        return -1.0f;
    }
    float temp[FILTER_SIZE];
    for (int i = 0; i < FILTER_SIZE; i++) {
        temp[i] = buf.samples[i];
    }
    for (int i = 0; i < FILTER_SIZE - 1; i++) {
        for (int j = i + 1; j < FILTER_SIZE; j++) {
            if (temp[i] > temp[j]) {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }
    return temp[FILTER_SIZE / 2];
}

// =============================================================================
// IMPLEMENTATION CÁC HÀM API GIAO TIẾP
// =============================================================================

void HC_SR04_CheckConflicts() {
    // Quét toàn bộ project và in thông tin chập chân cảnh báo chéo phần cứng
    Serial.println(F("\n⚠️ [GPIO CONFLICT DETECTED]"));
    bool has_conflict = false;

    #ifdef ENC_RL_A
    if (ENC_RL_A == 37 || ENC_RL_A == 38 || ENC_RL_A == 39 || ENC_RL_A == 40) {
        Serial.printf("   - GPIO %d (HC-SR04 pin) trùng chân với ENC_RL_A trong PinMap.h!\n", ENC_RL_A);
        has_conflict = true;
    }
    #endif

    #ifdef ENC_RL_B
    if (ENC_RL_B == 37 || ENC_RL_B == 38 || ENC_RL_B == 39 || ENC_RL_B == 40) {
        Serial.printf("   - GPIO %d (HC-SR04 pin) trùng chân với ENC_RL_B trong PinMap.h!\n", ENC_RL_B);
        has_conflict = true;
    }
    #endif

    #ifdef ENC_FR_A
    if (ENC_FR_A == 37 || ENC_FR_A == 38 || ENC_FR_A == 39 || ENC_FR_A == 40) {
        Serial.printf("   - GPIO %d (HC-SR04 pin) trùng chân với ENC_FR_A trong PinMap.h!\n", ENC_FR_A);
        has_conflict = true;
    }
    #endif

    #ifdef ENC_FR_B
    if (ENC_FR_B == 37 || ENC_FR_B == 38 || ENC_FR_B == 39 || ENC_FR_B == 40) {
        Serial.printf("   - GPIO %d (HC-SR04 pin) trùng chân với ENC_FR_B trong PinMap.h!\n", ENC_FR_B);
        has_conflict = true;
    }
    #endif

    #ifdef ENC_FL_A
    if (ENC_FL_A == 37 || ENC_FL_A == 38 || ENC_FL_A == 39 || ENC_FL_A == 40) {
        Serial.printf("   - GPIO %d (HC-SR04 pin) trùng chân với ENC_FL_A trong PinMap.h!\n", ENC_FL_A);
        has_conflict = true;
    }
    #endif

    #ifdef ENC_FL_B
    if (ENC_FL_B == 37 || ENC_FL_B == 38 || ENC_FL_B == 39 || ENC_FL_B == 40) {
        Serial.printf("   - GPIO %d (HC-SR04 pin) trùng chân với ENC_FL_B trong PinMap.h!\n", ENC_FL_B);
        has_conflict = true;
    }
    #endif

    #ifdef ENC_RR_A
    if (ENC_RR_A == 37 || ENC_RR_A == 38 || ENC_RR_A == 39 || ENC_RR_A == 40) {
        Serial.printf("   - GPIO %d (HC-SR04 pin) trùng chân với ENC_RR_A trong PinMap.h!\n", ENC_RR_A);
        has_conflict = true;
    }
    #endif

    #ifdef ENC_RR_B
    if (ENC_RR_B == 37 || ENC_RR_B == 38 || ENC_RR_B == 39 || ENC_RR_B == 40) {
        Serial.printf("   - GPIO %d (HC-SR04 pin) trùng chân với ENC_RR_B trong PinMap.h!\n", ENC_RR_B);
        has_conflict = true;
    }
    #endif

    if (has_conflict) {
        Serial.println(F("   => GPIO Conflict detected with module: Encoder"));
    } else {
        Serial.println(F("   No GPIO conflicts detected."));
    }
    Serial.println(F("===========================\n"));
}

void HC_SR04_Init() {
    // 1. Cảnh báo trùng chân GPIO ngay khi khởi tạo
    HC_SR04_CheckConflicts();

    // 2. Giải phóng JTAG trên các chân GPIO tương ứng để chạy đúng GPIO thường
    gpio_reset_pin((gpio_num_t)HC_SR04_FRONT_TRIG);
    gpio_reset_pin((gpio_num_t)HC_SR04_FRONT_ECHO);
    gpio_reset_pin((gpio_num_t)HC_SR04_REAR_TRIG);
    gpio_reset_pin((gpio_num_t)HC_SR04_REAR_ECHO);

    // 3. Cấu hình chân TRIG làm OUTPUT
    pinMode(HC_SR04_FRONT_TRIG, OUTPUT);
    digitalWrite(HC_SR04_FRONT_TRIG, LOW);
    
    pinMode(HC_SR04_REAR_TRIG, OUTPUT);
    digitalWrite(HC_SR04_REAR_TRIG, LOW);

    // 4. Cấu hình chân ECHO làm INPUT chuẩn (Không Dùng PULLUP/PULLDOWN)
    pinMode(HC_SR04_FRONT_ECHO, INPUT);
    pinMode(HC_SR04_REAR_ECHO, INPUT);

    // 5. In sơ đồ chân phần cứng theo yêu cầu trong setup
    Serial.println(F("========================"));
    Serial.println(F("HC-SR04 GPIO Mapping"));
    Serial.println(F("========================"));
    Serial.printf("Front TRIG GPIO : %d\n", HC_SR04_FRONT_TRIG);
    Serial.printf("Front ECHO GPIO : %d\n", HC_SR04_FRONT_ECHO);
    Serial.println(F(""));
    Serial.printf("Rear TRIG GPIO  : %d\n", HC_SR04_REAR_TRIG);
    Serial.printf("Rear ECHO GPIO  : %d\n", HC_SR04_REAR_ECHO);
    Serial.println(F("========================"));

    // 6. Khởi tạo bộ đệm
    initBuffer(front_buffer, -1.0f);
    initBuffer(rear_buffer, -1.0f);

    front_online = false;
    rear_online = false;
    front_offline_reason = "No Init Reading";
    rear_offline_reason = "No Init Reading";
    current_test_mode = 0;
}

void HC_SR04_TestMode(int sensorSelect) {
    current_test_mode = sensorSelect;
    Serial.printf("\n⚙️ [HC-SR04] Thiet lap TestMode: %s\n", 
                  (sensorSelect == 0) ? "Alternating (Ca hai)" : 
                  (sensorSelect == 1) ? "Only Front (Chi truoc)" : "Only Rear (Chi sau)");
}

void HC_SR04_TestGPIO() {
    static unsigned long last_print = 0;
    if (millis() - last_print < 300) {
        return; 
    }
    last_print = millis();
    
    Serial.printf("GPIO37 = %s | GPIO38 = %s | GPIO39 = %s | GPIO40 = %s\n",
                  digitalRead(37) ? "HIGH" : "LOW",
                  digitalRead(38) ? "HIGH" : "LOW",
                  digitalRead(39) ? "HIGH" : "LOW",
                  digitalRead(40) ? "HIGH" : "LOW");
}

void HC_SR04_TestTrigger() {
    static unsigned long last_trigger = 0;
    if (millis() - last_trigger < 1000) {
        return; 
    }
    last_trigger = millis();

    // Front Trigger
    digitalWrite(HC_SR04_FRONT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(HC_SR04_FRONT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(HC_SR04_FRONT_TRIG, LOW);

    // Rear Trigger
    digitalWrite(HC_SR04_REAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(HC_SR04_REAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(HC_SR04_REAR_TRIG, LOW);

    Serial.println(F("Trigger Generated Successfully"));
}

float readSensor(const char* name, uint8_t trigPin, uint8_t echoPin) {
    // 1. Kiểm tra trạng thái pin Echo trước khi phát Trigger
    int before_val = digitalRead(echoPin);

    // 2. Phát xung kích hoạt (Trigger) đúng chuẩn HC-SR04
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // 3. Đo thời gian phản hồi bằng pulseIn với timeout 30ms (30000us)
    unsigned long duration = pulseIn(echoPin, HIGH, 30000);

    // 4. In debug cực chi tiết (chỉ khi hc_sr04_debug được bật)
    if (hc_sr04_debug) {
        Serial.printf("\n[%s]\n", name);
        Serial.printf("GPIO = %d / %d\n", trigPin, echoPin);
        Serial.printf("Echo Before = %s\n", (before_val == HIGH) ? "HIGH" : "LOW");
        Serial.println(F("Trigger Sent"));
    }

    if (duration > 0) {
        if (hc_sr04_debug) {
            Serial.println(F("Echo After = HIGH"));
            Serial.printf("Duration = %lu us\n", duration);
        }
        float distance = (float)duration * 0.0343f / 2.0f;
        if (hc_sr04_debug) {
            Serial.printf("Distance = %.1f cm\n", distance);
        }
        return distance;
    } else {
        if (hc_sr04_debug) {
            int after_val = digitalRead(echoPin);
            Serial.printf("Echo After = %s\n", (after_val == HIGH) ? "HIGH" : "LOW");
            Serial.println(F("pulseIn Timeout"));
            Serial.println(F("Reason"));
            if (after_val == HIGH) {
                Serial.println(F("Echo Pin Always HIGH"));
                Serial.println(F("Possible Causes"));
                Serial.println(F("- Short Circuit"));
                Serial.println(F("- Wrong Wiring"));
                Serial.println(F("- Echo Connected To VCC"));
                Serial.println(F("- Sensor Failure"));
            } else {
                Serial.println(F("Echo Pin Always LOW"));
                Serial.println(F("Possible Causes"));
                Serial.println(F("- No 5V Power"));
                Serial.println(F("- Sensor Failure"));
                Serial.println(F("- Wiring Error"));
                Serial.println(F("- Echo Wire Disconnected"));
            }
        }
        return -1.0f;
    }
}

void HC_SR04_Update() {
    unsigned long now = millis();
    static unsigned long last_measurement_time = 0;
    static int state = 0; // 0: FRONT, 1: REAR

    // Đo cách quãng ~60ms luân phiên không block CPU
    if (now - last_measurement_time < 60) {
        return;
    }

    if (current_test_mode == 1) {
        // Chỉ đo Front
        float dist = readSensor("FRONT", HC_SR04_FRONT_TRIG, HC_SR04_FRONT_ECHO);
        if (dist >= 2.0f && dist <= 450.0f) {
            addSample(front_buffer, dist);
            front_online = true;
            front_offline_reason = "None";
        } else {
            front_online = false;
            int echo_state = digitalRead(HC_SR04_FRONT_ECHO);
            front_offline_reason = (echo_state == HIGH) ? "Echo Stuck HIGH" : "Echo Stuck LOW / Timeout";
        }
        last_measurement_time = millis();
    } 
    else if (current_test_mode == 2) {
        // Chỉ đo Rear
        float dist = readSensor("REAR", HC_SR04_REAR_TRIG, HC_SR04_REAR_ECHO);
        if (dist >= 2.0f && dist <= 450.0f) {
            addSample(rear_buffer, dist);
            rear_online = true;
            rear_offline_reason = "None";
        } else {
            rear_online = false;
            int echo_state = digitalRead(HC_SR04_REAR_ECHO);
            rear_offline_reason = (echo_state == HIGH) ? "Echo Stuck HIGH" : "Echo Stuck LOW / Timeout";
        }
        last_measurement_time = millis();
    } 
    else {
        // Chế độ xen kẽ (Alternating)
        if (state == 0) {
            float dist = readSensor("FRONT", HC_SR04_FRONT_TRIG, HC_SR04_FRONT_ECHO);
            if (dist >= 2.0f && dist <= 450.0f) {
                addSample(front_buffer, dist);
                front_online = true;
                front_offline_reason = "None";
            } else {
                front_online = false;
                int echo_state = digitalRead(HC_SR04_FRONT_ECHO);
                front_offline_reason = (echo_state == HIGH) ? "Echo Stuck HIGH" : "Echo Stuck LOW / Timeout";
            }
            last_measurement_time = millis();
            state = 1;
        } else {
            float dist = readSensor("REAR", HC_SR04_REAR_TRIG, HC_SR04_REAR_ECHO);
            if (dist >= 2.0f && dist <= 450.0f) {
                addSample(rear_buffer, dist);
                rear_online = true;
                rear_offline_reason = "None";
            } else {
                rear_online = false;
                int echo_state = digitalRead(HC_SR04_REAR_ECHO);
                rear_offline_reason = (echo_state == HIGH) ? "Echo Stuck HIGH" : "Echo Stuck LOW / Timeout";
            }
            last_measurement_time = millis();
            state = 0;
        }
    }
}

float HC_SR04_GetFrontDistance() {
    return getMedian(front_buffer);
}

float HC_SR04_GetRearDistance() {
    return getMedian(rear_buffer);
}

float HC_SR04_GetMinDistance() {
    float f = HC_SR04_GetFrontDistance();
    float r = HC_SR04_GetRearDistance();
    if (f < 0.0f) return r;
    if (r < 0.0f) return f;
    return (f < r) ? f : r;
}

float HC_SR04_GetMaxDistance() {
    float f = HC_SR04_GetFrontDistance();
    float r = HC_SR04_GetRearDistance();
    if (f < 0.0f) return r;
    if (r < 0.0f) return f;
    return (f > r) ? f : r;
}

bool HC_SR04_FrontObstacle(float cm) {
    float f = HC_SR04_GetFrontDistance();
    return (f > 0.0f && f <= cm);
}

bool HC_SR04_RearObstacle(float cm) {
    float r = HC_SR04_GetRearDistance();
    return (r > 0.0f && r <= cm);
}

bool HC_SR04_HasObstacle(float cm) {
    return (HC_SR04_FrontObstacle(cm) || HC_SR04_RearObstacle(cm));
}

bool HC_SR04_FrontOnline() {
    return front_online;
}

bool HC_SR04_RearOnline() {
    return rear_online;
}

bool HC_SR04_AllOnline() {
    return (front_online && rear_online);
}

void HC_SR04_SetWarningDistance(float cm) {
    warning_distance = cm;
}

float HC_SR04_GetWarningDistance() {
    return warning_distance;
}

void HC_SR04_Print() {
    Serial.print(F("[HC-SR04] F: "));
    if (front_online) {
        Serial.print(HC_SR04_GetFrontDistance(), 1);
        Serial.print(F(" cm"));
    } else {
        Serial.print(F("OFFLINE (Reason: "));
        Serial.print(front_offline_reason);
        Serial.print(F(")"));
    }

    Serial.print(F(" | R: "));
    if (rear_online) {
        Serial.print(HC_SR04_GetRearDistance(), 1);
        Serial.print(F(" cm"));
    } else {
        Serial.print(F("OFFLINE (Reason: "));
        Serial.print(rear_offline_reason);
        Serial.print(F(")"));
    }

    float minDist = HC_SR04_GetMinDistance();
    Serial.print(F(" | Min: "));
    if (minDist >= 0.0f) {
        Serial.print(minDist, 1);
        Serial.print(F(" cm"));
    } else {
        Serial.print(F("N/A"));
    }

    Serial.print(F(" | Warn Limit: "));
    Serial.print(warning_distance, 1);
    Serial.print(F(" cm | Obstacle: "));
    Serial.println(HC_SR04_HasObstacle(warning_distance) ? F("YES") : F("NO"));

    // Hiển thị trạng thái logic tĩnh của chân pin để chẩn đoán phần cứng
    Serial.printf("[HC-SR04 Debug] Tinh Pin: F_TRIG=%d, F_ECHO=%d | R_TRIG=%d, R_ECHO=%d\n",
                  digitalRead(HC_SR04_FRONT_TRIG), digitalRead(HC_SR04_FRONT_ECHO),
                  digitalRead(HC_SR04_REAR_TRIG), digitalRead(HC_SR04_REAR_ECHO));
}

void HC_SR04_SetDebug(bool enable) {
    hc_sr04_debug = enable;
}

bool HC_SR04_GetDebug() {
    return hc_sr04_debug;
}
