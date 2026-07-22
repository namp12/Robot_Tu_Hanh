#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include "Encoder.h"
#include <Arduino.h>

class EncoderManager {
private:
    IEncoderDriver* _drivers[4];
    Encoder* _encoders[4];
    EncoderConfig _config;

    EncoderManager();

public:
    static EncoderManager& getInstance() {
        static EncoderManager instance;
        return instance;
    }

    ~EncoderManager();

    void begin();
    void update();
    void reset(int index);
    void resetAll();

    // Getters
    long getPulse(int index) const;
    float getRPM(int index) const;
    float getSpeed(int index) const;
    float getDistance(int index) const;
    int getDirection(int index) const;
    bool isHealthy(int index) const;
    Encoder* getEncoder(int index) const;

    // Odometry helpers
    float getLeftDistance() const;
    float getRightDistance() const;
    float getWheelDistance() const;
    float getWheelVelocity(int index) const;
    float getLinearVelocity() const;
    float getAngularVelocity() const;

    // Configuration
    void configureAll(const EncoderConfig& config);

    // ROS2 helper
    EncoderTelemetry getTelemetry(int index) const;
};

#endif // ENCODER_MANAGER_H
