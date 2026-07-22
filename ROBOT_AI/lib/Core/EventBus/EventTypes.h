#ifndef EVENT_TYPES_H
#define EVENT_TYPES_H

#include <Arduino.h>

enum EventType {
    EVENT_OBSTACLE_DETECTED,
    EVENT_BATTERY_LOW,
    EVENT_MOTOR_FAULT,
    EVENT_EMERGENCY_STOP,
    EVENT_GOAL_REACHED,
    EVENT_ENCODER_TIMEOUT,
    EVENT_ENCODER_FAILURE,
    EVENT_ENCODER_DIRECTION_ERROR
};

struct Event {
    EventType type;
    unsigned long timestamp;
    union {
        float distance;
        float voltage;
        int motor_id;
    } data;
};

class IEventSubscriber {
public:
    virtual ~IEventSubscriber() = default;
    virtual void onEvent(const Event& event) = 0;
};

#endif // EVENT_TYPES_H
