#ifndef EVENT_BUS_H
#define EVENT_BUS_H

#include "EventTypes.h"
#include <vector>
#include <unordered_map>

class EventBus {
private:
    std::unordered_map<EventType, std::vector<IEventSubscriber*>> _subscribers;

    EventBus() = default;

public:
    static EventBus& getInstance() {
        static EventBus instance;
        return instance;
    }

    void subscribe(EventType type, IEventSubscriber* subscriber) {
        _subscribers[type].push_back(subscriber);
    }

    void publish(const Event& event) {
        auto it = _subscribers.find(event.type);
        if (it != _subscribers.end()) {
            for (auto subscriber : it->second) {
                if (subscriber != nullptr) {
                    subscriber->onEvent(event);
                }
            }
        }
    }
};

#endif // EVENT_BUS_H
