#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <Arduino.h>
#include <vector>
#include <functional>

struct ScheduleTask {
    unsigned long intervalMs;
    unsigned long lastRunMs;
    std::function<void()> callback;
};

class Scheduler {
private:
    std::vector<ScheduleTask> _tasks;

public:
    void registerTask(unsigned long intervalMs, std::function<void()> callback) {
        _tasks.push_back({intervalMs, 0, callback});
    }

    void tick() {
        unsigned long now = millis();
        for (auto& task : _tasks) {
            if (now - task.lastRunMs >= task.intervalMs) {
                if (task.lastRunMs == 0) {
                    task.lastRunMs = now;
                } else {
                    task.lastRunMs += task.intervalMs;
                }
                task.callback();
            }
        }
    }
};

#endif // SCHEDULER_H
