#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#include <Arduino.h>
#include "MovementController.h"
#include "RobotStateManager.h"
#include "BuzzerManager.h"
#include "EncoderManager.h"

enum CommandType {
    CMD_MOVE,
    CMD_STOP,
    CMD_SET_MODE,
    CMD_BUZZER,
    CMD_RESET_ODOMETRY,
    CMD_PING
};

struct RobotCommand {
    CommandType type;
    float linear_x;
    float linear_y;
    float angular_z;
    RobotState mode;
    BuzzerMode buzzerMode;
};

class CommandManager {
private:
    MovementController* _moveControl;

    CommandManager();

public:
    static CommandManager& getInstance() {
        static CommandManager instance;
        return instance;
    }

    void begin(MovementController* moveControl);
    bool executeCommand(const RobotCommand& cmd);
};

#endif // COMMAND_MANAGER_H
