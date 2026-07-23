#include "CommandManager.h"

CommandManager::CommandManager() : _moveControl(nullptr) {}

void CommandManager::begin(MovementController* moveControl) {
    _moveControl = moveControl;
}

bool CommandManager::executeCommand(const RobotCommand& cmd) {
    switch (cmd.type) {
        case CMD_MOVE:
            if (_moveControl != nullptr) {
                _moveControl->move(cmd.linear_x, cmd.linear_y, cmd.angular_z);
            }
            break;

        case CMD_STOP:
            if (_moveControl != nullptr) {
                _moveControl->stop();
            }
            break;

        case CMD_SET_MODE:
            RobotStateManager::getInstance().setState(cmd.mode);
            break;

        case CMD_BUZZER:
            BuzzerManager::getInstance().setMode(cmd.buzzerMode);
            break;

        case CMD_RESET_ODOMETRY:
#if ENCODER_ENABLED
            EncoderManager::getInstance().resetAll();
#endif
            break;

        case CMD_PING:
            BuzzerManager::getInstance().beep(100);
            break;

        default:
            return false;
    }
    return true;
}
