#ifndef MOTION_HANDLERS_H
#define MOTION_HANDLERS_H

#include "IMotionHandler.h"
#include "Kinematics.h"

// Forward
class ForwardMotionHandler : public IMotionHandler {
public:
    void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) override {
        int speed = cmd.speed;
        int leftSpeed = constrain(speed - yawCorrection, -255, 255);
        int rightSpeed = constrain(speed + yawCorrection, -255, 255);
        car.setAllMotor(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    }
};

// Backward
class BackwardMotionHandler : public IMotionHandler {
public:
    void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) override {
        car.backward(cmd.speed);
    }
};

// Strafe Left
class StrafeLeftMotionHandler : public IMotionHandler {
public:
    void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) override {
        car.strafeLeft(cmd.speed);
    }
};

// Strafe Right
class StrafeRightMotionHandler : public IMotionHandler {
public:
    void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) override {
        car.strafeRight(cmd.speed);
    }
};

// Rotate Left
class RotateLeftMotionHandler : public IMotionHandler {
public:
    void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) override {
        car.rotateLeft(cmd.speed);
    }
};

// Rotate Right
class RotateRightMotionHandler : public IMotionHandler {
public:
    void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) override {
        car.rotateRight(cmd.speed);
    }
};

// Diagonal FL
class DiagonalFLMotionHandler : public IMotionHandler {
public:
    void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) override {
        car.diagonalFrontLeft(cmd.speed);
    }
};

// Diagonal FR
class DiagonalFRMotionHandler : public IMotionHandler {
public:
    void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) override {
        car.diagonalFrontRight(cmd.speed);
    }
};

// Diagonal BL
class DiagonalBLMotionHandler : public IMotionHandler {
public:
    void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) override {
        car.diagonalBackLeft(cmd.speed);
    }
};

// Diagonal BR
class DiagonalBRMotionHandler : public IMotionHandler {
public:
    void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) override {
        car.diagonalBackRight(cmd.speed);
    }
};

// Stop
class StopMotionHandler : public IMotionHandler {
public:
    void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) override {
        car.stop();
    }
};

// General Strafe (Mecanum Kinematics)
class StrafeMotionHandler : public IMotionHandler {
private:
    Kinematics _kinematics;
public:
    void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) override {
        float scaleVx = cmd.vx * 510.0f; // 0.5 m/s -> 255
        float scaleVy = cmd.vy * 510.0f;
        float scaleW  = cmd.wz * 150.0f; // 1.7 rad/s -> 255
        
        scaleW += yawCorrection;
        
        WheelSpeeds speeds = _kinematics.getWheelSpeeds(scaleVx, scaleVy, scaleW);
        car.setAllMotor(speeds.fl, speeds.fr, speeds.rl, speeds.rr);
    }
};

// Arc (Circular motion)
class ArcMotionHandler : public IMotionHandler {
public:
    void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) override {
        float diff = cmd.wz; // wz representing turn rate/radius ratio [-1.0, 1.0]
        int leftSpeed = constrain(cmd.speed * (1.0f - diff), -255, 255);
        int rightSpeed = constrain(cmd.speed * (1.0f + diff), -255, 255);
        car.setAllMotor(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    }
};

#endif // MOTION_HANDLERS_H
