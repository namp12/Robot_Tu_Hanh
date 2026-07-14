#include "Motor.h"

Motor::Motor(BTS7960 &motorFL, BTS7960 &motorFR, BTS7960 &motorRL,
             BTS7960 &motorRR) {
  _motorFL = &motorFL;
  _motorFR = &motorFR;
  _motorRL = &motorRL;
  _motorRR = &motorRR;
}

void Motor::begin() {
  _motorFL->begin();
  _motorFR->begin();
  _motorRL->begin();
  _motorRR->begin();
}

void Motor::setFrontLeft(int speed) { _motorFL->setSpeed(speed); }

void Motor::setFrontRight(int speed) { _motorFR->setSpeed(speed); }

void Motor::setRearLeft(int speed) { _motorRL->setSpeed(speed); }

void Motor::setRearRight(int speed) { _motorRR->setSpeed(speed); }

void Motor::setAllMotor(int fl, int fr, int rl, int rr) {
  _motorFL->setSpeed(fl);
  _motorFR->setSpeed(fr);
  _motorRL->setSpeed(rl);
  _motorRR->setSpeed(rr);
}

void Motor::forward(int speed) { setAllMotor(speed, speed, speed, speed); }

void Motor::backward(int speed) { setAllMotor(-speed, -speed, -speed, -speed); }

void Motor::strafeLeft(int speed) { setAllMotor(-speed, speed, speed, -speed); }

void Motor::strafeRight(int speed) { setAllMotor(speed, -speed, -speed, speed); }

void Motor::rotateLeft(int speed) { setAllMotor(-speed, speed, -speed, speed); }

void Motor::rotateRight(int speed) { setAllMotor(speed, -speed, speed, -speed); }

void Motor::diagonalFrontLeft(int speed) { setAllMotor(0, speed, speed, 0); }

void Motor::diagonalFrontRight(int speed) { setAllMotor(speed, 0, 0, speed); }

void Motor::diagonalBackLeft(int speed) { setAllMotor(-speed, 0, 0, -speed); }

void Motor::diagonalBackRight(int speed) { setAllMotor(0, -speed, -speed, 0); }

void Motor::stop() {
  _motorFL->stop();
  _motorFR->stop();
  _motorRL->stop();
  _motorRR->stop();
}

void Motor::brake() {
  _motorFL->brake();
  _motorFR->brake();
  _motorRL->brake();
  _motorRR->brake();
}
