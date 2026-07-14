#ifndef MOTOR_H
#define MOTOR_H

#include "BTS7960.h"
#include <Arduino.h>

/**
 * 🚗 LỚP ĐIỀU KHIỂN HỆ THỐNG ĐỘNG CƠ (MOTOR COORDINATOR)
 * Điều phối hoạt động của 4 bánh xe để điều khiển hướng đi của xe Mecanum.
 */
class Motor {
private:
  BTS7960 *_motorFL;
  BTS7960 *_motorFR;
  BTS7960 *_motorRL;
  BTS7960 *_motorRR;

public:
  Motor(BTS7960 &motorFL, BTS7960 &motorFR, BTS7960 &motorRL, BTS7960 &motorRR);

  void begin();

  // Điều khiển từng bánh độc lập
  void setFrontLeft(int speed);
  void setFrontRight(int speed);
  void setRearLeft(int speed);
  void setRearRight(int speed);

  // Điều khiển 4 bánh đồng thời
  void setAllMotor(int fl, int fr, int rl, int rr);

  // Điều khiển hướng đi của Robot (Mecanum)
  void forward(int speed);
  void backward(int speed);
  
  // Di chuyển ngang (Strafe)
  void strafeLeft(int speed);
  void strafeRight(int speed);
  
  // Xoay (Rotate)
  void rotateLeft(int speed);
  void rotateRight(int speed);

  // Di chuyển chéo (Diagonal)
  void diagonalFrontLeft(int speed);
  void diagonalFrontRight(int speed);
  void diagonalBackLeft(int speed);
  void diagonalBackRight(int speed);

  void stop();
  void brake();
};

#endif // MOTOR_H
