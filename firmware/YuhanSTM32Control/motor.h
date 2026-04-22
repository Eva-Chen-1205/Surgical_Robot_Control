#ifndef YUHAN_MOTOR_H
#define YUHAN_MOTOR_H

#include <Arduino.h>

class Motor {
 public:
  Motor();
  void init(int pinA, int pinB);
  void drive(int pwm);
  void stop();

 private:
  int _pinA;
  int _pinB;
};

#endif  // YUHAN_MOTOR_H
