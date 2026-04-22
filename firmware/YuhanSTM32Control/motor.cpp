#include "motor.h"

namespace {
constexpr int PWM_DEADBAND = 8;
}

Motor::Motor() : _pinA(-1), _pinB(-1) {}

void Motor::init(int pinA, int pinB) {
  _pinA = pinA;
  _pinB = pinB;
  pinMode(_pinA, OUTPUT);
  pinMode(_pinB, OUTPUT);
  stop();
}

void Motor::drive(int pwm) {
  if (_pinA < 0 || _pinB < 0) {
    return;
  }

  if (pwm > 255) {
    pwm = 255;
  } else if (pwm < -255) {
    pwm = -255;
  }

  if (pwm > -PWM_DEADBAND && pwm < PWM_DEADBAND) {
    stop();
    return;
  }

  if (pwm >= 0) {
    analogWrite(_pinB, 0);
    analogWrite(_pinA, pwm);
  } else {
    analogWrite(_pinA, 0);
    analogWrite(_pinB, -pwm);
  }
}

void Motor::stop() {
  if (_pinA < 0 || _pinB < 0) {
    return;
  }

  analogWrite(_pinA, 0);
  analogWrite(_pinB, 0);
}
