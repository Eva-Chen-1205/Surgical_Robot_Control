#ifndef YUHAN_FINGER_H
#define YUHAN_FINGER_H

#include <Arduino.h>
#include "motor.h"

class FingerController {
 public:
  FingerController();

  void init(int id,
            int motorPinA,
            int motorPinB,
            int encoderIndex,
            float kp,
            float ki,
            float kd,
            int holdPwm,
            unsigned long holdMs,
            long errDeadband,
            long backlashComp);

  void setTarget(long target);
  void setPid(float kp, float ki, float kd);
  void setHold(int holdPwm, unsigned long holdMs, long errDeadband);
  void setBacklash(long backlashComp);
  void setMaxPwm(int maxPwm);
  void setMotorInvert(bool invert);
  void setEncoderInvert(bool invert);
  void resetControllerState();

  long getTarget() const;
  long readEncoder() const;
  float getKp() const;
  float getKi() const;
  float getKd() const;
  int getHoldPwm() const;
  unsigned long getHoldMs() const;
  long getErrDeadband() const;
  long getBacklash() const;
  int getMaxPwm() const;
  int getLastOutput() const;
  bool getMotorInvert() const;
  bool getEncoderInvert() const;
  int computeU();
  void applyU(int u);
  void sendCmd();

 private:
  Motor _motor;
  int _id;
  int _encoderIndex;
  float _kp;
  float _ki;
  float _kd;
  long _target;
  float _error;
  float _prevError;
  float _integral;
  int _holdPwm;
  unsigned long _holdMs;
  long _errDeadband;
  long _releaseDeadband;
  int _maxPwm;
  int _lastDir;
  int _lastOutput;
  bool _holding;
  bool _holdLatched;
  bool _settled;
  unsigned long _holdUntil;
  long _backlashComp;
  float _prevTarget;
  int _prevTargetDir;
  float _targetEps;
  unsigned long _targetStillMs;
  unsigned long _targetStableSince;
  bool _targetStill;
};

void sendParallelPair(FingerController& fingerA,
                      FingerController& fingerB,
                      float kSync,
                      int pwmMax,
                      long syncDeadband);

#endif  // YUHAN_FINGER_H
