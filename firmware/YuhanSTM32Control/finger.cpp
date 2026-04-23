#include "finger.h"
#include "encoder_manager.h"

#include <math.h>

namespace {
constexpr float INTEGRAL_LIMIT = 50000.0f;

int clampInt(int value, int lo, int hi) {
  if (value < lo) {
    return lo;
  }
  if (value > hi) {
    return hi;
  }
  return value;
}

float clampFloat(float value, float lo, float hi) {
  if (value < lo) {
    return lo;
  }
  if (value > hi) {
    return hi;
  }
  return value;
}
}  // namespace

FingerController::FingerController()
    : _id(-1),
      _encoderIndex(0),
      _kp(0.0f),
      _ki(0.0f),
      _kd(0.0f),
      _target(0),
      _error(0.0f),
      _prevError(0.0f),
      _integral(0.0f),
      _holdPwm(0),
      _holdMs(0),
      _errDeadband(3),
      _releaseDeadband(8),
      _maxPwm(60),
      _lastDir(0),
      _lastOutput(0),
      _holding(false),
      _holdLatched(false),
      _settled(false),
      _holdUntil(0),
      _backlashComp(0),
      _prevTarget(0.0f),
      _prevTargetDir(0),
      _targetEps(0.001f),
      _targetStillMs(50),
      _targetStableSince(0),
      _targetStill(false) {}

void FingerController::init(int id,
                            int motorPinA,
                            int motorPinB,
                            int encoderIndex,
                            float kp,
                            float ki,
                            float kd,
                            int holdPwm,
                            unsigned long holdMs,
                            long errDeadband,
                            long backlashComp) {
  _id = id;
  _encoderIndex = encoderIndex;
  _motor.init(motorPinA, motorPinB);
  _target = 0;
  setPid(kp, ki, kd);
  setHold(holdPwm, holdMs, errDeadband);
  setBacklash(backlashComp);
  setMaxPwm(60);
  resetControllerState();
}

void FingerController::setTarget(long target) {
  _target = target;
}

void FingerController::setPid(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  resetControllerState();
}

void FingerController::setHold(int holdPwm, unsigned long holdMs, long errDeadband) {
  _holdPwm = holdPwm;
  _holdMs = holdMs;
  _errDeadband = errDeadband;
  _releaseDeadband = (_errDeadband > 0) ? (_errDeadband + 10) : 10;
  resetControllerState();
}

void FingerController::setBacklash(long backlashComp) {
  _backlashComp = backlashComp;
  resetControllerState();
}

void FingerController::setMaxPwm(int maxPwm) {
  if (maxPwm < 0) {
    maxPwm = 0;
  }
  if (maxPwm > 255) {
    maxPwm = 255;
  }
  _maxPwm = maxPwm;
}

void FingerController::setMotorInvert(bool invert) {
  _motor.setInvert(invert);
  resetControllerState();
}

void FingerController::setEncoderInvert(bool invert) {
  ::setEncoderInvert(_encoderIndex, invert);
  resetControllerState();
}

void FingerController::resetControllerState() {
  _error = 0.0f;
  _prevError = 0.0f;
  _integral = 0.0f;
  _lastDir = 0;
  _lastOutput = 0;
  _holding = false;
  _holdLatched = false;
  _settled = false;
  _holdUntil = 0;
  _prevTarget = static_cast<float>(_target);
  _prevTargetDir = 0;
  _targetStableSince = 0;
  _targetStill = false;
}

long FingerController::getTarget() const { return _target; }

long FingerController::readEncoder() const {
  return getEncoderCount(_encoderIndex);
}

float FingerController::getKp() const { return _kp; }
float FingerController::getKi() const { return _ki; }
float FingerController::getKd() const { return _kd; }
int FingerController::getHoldPwm() const { return _holdPwm; }
unsigned long FingerController::getHoldMs() const { return _holdMs; }
long FingerController::getErrDeadband() const { return _errDeadband; }
long FingerController::getBacklash() const { return _backlashComp; }
int FingerController::getMaxPwm() const { return _maxPwm; }
int FingerController::getLastOutput() const { return _lastOutput; }
bool FingerController::getMotorInvert() const { return _motor.isInverted(); }
bool FingerController::getEncoderInvert() const {
  return ::getEncoderInvert(_encoderIndex);
}

int FingerController::computeU() {
  const float dTarget = static_cast<float>(_target) - _prevTarget;
  const bool targetMoved = fabsf(dTarget) > _targetEps;

  if (targetMoved) {
    _targetStableSince = 0;
    _targetStill = false;
    _holding = false;
    _holdLatched = false;
    _settled = false;
  } else {
    if (_targetStableSince == 0) {
      _targetStableSince = millis();
    }
    _targetStill = (millis() - _targetStableSince) >= _targetStillMs;
  }

  int targetDir = 0;
  if (dTarget > _targetEps) {
    targetDir = 1;
  } else if (dTarget < -_targetEps) {
    targetDir = -1;
  }

  const bool dirChanged =
      (targetDir != 0 && _prevTargetDir != 0 && targetDir != _prevTargetDir);

  float effectiveTarget = static_cast<float>(_target);
  if (dirChanged && _backlashComp > 0) {
    effectiveTarget += static_cast<float>(targetDir * _backlashComp);
  }

  _error = effectiveTarget - static_cast<float>(readEncoder());

  // Crossing the target is a good time to dump integral residue.
  if ((_error > 0.0f && _prevError < 0.0f) || (_error < 0.0f && _prevError > 0.0f)) {
    _integral = 0.0f;
  }

  const float derivative = _error - _prevError;
  _integral = clampFloat(_integral + _error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  int u = static_cast<int>(_kp * _error + _kd * derivative + _ki * _integral);

  if (u > 0) {
    _lastDir = 1;
  } else if (u < 0) {
    _lastDir = -1;
  }

  const long deadband = (_errDeadband > 0) ? _errDeadband : 3;
  const long absError = labs(static_cast<long>(_error));
  const bool insideDeadband = absError <= deadband;

  if (insideDeadband && _targetStill) {
    // Entering the settle zone should zero integral to prevent hunting.
    _integral = 0.0f;

    if (!_settled) {
      _settled = true;
      _holding = true;
      _holdUntil = millis() + _holdMs;
    }

    // In hold mode, use a small sustained output in the error direction.
    // This lets the joint support load without bringing back full PID chatter.
    if (_holdPwm > 0) {
      const bool holdReady = (_holdMs == 0) || (millis() >= _holdUntil);
      if (holdReady) {
        if (_error > 0.0f) {
          u = _holdPwm;
        } else if (_error < 0.0f) {
          u = -_holdPwm;
        } else {
          u = 0;
        }
      } else {
        u = 0;
      }
    } else {
      u = 0;
    }
  } else if (_settled && absError >= _releaseDeadband) {
    // Rearm hold only after we clearly leave the settle zone.
    _settled = false;
    _holding = false;
  }

  _prevError = _error;
  _prevTarget = static_cast<float>(_target);
  _prevTargetDir = targetDir;
  return u;
}

void FingerController::applyU(int u) {
  u = clampInt(u, -_maxPwm, _maxPwm);
  _lastOutput = u;
  _motor.drive(u);
}

void FingerController::sendCmd() {
  applyU(computeU());
}

void sendParallelPair(FingerController& fingerA,
                      FingerController& fingerB,
                      float kSync,
                      int pwmMax,
                      long syncDeadband) {
  const int uA = fingerA.computeU();
  const int uB = fingerB.computeU();

  long errorA = fingerA.getTarget() - fingerA.readEncoder();
  long errorB = fingerB.getTarget() - fingerB.readEncoder();
  long syncError = errorA - errorB;
  if (labs(syncError) <= syncDeadband) {
    syncError = 0;
  }

  const int du = static_cast<int>(kSync * static_cast<float>(syncError));
  fingerA.applyU(clampInt(uA + du, -pwmMax, pwmMax));
  fingerB.applyU(clampInt(uB - du, -pwmMax, pwmMax));
}
