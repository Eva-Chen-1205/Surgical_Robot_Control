#include "encoder_manager.h"

namespace {
constexpr int NUM_ENCODERS = 5;
const int ENCA_PINS[NUM_ENCODERS] = {PB14, PB12, PB3, PB10, PA4};
const int ENCB_PINS[NUM_ENCODERS] = {PB15, PB13, PB4, PB11, PA5};

volatile long encoderCount[NUM_ENCODERS] = {0, 0, 0, 0, 0};
volatile int16_t encoder4Sub = 0;
volatile bool encoderInvert[NUM_ENCODERS] = {false, false, false, false, false};

inline int readStep(int idx) {
  const bool a = digitalRead(ENCA_PINS[idx]);
  const bool b = digitalRead(ENCB_PINS[idx]);
  int step = (a == b) ? 1 : -1;
  if (encoderInvert[idx]) {
    step = -step;
  }
  return step;
}

inline void readEncoder(int idx) {
  encoderCount[idx] += readStep(idx);
}

void readEncoder0() { readEncoder(0); }
void readEncoder1() { readEncoder(1); }
void readEncoder2() { readEncoder(2); }
void readEncoder3() { readEncoder(3); }

void readEncoder4() {
  const int step = readStep(4);
  encoder4Sub += step;

  if (encoder4Sub >= 100) {
    encoderCount[4]++;
    encoder4Sub -= 100;
  } else if (encoder4Sub <= -100) {
    encoderCount[4]--;
    encoder4Sub += 100;
  }
}
}  // namespace

void encoderInit() {
  for (int i = 0; i < NUM_ENCODERS; ++i) {
    pinMode(ENCA_PINS[i], INPUT_PULLUP);
    pinMode(ENCB_PINS[i], INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(ENCA_PINS[0]), readEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_PINS[1]), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_PINS[2]), readEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_PINS[3]), readEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_PINS[4]), readEncoder4, CHANGE);
}

long getEncoderCount(int index) {
  if (index < 0 || index >= NUM_ENCODERS) {
    return 0;
  }

  noInterrupts();
  const long count = encoderCount[index];
  interrupts();
  return count;
}

void setEncoderInvert(int index, bool invert) {
  if (index < 0 || index >= NUM_ENCODERS) {
    return;
  }

  noInterrupts();
  encoderInvert[index] = invert;
  interrupts();
}

bool getEncoderInvert(int index) {
  if (index < 0 || index >= NUM_ENCODERS) {
    return false;
  }

  noInterrupts();
  const bool invert = encoderInvert[index];
  interrupts();
  return invert;
}

void resetEncoderCount(int index) {
  if (index < 0 || index >= NUM_ENCODERS) {
    return;
  }

  noInterrupts();
  encoderCount[index] = 0;
  if (index == 4) {
    encoder4Sub = 0;
  }
  interrupts();
}

void resetAllEncoderCounts() {
  noInterrupts();
  for (int i = 0; i < NUM_ENCODERS; ++i) {
    encoderCount[i] = 0;
  }
  encoder4Sub = 0;
  interrupts();
}
