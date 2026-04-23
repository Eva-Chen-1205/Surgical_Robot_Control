#ifndef YUHAN_ENCODER_MANAGER_H
#define YUHAN_ENCODER_MANAGER_H

#include <Arduino.h>

void encoderInit();
long getEncoderCount(int index);
void setEncoderInvert(int index, bool invert);
bool getEncoderInvert(int index);
void resetEncoderCount(int index);
void resetAllEncoderCounts();

#endif  // YUHAN_ENCODER_MANAGER_H
