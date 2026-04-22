#include <Arduino.h>

#include <stdlib.h>
#include <string.h>

#include "encoder_manager.h"
#include "finger.h"

namespace {
constexpr int NUM_MOTORS = 5;
constexpr uint32_t CONTROL_US = 5000;      // 200 Hz
constexpr uint32_t ENCODER_PRINT_MS = 50;  // 20 Hz
constexpr size_t TARGET_FRAME_BYTES = NUM_MOTORS * sizeof(int32_t);
constexpr uint8_t FRAME_HEADER_0 = 0x59;   // 'Y'
constexpr uint8_t FRAME_HEADER_1 = 0x48;   // 'H'
constexpr size_t COMMAND_BUFFER_SIZE = 128;

enum class RxMode {
  Idle,
  BinaryFrame,
  Command
};

FingerController fingers[NUM_MOTORS];
int32_t targets[NUM_MOTORS] = {0, 0, 0, 0, 0};
uint8_t rxBuffer[TARGET_FRAME_BYTES] = {0};
size_t frameIndex = 0;
uint8_t headerState = 0;
RxMode rxMode = RxMode::Idle;
char commandBuffer[COMMAND_BUFFER_SIZE] = {0};
size_t commandIndex = 0;

bool parseIntValue(const char* text, long& value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char* end = nullptr;
  value = strtol(text, &end, 10);
  return end != text && *end == '\0';
}

bool parseFloatValue(const char* text, float& value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char* end = nullptr;
  value = strtof(text, &end);
  return end != text && *end == '\0';
}

bool parseMotorIndex(const char* text, int& index) {
  long parsed = 0;
  if (!parseIntValue(text, parsed)) {
    return false;
  }
  if (parsed < 0 || parsed >= NUM_MOTORS) {
    return false;
  }
  index = static_cast<int>(parsed);
  return true;
}

void printConfig(int index) {
  Serial.print("CFG,");
  Serial.print(index);
  Serial.print(",");
  Serial.print(fingers[index].getKp(), 4);
  Serial.print(",");
  Serial.print(fingers[index].getKi(), 4);
  Serial.print(",");
  Serial.print(fingers[index].getKd(), 4);
  Serial.print(",");
  Serial.print(fingers[index].getHoldPwm());
  Serial.print(",");
  Serial.print(fingers[index].getHoldMs());
  Serial.print(",");
  Serial.print(fingers[index].getErrDeadband());
  Serial.print(",");
  Serial.print(fingers[index].getBacklash());
  Serial.print(",");
  Serial.println(fingers[index].getMaxPwm());
}

void printAllConfigs() {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    printConfig(i);
  }
}

void printPwmOutputs() {
  Serial.print("PWM:");
  for (int i = 0; i < NUM_MOTORS; ++i) {
    Serial.print(" ");
    Serial.print(i);
    Serial.print("=");
    Serial.print(fingers[i].getLastOutput());
    Serial.print("/");
    Serial.print(fingers[i].getMaxPwm());
  }
  Serial.println();
}

void configureFingers() {
  // Backlash defaults to 0 while tuning because compensation often worsens hunting near target.
  fingers[0].init(0, PB6, PB7, 0, 0.30f, 0.0f, 0.18f, 0, 0, 30, 0);
  fingers[1].init(1, PA2, PA3, 1, 0.30f, 0.0f, 0.18f, 0, 0, 30, 0);
  fingers[2].init(2, PB0, PB1, 2, 0.08f, 0.0f, 0.01f, 0, 0, 30, 0);
  fingers[3].init(3, PB8, PB9, 3, 0.18f, 0.0f, 0.03f, 0, 0, 30, 0);
  fingers[4].init(4, PA0, PA1, 4, 2.00f, 0.0f, 0.0f, 0, 0, 3, 0);
  fingers[0].setMaxPwm(40);
  fingers[1].setMaxPwm(40);
  fingers[2].setMaxPwm(35);
  fingers[3].setMaxPwm(35);
  fingers[4].setMaxPwm(25);
}

void applyTargets() {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    fingers[i].setTarget(targets[i]);
  }
}

void decodeFrameAndApply() {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    int32_t value =
        static_cast<int32_t>(static_cast<uint32_t>(rxBuffer[i * 4 + 0])) |
        static_cast<int32_t>(static_cast<uint32_t>(rxBuffer[i * 4 + 1]) << 8) |
        static_cast<int32_t>(static_cast<uint32_t>(rxBuffer[i * 4 + 2]) << 16) |
        static_cast<int32_t>(static_cast<uint32_t>(rxBuffer[i * 4 + 3]) << 24);
    targets[i] = value;
  }
  applyTargets();
}

void processCommandLine(char* line) {
  char* token = strtok(line, ",");
  if (token == nullptr) {
    return;
  }

  if (strcmp(token, "!GETALL") == 0) {
    printAllConfigs();
    return;
  }

  if (strcmp(token, "!ZEROENC") == 0) {
    resetAllEncoderCounts();
    Serial.println("ACK,ZEROENC");
    return;
  }

  int motorIndex = 0;
  if (strcmp(token, "!GET") == 0) {
    if (parseMotorIndex(strtok(nullptr, ","), motorIndex)) {
      printConfig(motorIndex);
    } else {
      Serial.println("ERR,GET");
    }
    return;
  }

  if (strcmp(token, "!PID") == 0) {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (parseMotorIndex(strtok(nullptr, ","), motorIndex) &&
        parseFloatValue(strtok(nullptr, ","), kp) &&
        parseFloatValue(strtok(nullptr, ","), ki) &&
        parseFloatValue(strtok(nullptr, ","), kd)) {
      fingers[motorIndex].setPid(kp, ki, kd);
      printConfig(motorIndex);
    } else {
      Serial.println("ERR,PID");
    }
    return;
  }

  if (strcmp(token, "!HOLD") == 0) {
    long holdPwm = 0;
    long holdMs = 0;
    long deadband = 0;
    if (parseMotorIndex(strtok(nullptr, ","), motorIndex) &&
        parseIntValue(strtok(nullptr, ","), holdPwm) &&
        parseIntValue(strtok(nullptr, ","), holdMs) &&
        parseIntValue(strtok(nullptr, ","), deadband)) {
      fingers[motorIndex].setHold(static_cast<int>(holdPwm),
                                  static_cast<unsigned long>(holdMs),
                                  deadband);
      printConfig(motorIndex);
    } else {
      Serial.println("ERR,HOLD");
    }
    return;
  }

  if (strcmp(token, "!BACKLASH") == 0) {
    long backlash = 0;
    if (parseMotorIndex(strtok(nullptr, ","), motorIndex) &&
        parseIntValue(strtok(nullptr, ","), backlash)) {
      fingers[motorIndex].setBacklash(backlash);
      printConfig(motorIndex);
    } else {
      Serial.println("ERR,BACKLASH");
    }
    return;
  }

  if (strcmp(token, "!PWM") == 0) {
    long maxPwm = 0;
    if (parseMotorIndex(strtok(nullptr, ","), motorIndex) &&
        parseIntValue(strtok(nullptr, ","), maxPwm)) {
      fingers[motorIndex].setMaxPwm(static_cast<int>(maxPwm));
      printConfig(motorIndex);
    } else {
      Serial.println("ERR,PWM");
    }
    return;
  }

  Serial.println("ERR,UNKNOWN");
}

void processIncomingSerial() {
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0) {
      break;
    }

    const uint8_t byteValue = static_cast<uint8_t>(incoming);

    if (rxMode == RxMode::BinaryFrame) {
      rxBuffer[frameIndex++] = byteValue;
      if (frameIndex >= TARGET_FRAME_BYTES) {
        decodeFrameAndApply();
        frameIndex = 0;
        rxMode = RxMode::Idle;
      }
      continue;
    }

    if (rxMode == RxMode::Command) {
      if (byteValue == '\r') {
        continue;
      }

      if (byteValue == '\n') {
        commandBuffer[commandIndex] = '\0';
        processCommandLine(commandBuffer);
        commandIndex = 0;
        rxMode = RxMode::Idle;
      } else if (commandIndex + 1 < COMMAND_BUFFER_SIZE) {
        commandBuffer[commandIndex++] = static_cast<char>(byteValue);
      } else {
        commandIndex = 0;
        rxMode = RxMode::Idle;
        Serial.println("ERR,COMMAND_TOO_LONG");
      }
      continue;
    }

    if (byteValue == '!') {
      rxMode = RxMode::Command;
      commandIndex = 0;
      commandBuffer[commandIndex++] = '!';
      headerState = 0;
      continue;
    }

    if (headerState == 0) {
      headerState = (byteValue == FRAME_HEADER_0) ? 1 : 0;
    } else {
      if (byteValue == FRAME_HEADER_1) {
        rxMode = RxMode::BinaryFrame;
        frameIndex = 0;
      }
      headerState = (byteValue == FRAME_HEADER_0) ? 1 : 0;
    }
  }
}

void printEncoders() {
  Serial.print("ENC:");
  for (int i = 0; i < NUM_MOTORS; ++i) {
    Serial.print(" ");
    Serial.print(i);
    Serial.print("=");
    Serial.print(getEncoderCount(i));
  }
  Serial.println();
}

void updateControl() {
  fingers[0].sendCmd();
  fingers[1].sendCmd();

  const float kSync = 0.5f;
  const int pwmMax = 150;
  const long syncDeadband = 10;
  sendParallelPair(fingers[2], fingers[3], kSync, pwmMax, syncDeadband);

  fingers[4].sendCmd();
}
}  // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t waitStart = millis();
  while (!Serial && (millis() - waitStart < 2000UL)) {
    delay(10);
  }

  encoderInit();
  configureFingers();
  applyTargets();

  Serial.println("YuhanSTM32Control Ready");
  Serial.println("RX frame: YH + 5 x int32 little-endian");
  Serial.println("PID cmd : !PID,motor,kp,ki,kd");
  Serial.println("HOLD cmd: !HOLD,motor,holdPwm,holdMs,deadband");
  Serial.println("BKL cmd : !BACKLASH,motor,value");
  Serial.println("PWM cmd : !PWM,motor,maxPwm");
  printAllConfigs();
  printEncoders();
  printPwmOutputs();
}

void loop() {
  processIncomingSerial();

  static uint32_t lastControlUs = 0;
  const uint32_t nowUs = micros();
  if (static_cast<uint32_t>(nowUs - lastControlUs) >= CONTROL_US) {
    lastControlUs = nowUs;
    updateControl();
  }

  static uint32_t lastPrintMs = 0;
  const uint32_t nowMs = millis();
  if (static_cast<uint32_t>(nowMs - lastPrintMs) >= ENCODER_PRINT_MS) {
    lastPrintMs = nowMs;
    printEncoders();
    printPwmOutputs();
  }
}
