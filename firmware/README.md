# STM32 韌體說明

這裡放的是要燒進 STM32 的程式，而且全部都留在 `Yuhan_motor` 裡，不依賴其他舊資料夾。

## 主要檔案

- `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\YuhanSTM32Control.ino`
- `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\encoder_manager.cpp`
- `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\motor.cpp`
- `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\finger.cpp`

## 韌體功能

- 接收 target frame
- 執行 5 軸控制
- 輸出 encoder
- 輸出目前 PWM
- 支援即時調整：
  - PID
  - hold
  - backlash
  - max PWM

## 通訊格式

### Target frame

- header: `YH`
- payload: `5 x int32 little-endian`

總長度：`22 bytes`

### 文字命令

```text
!GETALL
!GET,0
!PID,0,0.30,0.00,0.18
!HOLD,0,0,0,30
!BACKLASH,0,0
!PWM,0,30
!ZEROENC
```

### 回傳格式

```text
ENC: 0=123 1=456 2=789 3=10 4=11
CFG,0,0.3000,0.0000,0.1800,0,0,30,0,40
PWM: 0=12/40 1=0/40 2=-5/35 3=0/35 4=0/25
```

## Arduino IDE 燒錄步驟

1. 安裝 Arduino IDE 2.x
2. 安裝 STM32 Arduino Core
3. 開啟：

   `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\YuhanSTM32Control.ino`

4. 選對板子
5. 選對 COM Port
6. 選對 Upload Method
7. 按 `Verify`
8. 按 `Upload`

## 調參提醒

### 如果會來回擺動

1. `backlash = 0`
2. `holdPwm = 0`
3. `holdMs = 0`
4. `Ki = 0`
5. `maxPwm = 20`
6. 降低 `Kp`
7. 再小幅增加 `Kd`

### 如果到位後會滑掉

1. `holdPwm = 2`
2. `holdMs = 0`
3. `maxPwm >= holdPwm`
4. 還不夠再加到 `3`、`5`

## 腳位

### Motor pins

- Motor 0: `PB6`, `PB7`
- Motor 1: `PA2`, `PA3`
- Motor 2: `PB0`, `PB1`
- Motor 3: `PB8`, `PB9`
- Motor 4: `PA0`, `PA1`

### Encoder pins

- Encoder 0: `PB14`, `PB15`
- Encoder 1: `PB12`, `PB13`
- Encoder 2: `PB3`, `PB4`
- Encoder 3: `PB10`, `PB11`
- Encoder 4: `PA4`, `PA5`
