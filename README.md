# Yuhan Motor Console

這個資料夾是獨立可保存的控制包。之後如果你把其他舊資料夾刪掉，只保留 `D:\Codex\Control\Yuhan_motor`，這一包還是可以繼續用。

它包含兩部分：

- 電腦端控制頁面：`D:\Codex\Control\Yuhan_motor\index.html`
- STM32 韌體：`D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\YuhanSTM32Control.ino`

---

## 檔案結構

```text
Yuhan_motor/
  app.js
  example_trajectory.txt
  index.html
  README.md
  server.py
  start_console.ps1
  styles.css
  firmware/
    README.md
    YuhanSTM32Control/
      YuhanSTM32Control.ino
      encoder_manager.h
      encoder_manager.cpp
      motor.h
      motor.cpp
      finger.h
      finger.cpp
```

---

## 這一包現在能做什麼

### 電腦端頁面

- 連接 STM32
- 送 5 軸 target
- 手動 `+/-` jog
- 播放軌跡檔
- 顯示 encoder
- 顯示目前 PWM 輸出與 PWM 上限
- 匯出 encoder log 成 `.csv`
- 調每一軸的：
  - `Kp`
  - `Ki`
  - `Kd`
  - `holdPwm`
  - `holdMs`
  - `deadband`
  - `backlash`
  - `maxPwm`

### STM32 韌體

- 接收 5 軸目標值
- 執行 5 軸控制
- 讀 encoder
- 回傳 encoder 與 PWM
- 接收 PID / hold / backlash / max PWM 參數更新

---

## 詳細使用步驟

### A. 先燒 STM32

1. 安裝 Arduino IDE 2.x
2. 安裝 STM32 Arduino Core
3. 開啟：

   `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\YuhanSTM32Control.ino`

4. 選對你的 STM32 板子
5. 選對 COM Port
6. 選對 Upload Method
7. 按 `Verify`
8. 按 `Upload`

### B. 開控制頁

1. 開 PowerShell
2. 進到：

   `D:\Codex\Control\Yuhan_motor`

3. 執行：

```powershell
.\start_console.ps1
```

4. 用 Edge 或 Chrome 開：

```text
http://127.0.0.1:8765
```

### C. 連線

1. 按 `Connect`
2. 選 STM32 的序列埠
3. 確認右上角變成 `Connected`
4. 按 `Read Config`
5. 確認頁面已讀回每軸參數

### D. 安全測試順序

第一次測試時，建議先用這組保守設定：

- `Ki = 0`
- `backlash = 0`
- `holdPwm = 0`
- `holdMs = 0`
- `maxPwm = 15 ~ 20`

先做：

1. 單軸測試
2. 小 target
3. 看 encoder 方向是否正確
4. 看 `Current PWM` 是否一直撞到 `Max PWM`

### E. encoder 記錄

1. 按 `Start Log`
2. 做你的 target 測試或軌跡播放
3. 按 `Stop Log`
4. 按 `Export CSV`

### F. 軌跡播放

1. 在 `Trajectory` 區塊選：

   `D:\Codex\Control\Yuhan_motor\example_trajectory.txt`

2. 設定 `Playback Interval`
3. 按 `Play Trajectory`

---

## 通訊格式

### 1. 送給 STM32 的 target frame

頁面送給 STM32 的 target 不是裸 20 bytes，而是：

- header: `YH`
- payload: `5 x int32 little-endian`

總長度：

- `2 bytes header`
- `20 bytes payload`

合計：

- `22 bytes`

### 2. STM32 支援的文字命令

```text
!GETALL
!GET,0
!PID,0,0.30,0.00,0.18
!HOLD,0,0,0,30
!BACKLASH,0,0
!PWM,0,30
!ZEROENC
```

### 3. STM32 回傳的資料

#### Encoder

```text
ENC: 0=123 1=456 2=789 3=10 4=11
```

#### 設定值

```text
CFG,0,0.3000,0.0000,0.1800,0,0,30,0,40
```

欄位順序：

- motor
- kp
- ki
- kd
- holdPwm
- holdMs
- deadband
- backlash
- maxPwm

#### PWM 狀態

```text
PWM: 0=12/40 1=0/40 2=-5/35 3=0/35 4=0/25
```

`PWM:` 這一行的格式是：

- 左邊：目前實際輸出 PWM
- 右邊：目前限制的最大 PWM

例如：

```text
0=12/40
```

代表：

- 第 0 軸目前實際輸出 `12`
- 第 0 軸最大允許輸出 `40`

---

## `hold` 是什麼

這是目前最重要的概念。

### 一句話版本

`hold` 的意思是：

**「到位置後，持續用一點小力撐住，不要讓機構自己滑掉。」**

### 為什麼需要 hold

如果沒有 `hold`：

1. 馬達移動到目標附近
2. 控制器覺得「差不多到了」
3. 輸出變得很小，甚至變成 0
4. 但如果機構有：
   - 重力
   - 拉力
   - 彈性
   - 外力
5. 關節就會慢慢掉下來

所以 `hold` 的工作不是「把它拉到目標」，而是：

**「已經到目標後，幫你撐住。」**

### `hold` 和 PID 的差別

可以把控制分成兩個階段：

#### 階段 1：移動到目標

- 誤差大
- 主要靠 PID
- 目標是把位置拉近

#### 階段 2：留在目標附近

- 誤差已經很小
- target 也沒有再變
- 改用小的固定支撐力
- 這就是 `hold`

所以：

- `PID`：負責「到目標」
- `hold`：負責「留在目標」

### 現在這版韌體裡的 `hold` 行為

在目前 `Yuhan_motor` 的韌體中：

1. 當誤差已經進入 `deadband`
2. 而且 target 已經穩定一小段時間
3. 就進入 hold 模式
4. 此時不再用大的 PID 輸出亂打
5. 改成用小輸出支撐負載

這樣做的目的，是避免：

- 到點後還一直大力修正
- 造成來回擺動
- 但又能保留足夠力氣撐住位置

---

## `holdPwm`、`holdMs`、`deadband` 各代表什麼

### `holdPwm`

意思是：

**進入 hold 後，要用多大的小輸出撐住位置。**

例如：

- `holdPwm = 0`：完全不撐
- `holdPwm = 2`：給一點很小的支撐力
- `holdPwm = 5`：給更大的支撐力

太小會：

- hold 不住
- 到位後慢慢滑掉

太大會：

- 開始抖
- 在目標附近來回推
- 發熱變高

所以 `holdPwm` 要找的是：

**「剛好夠撐住的最小值」**

### `holdMs`

意思是：

**進入穩定區後，要等多久才開始持續 hold。**

例如：

- `holdMs = 0`：一進入穩定區就開始 hold
- `holdMs = 20`：等 20 ms 再開始 hold

通常：

- 想快點撐住：設 `0`
- 想保守一點：可設 `20`

### `deadband`

意思是：

**誤差小到什麼程度，才算已經到位。**

只有誤差夠小，系統才會從「移動模式」切到「hold 模式」。

如果 `deadband` 太小：

- 很難進入 hold
- 容易一直在到位邊緣抖動

如果 `deadband` 太大：

- 還沒真的到位就提早進 hold
- 精度會下降

---

## `hold` 和 `maxPwm` 的關係

這點很重要。

`maxPwm` 是整體安全上限，表示：

**控制器最多只能輸出多大的 PWM。**

所以：

- `holdPwm` 必須小於或等於 `maxPwm`

如果：

- `holdPwm = 5`
- `maxPwm = 3`

那實際上最多只會輸出到 `3`，不是 `5`

所以當你覺得：

> 為什麼我明明開了 hold，還是撐不住？

要先檢查：

1. `holdPwm` 有沒有太小
2. `maxPwm` 有沒有把它卡住

---

## 調參建議

### 情況 1：會來回擺動

先這樣試：

1. `backlash = 0`
2. `holdPwm = 0`
3. `holdMs = 0`
4. `Ki = 0`
5. `maxPwm = 20`
6. 降低 `Kp`
7. 再小幅增加 `Kd`

### 情況 2：到位後會慢慢滑掉

先這樣試：

1. `holdPwm = 2`
2. `holdMs = 0`
3. `maxPwm >= holdPwm`
4. 如果還 hold 不住，再慢慢加到 `3`、`5`
5. 如果開始抖，就把 `holdPwm` 往回降

### 情況 3：一開始就爆衝

這時先不要碰 hold，先處理：

1. 檢查方向是不是錯
2. `Ki = 0`
3. `backlash = 0`
4. `maxPwm = 10 ~ 15`
5. `holdPwm = 0`
6. 單軸小步測試

---

## count、角度、encoder 的關係

目前頁面送的是 **raw count**，不是角度。

也就是說：

- 你現在輸入 `500`
- 代表的是 `500 count`
- 不是 `500 度`

角度和 encoder count 的關係通常是：

```text
角度 = (encoder_count - zero_offset) / counts_per_degree
```

但每一軸的 `counts_per_degree` 可能不同，所以不能直接把所有軸都當成同一個角度比例。

---

## CSV 會存哪些欄位

- `iso_time`
- `elapsed_ms`
- `source`
- `raw_line`
- `target_0`
- `target_1`
- `target_2`
- `target_3`
- `target_4`
- `encoder_0`
- `encoder_1`
- `encoder_2`
- `encoder_3`
- `encoder_4`
- `pwm_0`
- `pwm_1`
- `pwm_2`
- `pwm_3`
- `pwm_4`

所以你之後分析時，可以同時看：

- target
- encoder
- 誤差
- PWM 輸出

---

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

---

## 修改範圍

這次所有修改都只在：

- `D:\Codex\Control\Yuhan_motor`

沒有去動其他資料夾的程式。
