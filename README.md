# Yuhan Motor Console

這一包是獨立可用的控制系統，只需要保留：
- `D:\Codex\Control\Yuhan_motor\index.html`
- `D:\Codex\Control\Yuhan_motor\app.js`
- `D:\Codex\Control\Yuhan_motor\styles.css`
- `D:\Codex\Control\Yuhan_motor\server.py`
- `D:\Codex\Control\Yuhan_motor\start_console.ps1`
- `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\YuhanSTM32Control.ino`
- `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\encoder_manager.*`
- `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\motor.*`
- `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\finger.*`

這一版只依賴 `Yuhan_motor` 內的檔案，不需要再讀其他舊資料夾。

---

## 功能

### 電腦端頁面
- 連線 STM32 序列埠
- 送出 5 軸 target
- 單軸 `+/- step` jog
- 播放 trajectory 檔案
- 支援 `Raw Count` / `Degree` / `Radian` 輸入模式
- 顯示 encoder 值
- 顯示目前 PWM 輸出與 PWM 上限
- 匯出 encoder log 為 `.csv`
- 每軸調整：
  - `Kp`
  - `Ki`
  - `Kd`
  - `holdPwm`
  - `holdMs`
  - `deadband`
  - `backlash`
  - `maxPwm`
  - `motorInvert`
  - `encoderInvert`

### STM32 韌體
- 接收 5 軸 target frame
- 執行 5 軸位置控制
- 回傳 encoder 與 PWM
- 接收 PID / hold / backlash / max PWM / invert 參數更新
- 預設將第 3、4 軸同步補償關閉，方便單軸除錯

---

## 目錄

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

## 使用步驟

### 1. 燒錄 STM32

1. 開 Arduino IDE 2.x。
2. 安裝 STM32 Arduino Core。
3. 開啟：
   `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\YuhanSTM32Control.ino`
4. 選對 STM32 板子。
5. 選對 COM Port。
6. 選對 Upload Method。
7. 按 `Verify`。
8. 按 `Upload`。

### 2. 開控制頁

1. 用 PowerShell 進到：
   `D:\Codex\Control\Yuhan_motor`
2. 執行：

```powershell
.\start_console.ps1
```

3. 用 Edge 或 Chrome 開：

```text
http://127.0.0.1:8765
```

### 3. 連線

1. 按 `Connect`。
2. 選擇 STM32 的序列埠。
3. 按 `Read Config`。
4. 確認頁面顯示 `Connected`。

### 4. 如果要直接用角度

1. 到 `Angle Calibration` 填每軸 `Counts / Degree`
2. 按 `Save Angle Config`
3. 把 `Target Unit` 切到 `Degree`
4. 之後：
   - target 輸入框會改用角度
   - `Jog Step` 也會改用角度
   - 前端會自動換算成 count 再送給 STM32

### 5. 如果要直接用弧度

1. `Angle Calibration` 一樣要先填好每軸 `Counts / Degree`
2. 把 `Target Unit` 切到 `Radian`
3. 如果是軌跡檔，就把 `Trajectory Unit` 切到 `Radian`
4. 前端會先把弧度轉成角度，再換成 count 送給 STM32

---

## 最安全的第一次測試設定

先只測單軸，建議從第四軸開始時先用：
- `Ki = 0`
- `backlash = 0`
- `holdPwm = 0`
- `holdMs = 0`
- `maxPwm = 10 ~ 15`

然後：
1. 只改一個很小的 target，例如 `-100` 或 `100`。
2. 看 encoder 有沒有往你預期的方向變。
3. 看 `PWM` 是否長時間貼住上限，例如 `15 / 15` 或 `-15 / 15`。
4. 如果方向不對，先不要調大 `Kp`，先改 `motorInvert` 或 `encoderInvert`。

---

## 第四軸方向排查

如果你看到以下現象：
- 給負方向 target，但馬達往正方向跑
- encoder 數值一直往反方向變
- PWM 一直貼上限
- 馬達發出逼逼聲，像卡住或一直硬推

這通常不是單純 PID 太大，而是下面其中一個：
- `motorInvert` 方向錯
- `encoderInvert` 方向錯
- `maxPwm` 太大，方向還沒確定前就先撞上限

### 建議排查順序

1. 先把第四軸 `maxPwm` 設成 `10`。
2. `Kp` 先保守，例如 `0.03 ~ 0.08`。
3. `Ki = 0`、`Kd = 0` 也可以先試。
4. 只送一個小 target，例如 `-100`。
5. 觀察：
   - 如果 target 是負，encoder 也開始往負方向靠近，方向多半是對的。
   - 如果 target 是負，但 encoder 往正方向跑遠，通常是 `encoderInvert` 錯。
   - 如果 target 是負，但馬達明顯往相反方向轉，通常先試 `motorInvert`。

### `motorInvert` 與 `encoderInvert` 怎麼試

只改一個，不要兩個一起翻。

1. 先翻 `motorInvert`。
2. 再做一次小 target 測試。
3. 如果還是錯，再把 `motorInvert` 改回來，改試 `encoderInvert`。
4. 如果兩個都翻才正常，也可以這樣做，但要記錄下來。

### 什麼情況不要繼續硬推

如果看到以下任一情況，先停：
- PWM 一直卡在上限
- encoder 離 target 越來越遠
- 馬達持續逼逼叫
- 機構有明顯卡住或拉扯

這時先按 `Stop All`，再回來調 `invert` 或降低 `maxPwm`。

---

## 目前保存的 PID 基準

這是你單軸測試後，準備進入軌跡測試前的基準設定。
我已經把它們寫進：
- `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\YuhanSTM32Control.ino`
- `D:\Codex\Control\Yuhan_motor\app.js`

### Saved Baseline

| Motor | Kp | Ki | Kd | Hold PWM | Hold ms | Deadband | Backlash | Max PWM | Motor Inv | Encoder Inv |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 1 | 0.27 | 0 | 0.01 | 0 | 0 | 30 | 0 | 100 | off | off |
| 2 | 0.30 | 0.001 | 0.01 | 0 | 0 | 100 | 0 | 100 | off | off |
| 3 | 0.35 | 0 | 0.01 | 0 | 0 | 100 | 0 | 100 | off | off |
| 4 | 0.20 | 0 | 0.01 | 0 | 0 | 30 | 0 | 80 | on | off |
| 5 | 2.00 | 0 | 0 | 0 | 0 | 3 | 0 | 25 | off | off |

### 目前額外保留的條件

- 第 3、4 軸同步補償維持關閉
- `hold` 先全部關閉
- `backlash` 先全部關閉

如果之後頁面上的設定被改亂，只要重新燒錄韌體，再按一次 `Read Config`，就能回到這一組基準。

---

## PID 預設組存檔

頁面現在支援把目前的 PID 表格設定存成多組 preset，之後可以再讀回來。

### 能做什麼

- 存目前畫面上的 5 軸 PID / hold / backlash / PWM / invert 設定
- 一台電腦上保存多組名稱不同的 preset
- 之後把任一組 preset 載回表格
- 刪除不需要的 preset

### 使用方式

1. 先把表格中的數值調好
2. 在 `Preset Name` 輸入名稱
3. 按 `Save Current PID`
4. 之後如果要再叫回來：
   - 在 `Saved Presets` 選擇一組
   - 按 `Load Selected PID`
   - 再按 `Apply All` 送到 STM32

### 注意

- 這些 preset 目前是存在**瀏覽器 localStorage**
- 也就是說：
  - 同一台電腦、同一個瀏覽器，會保留
  - 換瀏覽器或清掉瀏覽器資料，preset 可能會消失
- 韌體內建 baseline 仍然保留在：
  - `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\YuhanSTM32Control.ino`

---

## 角度輸入與校正

頁面現在支援兩種輸入單位：
- `Raw Count`
- `Degree`
- `Radian`

STM32 韌體本身仍然只吃 **count**。
所以當你在頁面上切到 `Degree` 或 `Radian` 時，前端會先自動換算：

```text
count = degree * counts_per_degree
```

如果輸入是弧度，前端會先做：

```text
degree = radian * 180 / pi
```

### `Counts / Degree` 是什麼

這代表：

```text
1 度 = 幾個 encoder count
```

例如：
- 如果某軸 `Counts / Degree = 1500`
- 你輸入 `10 degree`
- 前端就會送：

```text
15000 count
```

### 怎麼設定

1. 在 `Angle Calibration` 表格中，逐軸填入 `Counts / Degree`
2. 按 `Save Angle Config`
3. 再把 `Target Unit` 或 `Trajectory Unit` 切到 `Degree`

### 目前內建的理論預設值

根據你提供的規格：
- encoder: `IE2-512`
- gear ratio: `546:1`
- 韌體目前前 4 軸採用 2x decoding

頁面現在一打開就先帶入：
- Motor 1 ~ 4: `1553.0667 counts/degree`
- Motor 5: `15.5307 counts/degree`

第 5 軸比較小，是因為目前韌體在
`D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\encoder_manager.cpp`
裡有 `/100` 的特殊縮放。

如果之後你改了 encoder 解碼方式，或第 5 軸縮放邏輯改掉，這些值也要一起更新。

### 存在哪裡

- 這些角度校正值目前存在瀏覽器 `localStorage`
- 同一台電腦、同一個瀏覽器會保留
- 換瀏覽器或清掉瀏覽器資料就可能消失

### 頁面會怎麼顯示

- motor card 的 target 可以直接輸入角度
- encoder 區塊會同時顯示：
  - 原始 count
  - 換算後的 degree

---

## 軌跡測試前建議

開始跑 trajectory 前，建議先做這 5 件事：

1. 重新燒錄：
   `D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\YuhanSTM32Control.ino`
2. 開頁面後先按 `Read Config`
3. 確認第 4 軸仍然是：
   - `Motor Inv = on`
   - `Encoder Inv = off`
4. 先按 `Start Log`
5. 第一輪 trajectory 先用比較慢的 `Playback Interval`，例如 `10 ms` 或 `20 ms`

### 第一輪 trajectory 建議

- 先用短軌跡
- 先不要跨太大角度
- 先不要一次跑很久
- 跑的時候盯著：
  - `Current PWM`
  - `Error`
  - 第 4 軸 encoder 是否開始往反方向飆

### 如果跑 trajectory 時出現這些狀況

- `PWM` 長時間貼上限
- encoder 離 target 越來越遠
- 馬達持續逼逼叫
- 機構明顯卡住

請立刻：
1. 按 `Stop All`
2. 按 `Stop Trajectory`
3. 匯出 CSV
4. 保留當次 log 來回看是哪一軸先出問題

---

## Trajectory 檔案格式

trajectory 檔目前支援：
- `.txt`
- `.csv`

### 基本規則

- 一行代表一個時間點
- 每行必須有 **5 個數字**
- 順序固定是：

```text
Motor1, Motor2, Motor3, Motor4, Motor5
```

- 可以用逗號或空白分隔
- 檔案內不要放表頭

### 如果 `Trajectory Unit = Raw Count`

每一行就是 count：

```text
0,0,0,0,0
100,120,50,-30,0
200,180,80,-60,0
```

### 如果 `Trajectory Unit = Degree`

每一行就是 degree：

```text
0,0,0,0,0
1.5,2.0,0.8,-0.5,0
3.0,3.5,1.2,-1.0,0
```

頁面會根據每軸的 `Counts / Degree` 自動換成 count。

### 時間怎麼決定

trajectory 檔本身沒有時間欄位。
每一行之間的時間由頁面上的：

- `Playback Interval (ms)`

決定。

### 你之前存的是弧度怎麼辦

目前頁面直接支援的是：
- `count`
- `degree`
- `radian`

如果你原本存的是弧度，先做其中一種：

1. 直接把 `Trajectory Unit` 切到 `Radian` 後載入
2. 或先把弧度轉成 degree 再載入
3. 或先把弧度轉成 count 再載入

弧度轉角度：

```text
degree = radian * 180 / pi
```

---

## `hold` 是什麼

`hold` 的概念是：

**到目標附近後，用一點小輸出把位置撐住，不要滑掉。**

但如果你的機構在不同角度受力差很多，固定的 `holdPwm` 不一定適合所有角度，所以目前可以先關掉：
- `holdPwm = 0`
- `holdMs = 0`

### `hold` 和 PID 的差別

- `PID`：把位置拉到目標。
- `hold`：到了目標附近後，用小力維持住。

### 為什麼目前可以先不用 `hold`

因為你現在比較需要先解決：
- 方向對不對
- 會不會 runaway
- 會不會爆衝

等基本閉迴路穩了，再看要不要做角度相關的補償或 hold。

---

## `backlash` 是什麼

`backlash` 是反向切換時的補償。

但在還沒把單軸方向、PID、PWM 上限穩住之前，建議先關掉：
- `backlash = 0`

不然它很容易讓你在目標附近看起來更抖、更難判斷真正問題。

---

## PWM 代表什麼

PWM 可以理解成馬達輸出的力道上限。

### `Current PWM`
- 目前控制器真正送出去的輸出。

### `Max PWM`
- 這一軸允許的最大輸出上限。
- 韌體最後會把輸出限制在 `-maxPwm ~ +maxPwm`。

### PWM 太大會怎樣
- 容易爆衝
- 方向錯時特別危險
- 容易發熱
- 容易在目標附近來回震盪

### PWM 太小會怎樣
- 推不動
- 只有逼逼聲，encoder 幾乎不動
- 克服不了靜摩擦

---

## 通訊格式

### 1. 電腦送到 STM32 的 target frame

頁面送的是：
- header: `YH`
- payload: `5 x int32 little-endian`

總長度：
- `2 bytes header`
- `20 bytes payload`
- 合計 `22 bytes`

### 2. STM32 可接收的文字命令

```text
!GETALL
!GET,0
!PID,0,0.30,0.00,0.18
!HOLD,0,0,0,30
!BACKLASH,0,0
!PWM,0,30
!MOTORINV,0,0
!ENCINV,0,1
!ZEROENC
```

### 3. STM32 回傳格式

#### Encoder

```text
ENC: 0=123 1=456 2=789 3=10 4=11
```

#### Config

```text
CFG,0,0.3000,0.0000,0.1800,0,0,30,0,40,0,1
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
- motorInvert
- encoderInvert

#### PWM

```text
PWM: 0=12/40 1=0/40 2=-5/35 3=0/20 4=0/25
```

例如：
- `3=0/20` 代表第四軸目前輸出是 `0`，上限是 `20`。

---

## count、角度、encoder 的關係

目前頁面送的是 **raw count**，不是角度。

關係通常是：

```text
角度 = (encoder_count - zero_offset) / counts_per_degree
```

或：

```text
encoder_count = zero_offset + 角度 * counts_per_degree
```

也就是說，每一軸都需要知道：

```text
1 度 = 幾個 encoder count
```

如果你還沒有標定，請先把現在的 target 當作 raw count 看待，不要直接當角度。

---

## CSV 會記錄什麼

匯出的 `.csv` 目前包含：
- `iso_time`
- `elapsed_ms`
- `source`
- `raw_line`
- `target_0 ~ target_4`
- `encoder_0 ~ encoder_4`
- `pwm_0 ~ pwm_4`

這樣之後可以分析：
- 目標
- encoder 追隨狀況
- PWM 是否常常撞上限
- 哪個時刻開始 runaway

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

## 目前這版針對你現在問題做的保護

- 第四軸預設 `maxPwm` 降到 `20`
- 第 3、4 軸同步補償先關閉，方便單軸除錯
- 頁面可直接調整 `motorInvert`
- 頁面可直接調整 `encoderInvert`
- 韌體會回報每軸 `Current PWM / Max PWM`

如果你現在要先解第四軸，我建議你這樣做：
1. 重燒：`D:\Codex\Control\Yuhan_motor\firmware\YuhanSTM32Control\YuhanSTM32Control.ino`
2. 頁面按 `Read Config`
3. 第四軸設：`Kp=0.03`、`Ki=0`、`Kd=0`、`maxPwm=10`
4. `holdPwm=0`、`holdMs=0`、`backlash=0`
5. 先送 `-100`
6. 如果方向不對，只翻一個：先試 `motorInvert`，再試 `encoderInvert`
7. 一旦看到 encoder 離 target 越跑越遠，就立刻 `Stop All`
