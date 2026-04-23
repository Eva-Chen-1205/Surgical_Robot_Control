# Surgical Robot Control 專案檔案分析與中文版使用手冊

## 1. 專案總覽

這個資料夾目前可以分成 **兩套彼此相關、但用途不同的控制系統**：

1. `根目錄` 的現行 STM32 馬達控制台
   用途：透過瀏覽器與 Web Serial 直接控制 5 軸馬達。
2. `Control_panel/` 的舊版研究／驗證流程
   用途：輸入姿態座標後做逆運動學、輸出軌跡、再用正運動學驗證，並可把角度資料轉送到 Unity。
3. `firmware/` 的 STM32 韌體程式
   用途：實際執行 5 軸閉迴路控制、讀取 encoder、輸出 PWM。

如果以「實際操作硬體」來看，目前最重要、最實用的是：

`根目錄控制台 + firmware 韌體`

## 2. 系統架構說明

### 目前主系統

`index.html` + `app.js` + `styles.css`
作用：瀏覽器控制介面，負責序列埠連線、送出 target、播放 trajectory、調整 PID、監看 encoder、匯出 CSV。

`server.py`
作用：提供本機網頁伺服器，讓瀏覽器能穩定開啟控制頁面，並避免快取問題。

`firmware/YuhanSTM32Control/*.ino|*.cpp|*.h`
作用：燒錄到 STM32 後執行實際控制，接收 PC 傳來的 target frame 與文字命令，回傳 encoder 與 PWM 狀態。

### 舊版研究流程

`Control_panel/index.html`
作用：輸入姿態參數（yaw/pitch/Z/Y/X），做逆運動學計算，並可產生軌跡。

`Control_panel/ik_server.cpp`
作用：C++ WebSocket 伺服器，接收姿態輸入，轉成 `theta1~theta5`，並寫出關節軌跡檔。

`Control_panel/fk_test.cpp`
作用：讀取關節軌跡，再用數值法解回姿態，驗證逆運動學結果是否合理。

`Control_panel/bridge_server.py`
作用：把控制器瀏覽器送出的 theta 資料廣播給 Unity 端顯示或使用。

## 3. 檔案逐一分析

### 根目錄檔案

| 檔案 | 類型 | 功能與用途 | 備註 |
| --- | --- | --- | --- |
| `.gitignore` | 設定檔 | 忽略 Python 快取資料夾與 `.pyc` 檔。 | 只處理根目錄 Python 快取。 |
| `README.md` | 說明文件 | 目前主控制台的操作說明，包含序列通訊、PID 調整、角度輸入、trajectory 格式與安全提醒。 | 內容很重要，但內文仍有舊路徑 `Yuhan_motor`。 |
| `index.html` | 前端入口 | 定義目前主控制台 UI 結構，包括序列設定、trajectory 區、5 軸 motor card、角度校正、PID 表格、encoder 監看與 serial log。 | 由 `server.py` 提供給瀏覽器載入。 |
| `styles.css` | 前端樣式 | 控制整體版面、深色主題、按鈕、表格、motor card 與響應式顯示。 | 純顯示層。 |
| `app.js` | 前端主程式 | 整個控制台的核心邏輯。負責 Web Serial、binary frame 傳送、文字命令傳送、encoder/PWM/CFG 解析、PID preset、角度換算、trajectory 播放、記錄與匯出。 | 這是 PC 端最核心的程式。 |
| `server.py` | Python 工具 | 在 `127.0.0.1:8765` 啟動本機 HTTP server，並關閉快取。 | 用來安全開啟控制頁面。 |
| `start_console.ps1` | 啟動腳本 | 切到目前資料夾後，執行 `python .\server.py`。 | 最方便的啟動方式。 |
| `generate_motor5_corrected_data.py` | 分析腳本 | 讀取 encoder log CSV，針對 motor 5 目前 `/100` 的 encoder 縮放做補正，輸出新的分析 CSV。 | 屬於實驗分析工具，非日常控制必備。 |

### `Control_panel/` 資料夾

| 檔案 | 類型 | 功能與用途 | 備註 |
| --- | --- | --- | --- |
| `Control_panel/.gitignore` | 設定檔 | 忽略 `.exe`、`.o`、`build/`、trajectory 文字檔與 `.vscode`。 | 主要是忽略編譯與輸出產物。 |
| `Control_panel/index.html` | 舊版前端 | 以姿態空間操作機械手，送 pose 到 `ik_server.exe`，接收 theta，並可輸出軌跡或轉送給 bridge server。 | 是另一套舊控制流程，與根目錄 `index.html` 不同。 |
| `Control_panel/bridge_server.py` | Python 橋接伺服器 | 透過 WebSocket 提供 `/controller`、`/unity`、`/save` 路徑，把瀏覽器控制資料轉送到 Unity。也會快取最後一筆 theta 給後加入的 Unity client。 | 程式邏輯可用，但終端輸出文字部分有亂碼。 |
| `Control_panel/ik_server.cpp` | C++ 伺服器原始碼 | 手動實作 WebSocket server，接收姿態陣列，做 IK 計算，輸出 `theta1~theta5`，並寫入 `ik_trajectory_*.txt`。 | 也支援 `SAVE:<檔名>|<內容>` 直接存檔。 |
| `Control_panel/ik_server.exe` | 編譯後執行檔 | `ik_server.cpp` 編譯後的程式。 | 可由原始碼重新編譯。 |
| `Control_panel/fk_test.cpp` | C++ 驗證原始碼 | 讀取 IK 產生的關節軌跡，用數值法做 FK，回推姿態，輸出 `fk_ik_trajectory_*.txt`。 | 用來驗證 IK 軌跡是否合理。 |
| `Control_panel/fk_test.exe` | 編譯後執行檔 | `fk_test.cpp` 編譯後的程式。 | 可由原始碼重新編譯。 |
| `Control_panel/run_all.bat` | 啟動批次檔 | 先編譯 `ik_server.cpp` 與 `fk_test.cpp`，再同時啟動 IK server、FK test 與 bridge server。 | 是舊流程的一鍵啟動入口。 |
| `Control_panel/unity_test.html` | 測試頁 | 連到 `ws://<IP>:9090/unity`，顯示 theta 值、更新速率、資料 log，並附 Unity C# 範例程式。 | 很適合 Unity 端測試與展示。 |
| `Control_panel/User_Manual.md` | 使用手冊 | 舊版 `Control_panel` 流程的操作文件，內容涵蓋 web UI、IK server、FK 驗證、bridge server、Unity monitor。 | 目前檔案有編碼亂碼問題，但仍可辨識它原本的用途。 |
| `Control_panel/Plot_Trajectory.m` | MATLAB 腳本 | 讓使用者選資料夾，自動找最新 `return_trajectory_*.txt`，並畫出 3D 軌跡、姿態曲線、關節曲線。 | 用於離線分析。 |
| `Control_panel/TestPlot.m` | MATLAB 腳本 | 與 `Plot_Trajectory.m` 類似，但預設直接使用目前目錄。 | 比較方便做快速測試。 |
| `Control_panel/Unity)` | 雜項殘留檔 | 內容只剩一小段字串：`[Bridge Server] Port 9090 (browser -`。 | 看起來像誤存或殘留檔，沒有實際功能。 |
| `Control_panel/ik_trajectory_20260422_211945.txt` | 輸出資料 | 由 `ik_server` 產生的關節軌跡檔，每行 5 個關節值。 | 共 45,953 行，屬於產物。 |
| `Control_panel/fk_ik_trajectory_20260422_211945.txt` | 驗證資料 | `fk_test` 根據 IK 軌跡回算出的姿態軌跡。 | 同樣 45,953 行，用來做比對。 |
| `Control_panel/return_trajectory_motor_20260422_220814.txt` | 輸出資料 | 瀏覽器 `smoothReturn()` 功能輸出的 motor 軌跡。 | 共 9,240 行，tab 分隔。 |
| `Control_panel/return_trajectory_motor_20260422_220825.txt` | 輸出資料 | 同類型 motor 軌跡。 | 另一組測試結果。 |
| `Control_panel/return_trajectory_motor_20260422_221149.txt` | 輸出資料 | 同類型 motor 軌跡。 | 另一組測試結果。 |
| `Control_panel/return_trajectory_motor_20260422_221343.txt` | 輸出資料 | 同類型 motor 軌跡。 | 另一組測試結果。 |
| `Control_panel/return_trajectory_pose_20260422_220814.txt` | 輸出資料 | 與上面同時間戳對應的 pose 軌跡。 | 共 9,240 行，欄位為 yaw/pitch/Z/Y/X。 |
| `Control_panel/return_trajectory_pose_20260422_220825.txt` | 輸出資料 | 同類型 pose 軌跡。 | 另一組測試結果。 |
| `Control_panel/return_trajectory_pose_20260422_221149.txt` | 輸出資料 | 同類型 pose 軌跡。 | 另一組測試結果。 |
| `Control_panel/return_trajectory_pose_20260422_221343.txt` | 輸出資料 | 同類型 pose 軌跡。 | 另一組測試結果。 |

### `Control_panel/.vscode/`

| 檔案 | 類型 | 功能與用途 | 備註 |
| --- | --- | --- | --- |
| `Control_panel/.vscode/c_cpp_properties.json` | IDE 設定 | 設定 VS Code 的 GCC 路徑與 IntelliSense include path。 | 使用 `C:/msys64/ucrt64/bin/gcc.exe`。 |
| `Control_panel/.vscode/launch.json` | IDE 設定 | 設定 C/C++ debug session。 | 路徑仍指向另一個舊專案位置，屬於過期設定。 |
| `Control_panel/.vscode/settings.json` | IDE 設定 | C/C++ Runner 編譯器、warning、搜尋路徑與 debugger 設定。 | 只在 VS Code 環境中有用。 |

### `Control_panel/Kinematics_Simple/`

| 檔案 | 類型 | 功能與用途 | 備註 |
| --- | --- | --- | --- |
| `Control_panel/Kinematics_Simple/README.md` | 說明文件 | 簡單介紹獨立 kinematics 工具的用途與使用方式。 | 英文內容清楚。 |
| `Control_panel/Kinematics_Simple/build.bat` | 編譯腳本 | 用 `g++ -O3` 編譯 `kinematics_tool.cpp`。 | 用來產生獨立 CLI 工具。 |
| `Control_panel/Kinematics_Simple/kinematics_tool.cpp` | C++ 原始碼 | 提供 CLI 介面，可做 Pose -> Joint 的 IK，或 Joint -> Pose 的 FK。 | 適合快速做數學驗證，不必啟動 WebSocket。 |
| `Control_panel/Kinematics_Simple/build/Debug/kinematics_tool.o` | 物件檔 | 編譯過程中的中間產物。 | 非原始碼。 |
| `Control_panel/Kinematics_Simple/build/Debug/outDebug.exe` | Debug 執行檔 | IDE 或 build 工具產生的 debug 版可執行檔。 | 非原始碼。 |

### `firmware/` 韌體資料夾

| 檔案 | 類型 | 功能與用途 | 備註 |
| --- | --- | --- | --- |
| `firmware/README.md` | 說明文件 | 韌體端的通訊協定、腳位配置與調校提示。 | 目前有編碼亂碼，而且仍引用舊路徑名稱。 |
| `firmware/YuhanSTM32Control/YuhanSTM32Control.ino` | 韌體主程式 | STM32 主入口。初始化 encoder 與 controller，接收 `YH + 5 x int32` frame，解析文字命令，以 200 Hz 更新控制，並定期印出 encoder 與 PWM。 | 韌體最核心的檔案。 |
| `firmware/YuhanSTM32Control/encoder_manager.h` | 標頭檔 | 宣告 encoder 初始化、讀值、反相、歸零等 API。 | 給 `.ino` 與 `finger.cpp` 使用。 |
| `firmware/YuhanSTM32Control/encoder_manager.cpp` | 韌體模組 | 管理 5 組 encoder 中斷讀值，且 motor 5 有特別的 `/100` 縮放機制。 | 這直接影響角度換算與 log 補正。 |
| `firmware/YuhanSTM32Control/motor.h` | 標頭檔 | 宣告 `Motor` 類別，管理 H-bridge PWM 與方向控制。 | 底層輸出抽象。 |
| `firmware/YuhanSTM32Control/motor.cpp` | 韌體模組 | 實作 PWM deadband、方向反轉、正反轉與停止。 | 負責真正輸出到馬達腳位。 |
| `firmware/YuhanSTM32Control/finger.h` | 標頭檔 | 宣告 `FingerController`，也宣告 `sendParallelPair`。 | 每軸控制器的主要介面。 |
| `firmware/YuhanSTM32Control/finger.cpp` | 韌體模組 | 實作 PID、integral 限制、hold 模式、backlash 補償、deadband 與雙軸同步控制。 | 韌體控制邏輯核心。 |

### 快取與產物

| 檔案 | 類型 | 功能與用途 | 備註 |
| --- | --- | --- | --- |
| `__pycache__/server.cpython-312.pyc` | Python 快取 | `server.py` 的編譯快取。 | 自動產生，可忽略。 |

## 4. 核心程式實際負責什麼

### `app.js` 的主要責任

- 建立 5 軸 motor card、PID 表格與角度校正表格
- 透過 Web Serial 與 STM32 連線
- 送出 binary target frame：`YH + 5 x int32 little-endian`
- 送出文字指令：
  - `!PID`
  - `!HOLD`
  - `!BACKLASH`
  - `!PWM`
  - `!MOTORINV`
  - `!ENCINV`
  - `!ZEROENC`
- 解析韌體回傳：
  - `CFG,...`
  - `PWM: ...`
  - `ENC: ...`
- 在瀏覽器 `localStorage` 中保存 PID preset 與角度校正值
- 讀取 trajectory 檔，並支援 count / degree / radian 轉換
- 匯出 CSV 與 raw serial text log

### `YuhanSTM32Control.ino` 的主要責任

- 建立 5 個 `FingerController`
- 同時支援兩種接收模式：
  - binary target frame
  - text command
- 以 `CONTROL_US = 5000` 執行 200 Hz 控制迴圈
- 每 `50 ms` 輸出 encoder 與 PWM 狀態
- 預設將 motor 3 與 motor 4 的同步補償關閉，方便單軸除錯

### `finger.cpp` 的控制特性

- 使用 PID 計算馬達輸出
- integral 有上限，避免積分暴衝
- hold 模式可在接近目標時給一點小 PWM 撐住位置
- backlash 補償只在 target 反向切換時才介入
- 每軸輸出都會被 `maxPwm` 限制

## 5. 目前主系統的使用方式

### 目標

透過瀏覽器控制 STM32 的 5 軸馬達，讀取 encoder / PWM，調整 PID，並播放 trajectory。

### 第 1 步：燒錄韌體

1. 開啟 Arduino IDE 2.x。
2. 安裝 STM32 Arduino Core。
3. 開啟：
   `firmware/YuhanSTM32Control/YuhanSTM32Control.ino`
4. 選對 STM32 板子、COM Port、Upload Method。
5. 按 `Verify`。
6. 按 `Upload`。

### 第 2 步：啟動本機控制頁伺服器

在專案根目錄執行：

```powershell
.\start_console.ps1
```

或直接手動執行：

```powershell
python .\server.py
```

接著用瀏覽器開：

```text
http://127.0.0.1:8765
```

請使用 **Edge** 或 **Chrome**，因為頁面使用 Web Serial。

### 第 3 步：連線到 STM32

1. 按 `Connect`
2. 選擇 STM32 對應的序列埠
3. 按 `Read Config`
4. 確認頁面狀態變成 `Connected`

### 第 4 步：第一次安全測試建議

建議先用這組保守設定：

- `Ki = 0`
- `backlash = 0`
- `holdPwm = 0`
- `holdMs = 0`
- `maxPwm` 先設小一點

第一次動作建議：

1. 只測一軸
2. 只送很小的 target，例如 `100` 或 `-100`
3. 觀察 encoder 是否往 target 靠近
4. 觀察 PWM 是否長時間卡在上限
5. 如果方向錯，先調 `motorInvert` 或 `encoderInvert`，不要急著加大 PID

### 第 5 步：角度模式

如果你不想直接用 raw count，而想用角度：

1. 在 `Angle Calibration` 填好每一軸的 `Counts / Degree`
2. 按 `Save Angle Config`
3. 把 `Target Unit` 切到 `Degree` 或 `Radian`

前端會自動先換算成 count 再送給 STM32。

### 第 6 步：播放 trajectory

1. 準備 `.txt` 或 `.csv` 檔
2. 每行必須剛好 **5 個數字**
3. 不要有表頭
4. 選對 `Trajectory Unit`
5. 設定 `Playback Interval (ms)`
6. 按 `Play Trajectory`

格式範例：

```text
0,0,0,0,0
100,120,50,-30,0
200,180,80,-60,0
```

或：

```text
0 0 0 0 0
1.5 2.0 0.8 -0.5 0
3.0 3.5 1.2 -1.0 0
```

### 第 7 步：記錄與匯出

1. 測試前先按 `Start Log`
2. 執行控制或 trajectory
3. 按 `Stop Log`
4. 匯出：
   - `Export CSV`
   - `Export Raw TXT`

CSV 中包含：

- target
- encoder
- PWM
- timestamp
- raw line

## 6. `Control_panel` 舊流程怎麼使用

這套流程比較適合：

- 做姿態空間規劃
- 做 IK/FK 驗證
- 跟 Unity 串接展示

### 一鍵啟動

進入 `Control_panel/` 後執行：

```bat
run_all.bat
```

它會依序做三件事：

1. 編譯 `ik_server.cpp`
2. 編譯 `fk_test.cpp`
3. 啟動：
   - `ik_server.exe`
   - `fk_test.exe`
   - `python bridge_server.py`

### 控制器頁面

開啟：

```text
Control_panel/index.html
```

它的行為是：

- 連到 `ws://localhost:8080`
- 傳送 pose 陣列 `[yaw, pitch, z, y, x]`
- 從 `ik_server` 收到 theta JSON
- 可轉送 theta JSON 到 bridge server
- 可要求 `ik_server` 直接存檔

### Unity 轉接

1. 啟動 `bridge_server.py`
2. 看它印出的區域網路 IP
3. 控制器端 bridge URL：

```text
ws://<LAN-IP>:9090/controller
```

4. Unity 端連線 URL：

```text
ws://<LAN-IP>:9090/unity
```

### FK 驗證

`fk_test.exe` 會持續讀取 IK 軌跡檔，並產生驗證輸出：

- 輸入：`ik_trajectory_*.txt`
- 輸出：`fk_ik_trajectory_*.txt`

這樣可以檢查 IK 產生的關節軌跡，回推後是否接近原本的姿態路徑。

### MATLAB 畫圖

可使用：

- `Plot_Trajectory.m`
- `TestPlot.m`

來畫出：

- 3D 末端路徑
- yaw / pitch 曲線
- 各關節角度曲線

## 7. 通訊協定整理

### Binary target frame

- header：`YH`
- payload：`5 x int32 little-endian`
- 總長：`22 bytes`

### 韌體可接受的文字命令

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

### 韌體常見回傳格式

```text
ENC: 0=123 1=456 2=789 3=10 4=11
CFG,0,0.3000,0.0000,0.1800,0,0,30,0,40,0,1
PWM: 0=12/40 1=0/40 2=-5/35 3=0/20 4=0/25
```

## 8. 這個專案目前的重要觀察

### 真正與硬體控制直接相關的檔案

以下這些是目前最應該保留、理解、維護的：

- `index.html`
- `styles.css`
- `app.js`
- `server.py`
- `start_console.ps1`
- `firmware/YuhanSTM32Control/YuhanSTM32Control.ino`
- `firmware/YuhanSTM32Control/encoder_manager.*`
- `firmware/YuhanSTM32Control/motor.*`
- `firmware/YuhanSTM32Control/finger.*`

### 比較偏舊版或實驗用途的部分

- `Control_panel/` 整包
- `generate_motor5_corrected_data.py`
- 各種 `trajectory_*.txt`
- `.exe`
- `.o`
- `.pyc`

### 目前已看出的問題

1. 多份文件仍然引用舊專案名稱 `Yuhan_motor`
2. `Control_panel/User_Manual.md` 與 `firmware/README.md` 有編碼亂碼
3. `Control_panel/.vscode/launch.json` 指向舊的絕對路徑，已過期
4. `Control_panel/Unity)` 看起來是殘留垃圾檔
5. 專案內目前放了不少產物檔，對版本控管不太友善

## 9. 建議整理方向

如果之後你想把專案整理乾淨，建議順序如下：

1. 先把根目錄控制台 + firmware 定義成主系統
2. 再決定 `Control_panel/` 是否還要保留
3. 修復 `Control_panel/User_Manual.md` 與 `firmware/README.md` 的編碼
4. 視需要移除或封存產物：
   - `*.exe`
   - `*.o`
   - `*.pyc`
   - `*trajectory_*.txt`
5. 確認 `Control_panel/Unity)` 是否可刪除

## 10. 快速啟動清單

### 如果你的目標是直接控制硬體

1. 燒錄 `firmware/YuhanSTM32Control/YuhanSTM32Control.ino`
2. 執行 `.\start_console.ps1`
3. 開啟 `http://127.0.0.1:8765`
4. 連上 serial
5. 按 `Read Config`
6. 先做單軸小幅測試
7. 正式動作前先按 `Start Log`

### 如果你的目標是做 IK / FK / Unity 驗證

1. 進入 `Control_panel`
2. 執行 `run_all.bat`
3. 開啟 `Control_panel/index.html`
4. 若需要 Unity 顯示，讓 Unity 連 `ws://<LAN-IP>:9090/unity`

