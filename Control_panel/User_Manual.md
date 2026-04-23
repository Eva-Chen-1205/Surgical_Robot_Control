# 機器人手臂控制系統 操作手冊 (User Manual)

本手冊詳述了此專案中所有程式的功能與使用方法。

---

## 1. 系統總覽 (System Overview)
本系統主要由以下部分組成：
- **Web 控制面板 (`index.html`)**: 提供搖桿、拉桿以及軌跡生成功能的網頁界面。
- **IK 伺服器 (`ik_server.exe`)**: 背景運行的 C++ WebSocket 伺服器，負責計算逆向運動學並儲存軌跡。
- **FK 驗證工具 (`fk_test.exe`)**: 用於讀取軌跡檔並透過正向運動學驗證準確性的工具。
- **Bridge 中繼伺服器 (`bridge_server.py`)**: WebSocket 中繼伺服器，將 θ1~θ5 數據即時轉發給 Unity 數位雙生端。
- **Unity 測試頁面 (`unity_test.html`)**: 給同學驗證連線並可視化 θ 數據的測試工具。
- **獨立運動學工具 (`Kinematics_Simple/`)**: 一個獨立的、簡單輸入即可獲得答案的 IK/FK 轉換小工具。

---

## 2. Web 控制面板 (`index.html`)
開啟方式：直接使用瀏覽器開啟 `index.html`。

### 2.1 手動控制模式 (Manual Mode)
- **搖桿 (Joystick)**: 控制 Yaw (偏航角) 與 Pitch (俯仰角)。
- **點位拉桿 (Sliders)**: 手動調整 X, Y, Z 軸的座標。
- **保護開關**: 位於頁面頂部，切換後即可進入軌跡生成模式。

### 2.2 軌跡生成模式 (Trajectory Mode)
- **起點 (START)**: 自動同步當前手臂位置。
- **終點 (End Points)**: 可自行「+ 新增終點」，每個終點可輸入空間座標參數。
- **匯出軌跡**: 點擊「Generate & Export Trajectory」後，程式會自動內插平滑點位並傳送至伺服器存檔。

### 2.3 Bridge 連線（底部面板）
- **功能**: 將 IK 計算結果 (θ1~θ5) 即時轉發給 Unity 數位雙生。
- **使用方式**:
  1. 先啟動 `bridge_server.py`
  2. 在底部面板輸入 Bridge Server 地址（例如 `ws://192.168.x.x:9090/controller`）
  3. 點擊 **Connect**
  4. 連線成功後，所有 θ 數據會自動轉發

---

## 3. IK 伺服器 (`ik_server.exe`)
- **功能**: 接收來自網頁的點位請求，計算 5 個電機的轉角 (θ1-θ5)，並將結果儲存在 `ik_trajectory_YYYYMMDD_HHMMSS.txt` 中。
- **執行**: 確保在網頁操作前先執行此程式，或透過 `run_all.bat` 一鍵開啟。

---

## 4. FK 驗證工具 (`fk_test.exe`)
- **功能**: 自動讀取最新的 IK 軌跡檔，計算其對應的正向運動學位置，以便與原始設計進行比對。
- **輸出**: 生成 `fk_ik_trajectory_*.txt` 並在終端機顯示實時計算結果。

---

## 5. Bridge 中繼伺服器 (`bridge_server.py`)
- **功能**: 作為 WebSocket 中繼站，將你的控制端 θ1~θ5 即時轉發給同學的 Unity 數位雙生。
- **安裝依賴**: `pip install websockets`
- **啟動**: `python bridge_server.py`
- **連線方式**:
  - 控制端 (你的瀏覽器): `ws://<你的IP>:9090/controller`
  - Unity 端 (同學的 Unity): `ws://<你的IP>:9090/unity`
- **數據格式 (JSON)**:
  ```json
  {"theta1": 0.1234, "theta2": -0.5678, "theta3": 0.0012, "theta4": 0.3456, "theta5": 1.2345}
  ```

---

## 6. Unity 測試頁面 (`unity_test.html`)
- **功能**: 給同學在整合 Unity 之前，先用瀏覽器測試連線是否正常。
- **使用方式**: 在瀏覽器開啟此頁面，輸入 `ws://<IP>:9090/unity`，點擊 Connect。
- **內建功能**: 即時 θ 值可視化、統計面板、Unity C# 整合程式碼範例。

---

## 7. 獨立簡易工具 (`Kinematics_Simple/`)
位於子目錄 `Kinematics_Simple` 中。

- **使用場景**: 當您只想快速知道「某組座標對應的轉角」或「某組轉角對應的座標」時。
- **操作步驟**:
  1. 執行 `build.bat` 完成編譯。
  2. 執行 `kinematics_tool.exe`。
  3. 選擇選單 1 (IK) 或 2 (FK)，按照提示輸入數值。

---

## 8. 快速啟動指南

### 8.1 一般使用（不含 Unity）
1. 點擊根目錄的 `run_all.bat` (會同時開啟伺服器與驗證程式)。
2. 開啟 `index.html`。
3. 確認網頁左下角顯示 **Connected**。
4. 開始操作手臂或生成軌跡！

### 8.2 與同學的 Unity 數位雙生連線
1. 點擊 `run_all.bat`（會自動啟動 Bridge Server）。
2. 記下 Bridge Server 啟動時顯示的 **你的 IP 地址**。
3. 開啟 `index.html`，在底部 Bridge 面板輸入 `ws://你的IP:9090/controller`。
4. 點擊 **Connect**。
5. 將 `ws://你的IP:9090/unity` 地址告訴同學。
6. 同學在 Unity 中連線到該地址即可即時接收 θ1~θ5。

### 注意事項
- 確保兩台電腦在**同一個區域網路 (LAN)** 內。
- 如果有防火牆，需開放 **Port 9090**。
- Windows 防火牆提示時請選擇「允許存取」。

---
*備註：所有生成的軌跡檔案皆會包含時間戳記，方便紀錄與回溯。*
