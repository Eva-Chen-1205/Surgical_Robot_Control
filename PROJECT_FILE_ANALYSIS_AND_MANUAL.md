# Surgical Robot Control: File Analysis and User Manual

## 1. Project Overview

This folder currently contains **two related but different control systems**:

1. `root/` current browser-based STM32 motor console
   Role: direct 5-axis motor control through Web Serial.
2. `Control_panel/` older research and verification workflow
   Role: pose input -> inverse kinematics -> trajectory export -> forward-kinematics verification -> optional Unity relay.
3. `firmware/` STM32 firmware source
   Role: actual low-level closed-loop control for 5 motors and 5 encoders.

The most practical day-to-day control path is the **root console + firmware** combination.

## 2. Architecture Summary

### Current main path

`index.html` + `app.js` + `styles.css`
Role: browser UI for serial connection, target sending, trajectory playback, PID tuning, encoder monitoring, and CSV export.

`server.py`
Role: lightweight local web server so the browser can load the console reliably without cache issues.

`firmware/YuhanSTM32Control/*.ino|*.cpp|*.h`
Role: runs on STM32, receives binary target frames and text configuration commands, executes PID control, and prints encoder/PWM status back to the PC.

### Legacy / research path

`Control_panel/index.html`
Role: pose-space control UI using yaw/pitch/Z/Y/X input, local IK math, trajectory generation, and bridge forwarding.

`Control_panel/ik_server.cpp`
Role: C++ WebSocket server on port `8080`; accepts pose arrays, converts them to joint angles, logs joint trajectories, and can save exported files from the browser.

`Control_panel/fk_test.cpp`
Role: reads joint trajectory files and numerically reconstructs pose trajectories for verification.

`Control_panel/bridge_server.py`
Role: relays theta JSON from the controller browser to Unity clients through WebSocket on port `9090`.

## 3. File-by-File Analysis

### Root folder

| File | Type | Function and purpose | Notes |
| --- | --- | --- | --- |
| `.gitignore` | config | Ignores Python cache and `.pyc` files. | Only covers Python cache in root. |
| `README.md` | document | Main usage guide for the current Yuhan motor console system. Explains serial protocol, PID tuning, angle mode, trajectory format, and safety notes. | Content is useful, but many paths still point to `D:\Codex\Control\Yuhan_motor`, not this current folder. |
| `index.html` | frontend entry | UI structure for the current motor console: serial controls, trajectory controls, motor cards, angle calibration, PID table, encoder monitor, and serial log. | Loaded by browser through `server.py`. |
| `styles.css` | frontend style | Defines dark dashboard look, responsive layout, tables, motor cards, and button styles. | Pure presentation layer. |
| `app.js` | frontend logic | Core client logic. Handles Web Serial connection, binary target frame sending, text command sending, encoder/config/PWM parsing, PID preset storage, angle conversion, trajectory playback, raw log capture, and CSV export. | This is the main PC-side control program. |
| `server.py` | Python utility | Starts a local HTTP server on `127.0.0.1:8765` and disables cache headers. | Needed because Web Serial pages are easier to run from a local server than direct `file://` access. |
| `start_console.ps1` | launcher script | Changes directory to the script location and runs `python .\server.py`. | Fastest way to open the current console server. |
| `generate_motor5_corrected_data.py` | analysis utility | Post-processes a logged CSV to compensate for the motor-5 `/100` encoder scaling currently used in firmware. Adds corrected counts and angle columns. | Experimental analysis script, not required for normal operation. |

### Control_panel folder

| File | Type | Function and purpose | Notes |
| --- | --- | --- | --- |
| `Control_panel/.gitignore` | config | Ignores compiled executables, object files, build folder, trajectory text outputs, and `.vscode`. | Appropriate for generated artifacts in this subproject. |
| `Control_panel/index.html` | legacy frontend | Browser UI for pose-space manual control and trajectory generation. Sends pose arrays to `ik_server.exe`, can forward joint JSON to `bridge_server.py`, and can export pose/motor trajectories. | A separate older control workflow, different from root `index.html`. |
| `Control_panel/bridge_server.py` | Python relay server | WebSocket bridge between browser controller and Unity viewers. Supports `/controller`, `/unity`, and `/save`. Also caches the last theta packet for late-joining Unity clients. | Console output has some mojibake, but logic is readable. Requires `websockets`. |
| `Control_panel/ik_server.cpp` | C++ server source | Implements inverse kinematics and a manual WebSocket server. Receives pose arrays from browser, computes `theta1~theta5`, writes `ik_trajectory_*.txt`, and returns theta JSON to the browser. | Also supports `SAVE:<filename>|<content>` command for direct file export. |
| `Control_panel/ik_server.exe` | compiled binary | Built executable of `ik_server.cpp`. | Generated artifact; can be rebuilt from source. |
| `Control_panel/fk_test.cpp` | C++ verification source | Reads IK trajectory output and solves numerical forward kinematics to reconstruct the pose path, writing `fk_ik_trajectory_*.txt`. | Used to verify that IK trajectory maps back to expected pose path. |
| `Control_panel/fk_test.exe` | compiled binary | Built executable of `fk_test.cpp`. | Generated artifact; can be rebuilt from source. |
| `Control_panel/run_all.bat` | launcher script | Compiles `ik_server.cpp` and `fk_test.cpp`, then starts IK server, FK test, and bridge server in separate terminal windows. | Useful one-click startup for the old workflow. |
| `Control_panel/unity_test.html` | debug dashboard | Web page for testing Unity relay data. Connects to `ws://<IP>:9090/unity`, displays theta values, rates, raw log, and includes sample Unity C# integration code. | Good for teammate-side monitoring. |
| `Control_panel/User_Manual.md` | document | User manual for the legacy `Control_panel` workflow. Describes web UI, IK server, FK tool, bridge server, Unity monitor, and startup steps. | File content is currently garbled because of encoding mismatch, but the document intent is still identifiable. |
| `Control_panel/Plot_Trajectory.m` | MATLAB script | Lets user select a folder, finds the latest `return_trajectory_*.txt`, loads pose and motor trajectory pairs, and plots 3D path plus angle curves. | For offline visualization of exported return trajectories. |
| `Control_panel/TestPlot.m` | MATLAB script | Similar to `Plot_Trajectory.m`, but defaults to current working directory instead of prompting for a folder. | Quicker plotting helper for local testing. |
| `Control_panel/Unity)` | stray text file | Contains only a fragment of a console line: `[Bridge Server] Port 9090 (browser -`. | Looks accidental or truncated; not part of any usable workflow. |
| `Control_panel/ik_trajectory_20260422_211945.txt` | generated data | Logged motor joint trajectory output from `ik_server`. Each line has 5 joint values. | 45,953 lines; generated artifact. |
| `Control_panel/fk_ik_trajectory_20260422_211945.txt` | generated data | FK reconstruction output generated from the IK trajectory file. Each line stores the recovered pose. | 45,953 lines; used to compare against intended path. |
| `Control_panel/return_trajectory_motor_20260422_220814.txt` | generated data | Motor-space return trajectory exported by browser `smoothReturn()`. | 9,240 lines; tab-separated 5-axis joint values. |
| `Control_panel/return_trajectory_motor_20260422_220825.txt` | generated data | Same type as above, another exported run. | Generated artifact. |
| `Control_panel/return_trajectory_motor_20260422_221149.txt` | generated data | Same type as above, another exported run. | Generated artifact. |
| `Control_panel/return_trajectory_motor_20260422_221343.txt` | generated data | Same type as above, another exported run. | Generated artifact. |
| `Control_panel/return_trajectory_pose_20260422_220814.txt` | generated data | Pose-space return trajectory paired with the same timestamp motor trajectory. | 9,240 lines; tab-separated yaw/pitch/Z/Y/X values. |
| `Control_panel/return_trajectory_pose_20260422_220825.txt` | generated data | Same type as above, another exported run. | Generated artifact. |
| `Control_panel/return_trajectory_pose_20260422_221149.txt` | generated data | Same type as above, another exported run. | Generated artifact. |
| `Control_panel/return_trajectory_pose_20260422_221343.txt` | generated data | Same type as above, another exported run. | Generated artifact. |

### Control_panel/.vscode

| File | Type | Function and purpose | Notes |
| --- | --- | --- | --- |
| `Control_panel/.vscode/c_cpp_properties.json` | IDE config | Sets include path and GCC compiler path for VS Code C/C++ IntelliSense. | Uses `C:/msys64/ucrt64/bin/gcc.exe`. |
| `Control_panel/.vscode/launch.json` | IDE config | Debug launch setup for a C/C++ Runner debug session. | Paths still point to another old folder (`d:/Control_Robotics_Lab-main/...`), so it is stale. |
| `Control_panel/.vscode/settings.json` | IDE config | C/C++ Runner settings for compiler, warnings, include/exclude search, and debugger preferences. | Useful only in VS Code environment. |

### Control_panel/Kinematics_Simple

| File | Type | Function and purpose | Notes |
| --- | --- | --- | --- |
| `Control_panel/Kinematics_Simple/README.md` | document | Short English readme for the standalone CLI kinematics tool. | Clear and usable. |
| `Control_panel/Kinematics_Simple/build.bat` | build script | Compiles `kinematics_tool.cpp` into `kinematics_tool.exe` using `g++ -O3`. | Simple standalone build script. |
| `Control_panel/Kinematics_Simple/kinematics_tool.cpp` | C++ source | Standalone CLI tool for converting between pose and joint angles. Contains the same inverse kinematics family and a numerical forward-kinematics solver. | Good for quick math checks without WebSocket/UI. |
| `Control_panel/Kinematics_Simple/build/Debug/kinematics_tool.o` | object file | Intermediate compilation output. | Generated artifact. |
| `Control_panel/Kinematics_Simple/build/Debug/outDebug.exe` | debug binary | Debug executable created by an IDE/build tool. | Generated artifact. |

### Firmware folder

| File | Type | Function and purpose | Notes |
| --- | --- | --- | --- |
| `firmware/README.md` | document | Firmware-side README describing command protocol, pins, and tuning hints. | Content is partially garbled because of encoding mismatch. Also still references `Yuhan_motor` paths. |
| `firmware/YuhanSTM32Control/YuhanSTM32Control.ino` | firmware entry | Main STM32 sketch. Initializes encoders and finger controllers, receives `YH + 5 x int32` frames, parses text commands, runs 200 Hz control loop, and prints encoder/PWM status at 20 Hz. | This is the main firmware entry point. |
| `firmware/YuhanSTM32Control/encoder_manager.h` | header | Declares encoder initialization, count access, inversion, and reset APIs. | Used by `finger.cpp` and `.ino`. |
| `firmware/YuhanSTM32Control/encoder_manager.cpp` | firmware module | Handles 5 encoder channels with interrupts, including a special `/100` effective scaling path for motor 5. | Important: motor 5 scaling affects angle conversion and CSV correction. |
| `firmware/YuhanSTM32Control/motor.h` | header | Declares the low-level `Motor` class for H-bridge direction/PWM output. | Small abstraction layer. |
| `firmware/YuhanSTM32Control/motor.cpp` | firmware module | Implements PWM deadband, inversion, directional drive, and stop logic for one motor. | Hardware output layer. |
| `firmware/YuhanSTM32Control/finger.h` | header | Declares `FingerController`, the per-axis PID/hold/backlash controller, plus `sendParallelPair` for coupled axes. | Main control abstraction for each joint. |
| `firmware/YuhanSTM32Control/finger.cpp` | firmware module | Implements PID control, integral limiting, hold behavior, backlash compensation, deadband handling, and optional synchronized pair control. | Core control algorithm. |

### Cache and generated Python bytecode

| File | Type | Function and purpose | Notes |
| --- | --- | --- | --- |
| `__pycache__/server.cpython-312.pyc` | generated cache | Compiled Python bytecode for `server.py`. | Generated automatically; safe to ignore. |

## 4. Operational Role of Key Source Files

### `app.js` main responsibilities

- Builds the 5 motor cards and PID/angle tables.
- Connects to STM32 through Web Serial.
- Sends binary target frames with header `YH` and `5 x int32 little-endian`.
- Sends text configuration commands such as `!PID`, `!HOLD`, `!BACKLASH`, `!PWM`, `!MOTORINV`, `!ENCINV`, and `!ZEROENC`.
- Parses three kinds of firmware responses:
  - `CFG,...`
  - `PWM: ...`
  - `ENC: ...` or `ENC_CSV,...`
- Stores PID presets and angle calibration in browser `localStorage`.
- Loads trajectory files and converts `count`, `degree`, or `radian` input into raw counts.
- Exports structured CSV logs and raw serial text logs.

### `YuhanSTM32Control.ino` main responsibilities

- Maintains 5 `FingerController` objects.
- Accepts two receive modes:
  - binary target frame mode
  - text command mode
- Runs control at `200 Hz` using `CONTROL_US = 5000`.
- Prints encoder and PWM status every `50 ms`.
- Keeps motor 3 and motor 4 sync compensation disabled by default with `kSync = 0.0f` for easier debugging.

### `finger.cpp` control behavior

- PID output uses proportional, derivative, and optional integral terms.
- Integral is clamped to prevent runaway accumulation.
- Hold mode can apply a small sustaining PWM when target remains stable and error is inside deadband.
- Backlash compensation is only injected when commanded target direction reverses.
- Output is clamped by per-axis `maxPwm`.

## 5. How to Use the Current Main System

### Goal

Use the browser to control 5 STM32-driven axes, read encoder/PWM status, tune PID, and play trajectory files.

### Step 1: Upload firmware

1. Open Arduino IDE 2.x.
2. Install STM32 Arduino Core if it is not already installed.
3. Open:
   `firmware/YuhanSTM32Control/YuhanSTM32Control.ino`
4. Select the correct STM32 board, COM port, and upload method.
5. Click `Verify`.
6. Click `Upload`.

### Step 2: Start the local console server

From the project root:

```powershell
.\start_console.ps1
```

Or manually:

```powershell
python .\server.py
```

Then open:

```text
http://127.0.0.1:8765
```

Use **Edge** or **Chrome**, because the page depends on Web Serial.

### Step 3: Connect to STM32

1. Click `Connect`.
2. Choose the STM32 serial port.
3. Click `Read Config`.
4. Confirm connection status changes to `Connected`.

### Step 4: Basic safe first test

Recommended first-test settings:

- `Ki = 0`
- `backlash = 0`
- `holdPwm = 0`
- `holdMs = 0`
- conservative `maxPwm`

Suggested first move:

1. Change only one axis.
2. Send a very small target, such as `100` or `-100`.
3. Watch whether encoder value moves toward the target.
4. Check whether PWM stays saturated at the configured max.
5. If direction is wrong, fix `motorInvert` or `encoderInvert` before increasing PID gains.

### Step 5: Angle mode

If you want to work in angles instead of raw counts:

1. Fill `Counts / Degree` for all 5 motors in `Angle Calibration`.
2. Click `Save Angle Config`.
3. Change `Target Unit` to `Degree` or `Radian`.

The browser will still convert everything to raw counts before sending to STM32.

### Step 6: Trajectory playback

1. Prepare a `.txt` or `.csv` file.
2. Each row must contain exactly **5 numbers**.
3. No header row.
4. Choose the correct `Trajectory Unit`.
5. Set `Playback Interval (ms)`.
6. Click `Play Trajectory`.

Supported formats:

```text
0,0,0,0,0
100,120,50,-30,0
200,180,80,-60,0
```

or

```text
0 0 0 0 0
1.5 2.0 0.8 -0.5 0
3.0 3.5 1.2 -1.0 0
```

### Step 7: Logging and export

1. Click `Start Log` before testing.
2. Perform your motion or trajectory test.
3. Click `Stop Log`.
4. Export:
   - `Export CSV` for structured data
   - `Export Raw TXT` for raw serial lines

CSV includes target, encoder, PWM, timestamps, and raw line source.

## 6. How to Use the Legacy Control_panel Workflow

Use this only if you specifically want pose-space trajectory design, IK/FK validation, or Unity relay.

### One-click startup

From `Control_panel/`:

```bat
run_all.bat
```

This does three things:

1. Compiles `ik_server.cpp` -> `ik_server.exe`
2. Compiles `fk_test.cpp` -> `fk_test.exe`
3. Starts:
   - `ik_server.exe`
   - `fk_test.exe`
   - `python bridge_server.py`

### Browser controller

Open:

```text
Control_panel/index.html
```

Behavior:

- Connects to `ws://localhost:8080`
- Sends pose arrays like `[yaw, pitch, z, y, x]`
- Receives theta JSON from `ik_server`
- Can forward theta JSON to bridge server
- Can generate trajectory and ask `ik_server` to save files

### Unity relay

1. Start `bridge_server.py`.
2. Find its displayed LAN IP.
3. Browser controller bridge URL:

```text
ws://<LAN-IP>:9090/controller
```

4. Unity viewer URL:

```text
ws://<LAN-IP>:9090/unity
```

### FK verification

`fk_test.exe` watches the IK trajectory file and writes a mirrored verification output:

- input: `ik_trajectory_*.txt`
- output: `fk_ik_trajectory_*.txt`

This lets you check whether the generated motor trajectory reconstructs the intended pose path.

### MATLAB plotting

Use either:

- `Plot_Trajectory.m`
- `TestPlot.m`

to visualize:

- 3D end-effector path
- yaw/pitch curves
- motor joint curves

## 7. Protocol Summary

### Binary target frame

- header: `YH`
- payload: `5 x int32 little-endian`
- total size: `22 bytes`

### Text commands accepted by firmware

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

### Typical firmware responses

```text
ENC: 0=123 1=456 2=789 3=10 4=11
CFG,0,0.3000,0.0000,0.1800,0,0,30,0,40,0,1
PWM: 0=12/40 1=0/40 2=-5/35 3=0/20 4=0/25
```

## 8. Important Findings and Maintenance Notes

### What is production-relevant

These are the files that matter most for actual hardware use:

- `index.html`
- `styles.css`
- `app.js`
- `server.py`
- `start_console.ps1`
- `firmware/YuhanSTM32Control/YuhanSTM32Control.ino`
- `firmware/YuhanSTM32Control/encoder_manager.*`
- `firmware/YuhanSTM32Control/motor.*`
- `firmware/YuhanSTM32Control/finger.*`

### What is mostly legacy or experimental

- everything under `Control_panel/`
- `generate_motor5_corrected_data.py`
- generated trajectory `.txt` files
- compiled `.exe`, `.o`, `.pyc`

### Known issues

1. Several documents still reference the old folder name `Yuhan_motor` instead of this repository path.
2. `Control_panel/User_Manual.md` and `firmware/README.md` show encoding corruption.
3. `Control_panel/.vscode/launch.json` still points to an unrelated old absolute path.
4. `Control_panel/Unity)` appears to be an accidental leftover file.
5. The repository currently contains many generated artifacts that do not need to be versioned for day-to-day development.

## 9. Recommended Cleanup Strategy

If you want this folder easier to maintain, the safest cleanup order would be:

1. Keep the current root console and firmware as the main supported system.
2. Decide whether `Control_panel/` is still needed.
3. Fix document encoding for `Control_panel/User_Manual.md` and `firmware/README.md`.
4. Remove or archive generated files:
   - `*.exe`
   - `*.o`
   - `*.pyc`
   - `*trajectory_*.txt`
5. Delete the stray `Control_panel/Unity)` file if confirmed unnecessary.

## 10. Quick Start Checklist

### For real hardware control

1. Upload `firmware/YuhanSTM32Control/YuhanSTM32Control.ino`
2. Run `.\start_console.ps1`
3. Open `http://127.0.0.1:8765`
4. Connect serial
5. Read config
6. Test one axis with a small move
7. Start logging before any bigger motion

### For IK/FK/Unity experiments

1. Enter `Control_panel`
2. Run `run_all.bat`
3. Open `Control_panel/index.html`
4. If needed, connect Unity to `ws://<LAN-IP>:9090/unity`

