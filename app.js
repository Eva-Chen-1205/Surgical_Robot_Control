const MOTOR_COUNT = 5;
const MAX_LOG_LINES = 200;
const FRAME_HEADER = [0x59, 0x48]; // "YH"

const encoderState = Array.from({ length: MOTOR_COUNT }, () => 0);
const pwmState = Array.from({ length: MOTOR_COUNT }, () => 0);
const targets = Array.from({ length: MOTOR_COUNT }, () => 0);
const pidConfigs = [
  { kp: 0.30, ki: 0.0, kd: 0.18, holdPwm: 0, holdMs: 0, deadband: 30, backlash: 0, maxPwm: 40 },
  { kp: 0.30, ki: 0.0, kd: 0.18, holdPwm: 0, holdMs: 0, deadband: 30, backlash: 0, maxPwm: 40 },
  { kp: 0.08, ki: 0.0, kd: 0.01, holdPwm: 0, holdMs: 0, deadband: 30, backlash: 0, maxPwm: 35 },
  { kp: 0.18, ki: 0.0, kd: 0.03, holdPwm: 0, holdMs: 0, deadband: 30, backlash: 0, maxPwm: 35 },
  { kp: 2.00, ki: 0.0, kd: 0.00, holdPwm: 0, holdMs: 0, deadband: 3, backlash: 0, maxPwm: 25 }
];

const motorCards = [];
const pidRows = [];
const logSamples = [];
const rawLines = [];

let port = null;
let writer = null;
let reader = null;
let readLoopActive = false;
let autoSendTimer = null;
let trajectoryTimer = null;
let trajectoryPoints = [];
let trajectoryIndex = 0;
let isLogging = false;
let sessionStart = null;
let lastRxAt = null;
let lastTxAt = null;

const textEncoder = new TextEncoder();

const connectionState = document.querySelector("#connectionState");
const sampleCount = document.querySelector("#sampleCount");
const trajectoryCount = document.querySelector("#trajectoryCount");
const serialLog = document.querySelector("#serialLog");
const lastTxLabel = document.querySelector("#lastTxLabel");
const lastRxLabel = document.querySelector("#lastRxLabel");
const logStatus = document.querySelector("#logStatus");
const trajectoryStatus = document.querySelector("#trajectoryStatus");
const pidStatus = document.querySelector("#pidStatus");
const encoderTableBody = document.querySelector("#encoderTableBody");
const pidTableBody = document.querySelector("#pidTableBody");
const connectBtn = document.querySelector("#connectBtn");
const disconnectBtn = document.querySelector("#disconnectBtn");
const sendNowBtn = document.querySelector("#sendNowBtn");
const stopAllBtn = document.querySelector("#stopAllBtn");
const zeroTargetsBtn = document.querySelector("#zeroTargetsBtn");
const zeroEncoderBtn = document.querySelector("#zeroEncoderBtn");
const startLogBtn = document.querySelector("#startLogBtn");
const stopLogBtn = document.querySelector("#stopLogBtn");
const clearLogBtn = document.querySelector("#clearLogBtn");
const exportCsvBtn = document.querySelector("#exportCsvBtn");
const exportRawBtn = document.querySelector("#exportRawBtn");
const refreshConfigBtn = document.querySelector("#refreshConfigBtn");
const applyAllConfigBtn = document.querySelector("#applyAllConfigBtn");
const disableBacklashBtn = document.querySelector("#disableBacklashBtn");
const disableHoldBtn = document.querySelector("#disableHoldBtn");
const baudRateInput = document.querySelector("#baudRate");
const autoSendModeInput = document.querySelector("#autoSendMode");
const sendIntervalMsInput = document.querySelector("#sendIntervalMs");
const jogStepInput = document.querySelector("#jogStep");
const trajectoryFileInput = document.querySelector("#trajectoryFile");
const trajectoryIntervalMsInput = document.querySelector("#trajectoryIntervalMs");
const playTrajectoryBtn = document.querySelector("#playTrajectoryBtn");
const stopTrajectoryBtn = document.querySelector("#stopTrajectoryBtn");
const clearTrajectoryBtn = document.querySelector("#clearTrajectoryBtn");

buildMotorCards();
buildPidRows();
renderEncoderTable();
syncAllCards();
syncAllPidRows();
updateConnectionState(false);
updateLogSummary();
updateTrajectorySummary();
updateAutoSendTimer();

connectBtn.addEventListener("click", connectSerial);
disconnectBtn.addEventListener("click", disconnectSerial);
sendNowBtn.addEventListener("click", () => void sendTargets("manual"));
stopAllBtn.addEventListener("click", () => {
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    setTarget(i, 0, false);
  }
  syncAllCards();
  void sendTargets("stop-all");
});
zeroTargetsBtn.addEventListener("click", () => {
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    setTarget(i, 0, false);
  }
  syncAllCards();
  maybeAutoSend();
});
zeroEncoderBtn.addEventListener("click", () => void sendTextCommand("!ZEROENC", "zero-encoder"));
startLogBtn.addEventListener("click", startLogging);
stopLogBtn.addEventListener("click", stopLogging);
clearLogBtn.addEventListener("click", clearLogs);
exportCsvBtn.addEventListener("click", exportCsv);
exportRawBtn.addEventListener("click", exportRawText);
refreshConfigBtn.addEventListener("click", () => void readAllConfigs());
applyAllConfigBtn.addEventListener("click", () => void applyAllConfigs());
disableBacklashBtn.addEventListener("click", () => {
  for (const config of pidConfigs) {
    config.backlash = 0;
  }
  syncAllPidRows();
  void applyAllConfigs();
});
disableHoldBtn.addEventListener("click", () => {
  for (const config of pidConfigs) {
    config.holdPwm = 0;
    config.holdMs = 0;
  }
  syncAllPidRows();
  void applyAllConfigs();
});
trajectoryFileInput.addEventListener("change", loadTrajectoryFile);
playTrajectoryBtn.addEventListener("click", playTrajectory);
stopTrajectoryBtn.addEventListener("click", stopTrajectory);
clearTrajectoryBtn.addEventListener("click", clearTrajectory);
autoSendModeInput.addEventListener("change", updateAutoSendTimer);
sendIntervalMsInput.addEventListener("change", updateAutoSendTimer);

if (!("serial" in navigator)) {
  appendSerialLog("[system] 這個瀏覽器不支援 Web Serial，請改用 Edge 或 Chrome。");
}

function buildMotorCards() {
  const container = document.querySelector("#motorGrid");
  const template = document.querySelector("#motorCardTemplate");

  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    const node = template.content.cloneNode(true);
    const targetInput = node.querySelector(".target-input");
    const targetSlider = node.querySelector(".target-slider");
    const jogDownBtn = node.querySelector(".jog-down");
    const jogUpBtn = node.querySelector(".jog-up");
    const sendSingleBtn = node.querySelector(".send-single");
    const encoderValue = node.querySelector(".encoder-value");
    const errorValue = node.querySelector(".error-value");
    const pwmValue = node.querySelector(".pwm-value");

    node.querySelector("h3").textContent = `Motor ${i + 1}`;
    targetInput.value = "0";
    targetSlider.value = "0";

    const applyTarget = (value) => {
      setTarget(i, value);
      syncMotorCard(i);
    };

    targetInput.addEventListener("change", () => applyTarget(Number(targetInput.value)));
    targetSlider.addEventListener("input", () => applyTarget(Number(targetSlider.value)));
    jogDownBtn.addEventListener("click", () => applyTarget(targets[i] - getJogStep()));
    jogUpBtn.addEventListener("click", () => applyTarget(targets[i] + getJogStep()));
    sendSingleBtn.addEventListener("click", () => void sendTargets(`single-${i}`));

    container.appendChild(node);
    motorCards.push({ targetInput, targetSlider, encoderValue, errorValue, pwmValue });
  }
}

function buildPidRows() {
  const template = document.querySelector("#pidRowTemplate");
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    const node = template.content.cloneNode(true);
    const row = node.querySelector("tr");
    row.querySelector(".pid-motor-label").textContent = `Motor ${i + 1}`;

    const refs = {
      row,
      kp: row.querySelector(".pid-kp"),
      ki: row.querySelector(".pid-ki"),
      kd: row.querySelector(".pid-kd"),
      holdPwm: row.querySelector(".pid-hold-pwm"),
      holdMs: row.querySelector(".pid-hold-ms"),
      deadband: row.querySelector(".pid-deadband"),
      backlash: row.querySelector(".pid-backlash"),
      maxPwm: row.querySelector(".pid-max-pwm"),
      readBtn: row.querySelector(".pid-read"),
      applyBtn: row.querySelector(".pid-apply")
    };

    refs.readBtn.addEventListener("click", () => void readConfig(i));
    refs.applyBtn.addEventListener("click", () => void applyConfig(i));

    pidTableBody.appendChild(node);
    pidRows.push(refs);
  }
}

function renderEncoderTable() {
  encoderTableBody.innerHTML = "";
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    const row = document.createElement("tr");
    row.innerHTML = `
      <td>Motor ${i + 1}</td>
      <td id="target-cell-${i}">${targets[i]}</td>
      <td id="encoder-cell-${i}">${encoderState[i]}</td>
      <td id="delta-cell-${i}">${targets[i] - encoderState[i]}</td>
      <td id="pwm-cell-${i}">${pwmState[i]}</td>
      <td id="max-pwm-cell-${i}">${pidConfigs[i].maxPwm}</td>
    `;
    encoderTableBody.appendChild(row);
  }
}

function syncMotorCard(index) {
  const encoder = encoderState[index];
  const target = targets[index];
  motorCards[index].targetInput.value = String(target);
  motorCards[index].targetSlider.value = String(clampTarget(target));
  motorCards[index].encoderValue.textContent = String(encoder);
  motorCards[index].errorValue.textContent = String(target - encoder);
  motorCards[index].pwmValue.textContent = `${pwmState[index]} / ${pidConfigs[index].maxPwm}`;
  document.querySelector(`#target-cell-${index}`).textContent = String(target);
  document.querySelector(`#encoder-cell-${index}`).textContent = String(encoder);
  document.querySelector(`#delta-cell-${index}`).textContent = String(target - encoder);
  document.querySelector(`#pwm-cell-${index}`).textContent = String(pwmState[index]);
  document.querySelector(`#max-pwm-cell-${index}`).textContent = String(pidConfigs[index].maxPwm);
}

function syncAllCards() {
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    syncMotorCard(i);
  }
}

function syncPidRow(index) {
  const config = pidConfigs[index];
  const row = pidRows[index];
  row.kp.value = String(config.kp);
  row.ki.value = String(config.ki);
  row.kd.value = String(config.kd);
  row.holdPwm.value = String(config.holdPwm);
  row.holdMs.value = String(config.holdMs);
  row.deadband.value = String(config.deadband);
  row.backlash.value = String(config.backlash);
  row.maxPwm.value = String(config.maxPwm);
}

function syncAllPidRows() {
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    syncPidRow(i);
  }
}

function readPidRow(index) {
  const row = pidRows[index];
  pidConfigs[index] = {
    kp: Number(row.kp.value),
    ki: Number(row.ki.value),
    kd: Number(row.kd.value),
    holdPwm: Number(row.holdPwm.value),
    holdMs: Number(row.holdMs.value),
    deadband: Number(row.deadband.value),
    backlash: Number(row.backlash.value),
    maxPwm: Number(row.maxPwm.value)
  };
}

function setTarget(index, value, autoSend = true) {
  const safeValue = Number.isFinite(value) ? Math.round(value) : 0;
  targets[index] = safeValue;
  if (autoSend) {
    maybeAutoSend();
  }
}

function clampTarget(value) {
  return Math.max(-300000, Math.min(300000, Math.round(value)));
}

function getJogStep() {
  const step = Number(jogStepInput.value);
  return Number.isFinite(step) && step > 0 ? Math.round(step) : 500;
}

async function connectSerial() {
  if (!("serial" in navigator)) {
    alert("請用支援 Web Serial 的瀏覽器，例如 Microsoft Edge。");
    return;
  }

  if (port) {
    appendSerialLog("[system] 已經連線。");
    return;
  }

  try {
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: getBaudRate() });
    writer = port.writable.getWriter();
    sessionStart = new Date();
    updateConnectionState(true);
    appendSerialLog("[system] Serial connected.");
    readLoopActive = true;
    void readSerialLoop();
    updateAutoSendTimer();
    await readAllConfigs();
  } catch (error) {
    appendSerialLog(`[error] connect failed: ${error.message}`);
    await safeClosePort();
  }
}

async function disconnectSerial() {
  stopTrajectory();
  clearInterval(autoSendTimer);
  autoSendTimer = null;
  await safeClosePort();
  appendSerialLog("[system] Serial disconnected.");
  updateConnectionState(false);
}

async function safeClosePort() {
  readLoopActive = false;

  try {
    if (reader) {
      await reader.cancel();
      reader.releaseLock();
      reader = null;
    }
  } catch (error) {
    appendSerialLog(`[warn] reader close: ${error.message}`);
  }

  try {
    if (writer) {
      writer.releaseLock();
      writer = null;
    }
  } catch (error) {
    appendSerialLog(`[warn] writer close: ${error.message}`);
  }

  try {
    if (port) {
      await port.close();
    }
  } catch (error) {
    appendSerialLog(`[warn] port close: ${error.message}`);
  }

  port = null;
}

async function readSerialLoop() {
  if (!port?.readable) {
    return;
  }

  const decoder = new TextDecoder();
  let buffer = "";

  while (port && readLoopActive) {
    try {
      reader = port.readable.getReader();
      while (readLoopActive) {
        const { value, done } = await reader.read();
        if (done) {
          break;
        }
        if (!value) {
          continue;
        }

        buffer += decoder.decode(value, { stream: true });
        const parts = buffer.split(/\r?\n/);
        buffer = parts.pop() ?? "";

        for (const line of parts) {
          handleIncomingLine(line.trim());
        }
      }
    } catch (error) {
      appendSerialLog(`[error] read failed: ${error.message}`);
      break;
    } finally {
      if (reader) {
        reader.releaseLock();
        reader = null;
      }
    }
  }
}

function handleIncomingLine(line) {
  if (!line) {
    return;
  }

  lastRxAt = new Date();
  lastRxLabel.textContent = formatClock(lastRxAt);
  appendSerialLog(line);
  rawLines.push(`${new Date().toISOString()} ${line}`);
  if (rawLines.length > 5000) {
    rawLines.shift();
  }

  if (parseConfigLine(line)) {
    return;
  }

  if (parsePwmLine(line)) {
    return;
  }

  const parsed = parseEncoderLine(line);
  if (!parsed) {
    return;
  }

  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    encoderState[i] = parsed.values[i];
    syncMotorCard(i);
  }

  if (isLogging) {
    pushLogSample(parsed.source, line);
  }
}

function parseConfigLine(line) {
  const match = line.match(/^CFG,(\d+),([-\d.]+),([-\d.]+),([-\d.]+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)$/);
  if (!match) {
    return false;
  }

  const motorIndex = Number(match[1]);
  if (!Number.isInteger(motorIndex) || motorIndex < 0 || motorIndex >= MOTOR_COUNT) {
    return false;
  }

  pidConfigs[motorIndex] = {
    kp: Number(match[2]),
    ki: Number(match[3]),
    kd: Number(match[4]),
    holdPwm: Number(match[5]),
    holdMs: Number(match[6]),
    deadband: Number(match[7]),
    backlash: Number(match[8]),
    maxPwm: Number(match[9])
  };
  syncPidRow(motorIndex);
  syncMotorCard(motorIndex);
  pidStatus.textContent = `已讀到 Motor ${motorIndex + 1} 的設定。`;
  return true;
}

function parsePwmLine(line) {
  const match = line.match(/PWM:\s*0=(-?\d+)\/(\d+)\s+1=(-?\d+)\/(\d+)\s+2=(-?\d+)\/(\d+)\s+3=(-?\d+)\/(\d+)\s+4=(-?\d+)\/(\d+)/i);
  if (!match) {
    return false;
  }

  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    pwmState[i] = Number(match[1 + i * 2]);
    pidConfigs[i].maxPwm = Number(match[2 + i * 2]);
    syncMotorCard(i);
  }
  return true;
}

function parseEncoderLine(line) {
  const encStyleMatch = line.match(/ENC:\s*0=(-?\d+)\s+1=(-?\d+)\s+2=(-?\d+)\s+3=(-?\d+)\s+4=(-?\d+)/i);
  if (encStyleMatch) {
    return {
      source: "enc-text",
      values: encStyleMatch.slice(1).map(Number)
    };
  }

  if (line.startsWith("ENC_CSV")) {
    const tokens = line.split(",").slice(-5).map((token) => Number(token.trim()));
    if (tokens.length === MOTOR_COUNT && tokens.every(Number.isFinite)) {
      return {
        source: "enc-csv",
        values: tokens
      };
    }
  }

  return null;
}

function pushLogSample(source, rawLine) {
  const now = new Date();
  const elapsedMs = sessionStart ? now.getTime() - sessionStart.getTime() : 0;
  logSamples.push({
    iso_time: now.toISOString(),
    elapsed_ms: elapsedMs,
    source,
    raw_line: rawLine,
    target_0: targets[0],
    target_1: targets[1],
    target_2: targets[2],
    target_3: targets[3],
    target_4: targets[4],
    encoder_0: encoderState[0],
    encoder_1: encoderState[1],
    encoder_2: encoderState[2],
    encoder_3: encoderState[3],
    encoder_4: encoderState[4],
    pwm_0: pwmState[0],
    pwm_1: pwmState[1],
    pwm_2: pwmState[2],
    pwm_3: pwmState[3],
    pwm_4: pwmState[4]
  });
  updateLogSummary();
}

async function sendTargets(reason = "manual") {
  if (!writer) {
    if (reason === "manual" || reason === "stop-all" || reason.startsWith("single-")) {
      appendSerialLog("[warn] 尚未連線，無法送出 target。");
    }
    return;
  }

  const buffer = new ArrayBuffer(FRAME_HEADER.length + MOTOR_COUNT * 4);
  const bytes = new Uint8Array(buffer);
  bytes[0] = FRAME_HEADER[0];
  bytes[1] = FRAME_HEADER[1];
  const view = new DataView(buffer);
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    view.setInt32(FRAME_HEADER.length + i * 4, targets[i], true);
  }

  await writer.write(bytes);
  markTx(reason);
}

async function sendTextCommand(command, reason = "command") {
  if (!writer) {
    appendSerialLog("[warn] 尚未連線，無法送出命令。");
    return;
  }
  await writer.write(textEncoder.encode(`${command}\n`));
  markTx(reason);
}

function maybeAutoSend() {
  if (autoSendModeInput.value === "change") {
    void sendTargets("change");
  }
}

function updateAutoSendTimer() {
  clearInterval(autoSendTimer);
  autoSendTimer = null;

  if (autoSendModeInput.value === "interval" && writer) {
    autoSendTimer = setInterval(() => {
      void sendTargets("interval");
    }, getSendIntervalMs());
  }
}

async function readConfig(index) {
  await sendTextCommand(`!GET,${index}`, `get-config-${index}`);
}

async function readAllConfigs() {
  await sendTextCommand("!GETALL", "get-config-all");
}

async function applyConfig(index) {
  readPidRow(index);
  const cfg = pidConfigs[index];
  await sendTextCommand(
    `!PID,${index},${cfg.kp},${cfg.ki},${cfg.kd}`,
    `pid-${index}`
  );
  await sendTextCommand(
    `!HOLD,${index},${Math.round(cfg.holdPwm)},${Math.round(cfg.holdMs)},${Math.round(cfg.deadband)}`,
    `hold-${index}`
  );
  await sendTextCommand(
    `!BACKLASH,${index},${Math.round(cfg.backlash)}`,
    `backlash-${index}`
  );
  await sendTextCommand(
    `!PWM,${index},${Math.round(cfg.maxPwm)}`,
    `pwm-${index}`
  );
  pidStatus.textContent = `已送出 Motor ${index + 1} 的 PID 設定。`;
}

async function applyAllConfigs() {
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    await applyConfig(i);
  }
  pidStatus.textContent = "已送出全部 PID 設定。";
}

function startLogging() {
  isLogging = true;
  if (!sessionStart) {
    sessionStart = new Date();
  }
  logStatus.textContent = "Encoder 記錄中，之後可匯出 CSV。";
}

function stopLogging() {
  isLogging = false;
  logStatus.textContent = `記錄已停止，目前累積 ${logSamples.length} 筆樣本。`;
}

function clearLogs() {
  logSamples.length = 0;
  rawLines.length = 0;
  serialLog.textContent = "Waiting for serial data...";
  updateLogSummary();
  logStatus.textContent = "已清除記錄。";
}

function exportCsv() {
  if (logSamples.length === 0) {
    alert("目前沒有 encoder 樣本可以匯出。");
    return;
  }

  const header = Object.keys(logSamples[0]).join(",");
  const rows = logSamples.map((sample) =>
    Object.values(sample).map(csvEscape).join(",")
  );
  downloadBlob([header, ...rows].join("\n"), buildFileName("encoder_log", "csv"), "text/csv;charset=utf-8");
}

function exportRawText() {
  if (rawLines.length === 0) {
    alert("目前沒有 raw serial 內容可以匯出。");
    return;
  }

  downloadBlob(rawLines.join("\n"), buildFileName("serial_raw", "txt"), "text/plain;charset=utf-8");
}

function loadTrajectoryFile(event) {
  const file = event.target.files?.[0];
  if (!file) {
    return;
  }

  file.text().then((text) => {
    try {
      trajectoryPoints = parseTrajectoryText(text);
      trajectoryIndex = 0;
      updateTrajectorySummary();
      trajectoryStatus.textContent = `已載入 ${trajectoryPoints.length} 筆軌跡點：${file.name}`;
    } catch (error) {
      trajectoryPoints = [];
      trajectoryIndex = 0;
      updateTrajectorySummary();
      trajectoryStatus.textContent = `軌跡檔讀取失敗：${error.message}`;
    }
  });
}

function parseTrajectoryText(text) {
  const points = text
    .split(/\r?\n/)
    .map((line) => line.trim())
    .filter(Boolean)
    .map((line, index) => {
      const tokens = line
        .split(/[,\s]+/)
        .map((token) => token.trim())
        .filter(Boolean)
        .map(Number);

      if (tokens.length !== MOTOR_COUNT || tokens.some((value) => !Number.isFinite(value))) {
        throw new Error(`第 ${index + 1} 行不是 5 個數字`);
      }
      return tokens.map((value) => Math.round(value));
    });

  if (points.length === 0) {
    throw new Error("檔案中沒有可用資料");
  }
  return points;
}

function playTrajectory() {
  if (trajectoryPoints.length === 0) {
    alert("請先載入軌跡檔。");
    return;
  }

  stopTrajectory();
  trajectoryIndex = 0;
  const intervalMs = Math.max(1, Math.round(Number(trajectoryIntervalMsInput.value) || 5));
  trajectoryStatus.textContent = `軌跡播放中，每 ${intervalMs} ms 送出一筆。`;

  trajectoryTimer = setInterval(() => {
    if (trajectoryIndex >= trajectoryPoints.length) {
      stopTrajectory();
      trajectoryStatus.textContent = `軌跡播放完成，共 ${trajectoryPoints.length} 筆。`;
      return;
    }

    const point = trajectoryPoints[trajectoryIndex];
    for (let i = 0; i < MOTOR_COUNT; i += 1) {
      targets[i] = point[i];
    }
    syncAllCards();
    void sendTargets(`trajectory-${trajectoryIndex}`);
    trajectoryIndex += 1;
  }, intervalMs);
}

function stopTrajectory() {
  clearInterval(trajectoryTimer);
  trajectoryTimer = null;
}

function clearTrajectory() {
  stopTrajectory();
  trajectoryPoints = [];
  trajectoryIndex = 0;
  trajectoryFileInput.value = "";
  updateTrajectorySummary();
  trajectoryStatus.textContent = "已清除軌跡資料。";
}

function appendSerialLog(line) {
  const lines = serialLog.textContent === "Waiting for serial data..."
    ? []
    : serialLog.textContent.split("\n");
  lines.push(line);
  if (lines.length > MAX_LOG_LINES) {
    lines.splice(0, lines.length - MAX_LOG_LINES);
  }
  serialLog.textContent = lines.join("\n");
  serialLog.scrollTop = serialLog.scrollHeight;
}

function updateConnectionState(connected) {
  connectionState.textContent = connected ? "Connected" : "Disconnected";
}

function updateLogSummary() {
  sampleCount.textContent = String(logSamples.length);
}

function updateTrajectorySummary() {
  trajectoryCount.textContent = String(trajectoryPoints.length);
}

function getBaudRate() {
  const baudRate = Number(baudRateInput.value);
  return Number.isFinite(baudRate) && baudRate > 0 ? Math.round(baudRate) : 115200;
}

function getSendIntervalMs() {
  const intervalMs = Number(sendIntervalMsInput.value);
  return Number.isFinite(intervalMs) && intervalMs > 0 ? Math.round(intervalMs) : 50;
}

function formatClock(date) {
  return date.toLocaleTimeString("zh-TW", { hour12: false });
}

function csvEscape(value) {
  const text = String(value ?? "");
  if (/[",\n]/.test(text)) {
    return `"${text.replace(/"/g, "\"\"")}"`;
  }
  return text;
}

function buildFileName(prefix, extension) {
  const now = new Date();
  const stamp = now.toISOString().replace(/[:.]/g, "-");
  return `${prefix}_${stamp}.${extension}`;
}

function downloadBlob(content, filename, mimeType) {
  const blob = new Blob([content], { type: mimeType });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = filename;
  document.body.appendChild(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
}

function markTx(reason) {
  lastTxAt = new Date();
  lastTxLabel.textContent = `${formatClock(lastTxAt)} (${reason})`;
}
