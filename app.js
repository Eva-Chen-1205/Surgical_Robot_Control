const MOTOR_COUNT = 5;
const MAX_LOG_LINES = 200;
const FRAME_HEADER = [0x59, 0x48]; // "YH"
const PID_PRESET_STORAGE_KEY = "yuhanMotorPidPresets.v1";
const ANGLE_CONFIG_STORAGE_KEY = "yuhanMotorAngleConfig.v1";
const COUNT_LIMIT = 300000;
const DEFAULT_COUNTS_PER_DEGREE = [
  1553.0667,
  1553.0667,
  1553.0667,
  1553.0667,
  15.5307
];

const encoderState = Array.from({ length: MOTOR_COUNT }, () => 0);
const pwmState = Array.from({ length: MOTOR_COUNT }, () => 0);
const targets = Array.from({ length: MOTOR_COUNT }, () => 0);
const angleConfigs = Array.from({ length: MOTOR_COUNT }, (_, index) => ({
  countsPerDegree: DEFAULT_COUNTS_PER_DEGREE[index]
}));
const pidConfigs = [
  { kp: 0.27, ki: 0.0, kd: 0.01, holdPwm: 0, holdMs: 0, deadband: 30, backlash: 0, maxPwm: 100, motorInvert: false, encoderInvert: false },
  { kp: 0.30, ki: 0.001, kd: 0.01, holdPwm: 0, holdMs: 0, deadband: 100, backlash: 0, maxPwm: 100, motorInvert: false, encoderInvert: false },
  { kp: 0.35, ki: 0.0, kd: 0.01, holdPwm: 0, holdMs: 0, deadband: 100, backlash: 0, maxPwm: 100, motorInvert: false, encoderInvert: false },
  { kp: 0.20, ki: 0.0, kd: 0.01, holdPwm: 0, holdMs: 0, deadband: 30, backlash: 0, maxPwm: 80, motorInvert: true, encoderInvert: false },
  { kp: 2.00, ki: 0.0, kd: 0.00, holdPwm: 0, holdMs: 0, deadband: 3, backlash: 0, maxPwm: 25, motorInvert: false, encoderInvert: false }
];

const motorCards = [];
const pidRows = [];
const angleRows = [];
const logSamples = [];
const rawLines = [];
let pidPresets = [];

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
const angleStatus = document.querySelector("#angleStatus");
const encoderTableBody = document.querySelector("#encoderTableBody");
const pidTableBody = document.querySelector("#pidTableBody");
const angleTableBody = document.querySelector("#angleTableBody");
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
const presetNameInput = document.querySelector("#presetNameInput");
const presetSelect = document.querySelector("#presetSelect");
const savePresetBtn = document.querySelector("#savePresetBtn");
const loadPresetBtn = document.querySelector("#loadPresetBtn");
const deletePresetBtn = document.querySelector("#deletePresetBtn");
const baudRateInput = document.querySelector("#baudRate");
const autoSendModeInput = document.querySelector("#autoSendMode");
const sendIntervalMsInput = document.querySelector("#sendIntervalMs");
const targetUnitSelect = document.querySelector("#targetUnitSelect");
const jogStepInput = document.querySelector("#jogStep");
const trajectoryFileInput = document.querySelector("#trajectoryFile");
const trajectoryIntervalMsInput = document.querySelector("#trajectoryIntervalMs");
const trajectoryUnitSelect = document.querySelector("#trajectoryUnitSelect");
const playTrajectoryBtn = document.querySelector("#playTrajectoryBtn");
const stopTrajectoryBtn = document.querySelector("#stopTrajectoryBtn");
const clearTrajectoryBtn = document.querySelector("#clearTrajectoryBtn");
const saveAngleConfigBtn = document.querySelector("#saveAngleConfigBtn");
const resetAngleConfigBtn = document.querySelector("#resetAngleConfigBtn");

buildMotorCards();
buildPidRows();
buildAngleRows();
loadPidPresets();
loadAngleConfigs();
renderPidPresetOptions();
renderEncoderTable();
syncAllCards();
syncAllPidRows();
syncAllAngleRows();
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
savePresetBtn.addEventListener("click", saveCurrentPidPreset);
loadPresetBtn.addEventListener("click", loadSelectedPidPreset);
deletePresetBtn.addEventListener("click", deleteSelectedPidPreset);
trajectoryFileInput.addEventListener("change", loadTrajectoryFile);
playTrajectoryBtn.addEventListener("click", playTrajectory);
stopTrajectoryBtn.addEventListener("click", stopTrajectory);
clearTrajectoryBtn.addEventListener("click", clearTrajectory);
autoSendModeInput.addEventListener("change", updateAutoSendTimer);
sendIntervalMsInput.addEventListener("change", updateAutoSendTimer);
targetUnitSelect.addEventListener("change", handleTargetUnitChange);
saveAngleConfigBtn.addEventListener("click", saveAngleConfigs);
resetAngleConfigBtn.addEventListener("click", resetAngleConfigs);

if (!("serial" in navigator)) {
  appendSerialLog("[system] Web Serial is not available. Please use Edge or Chrome.");
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
    const unitBadge = node.querySelector(".motor-badge");
    const encoderValue = node.querySelector(".encoder-value");
    const encoderDegreeValue = node.querySelector(".encoder-degree-value");
    const errorValue = node.querySelector(".error-value");
    const pwmValue = node.querySelector(".pwm-value");

    node.querySelector("h3").textContent = `Motor ${i + 1}`;
    targetInput.value = "0";
    targetSlider.value = "0";

    const applyTarget = (value) => {
      const targetCount = convertDisplayToCount(i, value);
      if (targetCount === null) {
        angleStatus.textContent = `Motor ${i + 1} is missing counts/degree. Fill Angle Calibration first.`;
        return;
      }
      setTarget(i, targetCount);
      syncMotorCard(i);
    };

    targetInput.addEventListener("change", () => applyTarget(Number(targetInput.value)));
    targetSlider.addEventListener("input", () => applyTarget(Number(targetSlider.value)));
    jogDownBtn.addEventListener("click", () => {
      if (getTargetUnit() === "degree") {
        const currentDegree = convertCountToDegree(i, targets[i]);
        if (currentDegree === null) {
          angleStatus.textContent = `Motor ${i + 1} is missing counts/degree. Fill Angle Calibration first.`;
          return;
        }
        applyTarget(currentDegree - getDisplayJogStep());
        return;
      }
      if (getTargetUnit() === "radian") {
        const currentDegree = convertCountToDegree(i, targets[i]);
        if (currentDegree === null) {
          angleStatus.textContent = `Motor ${i + 1} is missing counts/degree. Fill Angle Calibration first.`;
          return;
        }
        applyTarget(currentDegree * Math.PI / 180 - getDisplayJogStep());
        return;
      }
      applyTarget(targets[i] - getDisplayJogStep());
    });
    jogUpBtn.addEventListener("click", () => {
      if (getTargetUnit() === "degree") {
        const currentDegree = convertCountToDegree(i, targets[i]);
        if (currentDegree === null) {
          angleStatus.textContent = `Motor ${i + 1} is missing counts/degree. Fill Angle Calibration first.`;
          return;
        }
        applyTarget(currentDegree + getDisplayJogStep());
        return;
      }
      if (getTargetUnit() === "radian") {
        const currentDegree = convertCountToDegree(i, targets[i]);
        if (currentDegree === null) {
          angleStatus.textContent = `Motor ${i + 1} is missing counts/degree. Fill Angle Calibration first.`;
          return;
        }
        applyTarget(currentDegree * Math.PI / 180 + getDisplayJogStep());
        return;
      }
      applyTarget(targets[i] + getDisplayJogStep());
    });
    sendSingleBtn.addEventListener("click", () => void sendTargets(`single-${i}`));

    container.appendChild(node);
    motorCards.push({ targetInput, targetSlider, unitBadge, encoderValue, encoderDegreeValue, errorValue, pwmValue });
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
      motorInvert: row.querySelector(".pid-motor-invert"),
      encoderInvert: row.querySelector(".pid-encoder-invert"),
      readBtn: row.querySelector(".pid-read"),
      applyBtn: row.querySelector(".pid-apply")
    };

    refs.readBtn.addEventListener("click", () => void readConfig(i));
    refs.applyBtn.addEventListener("click", () => void applyConfig(i));

    pidTableBody.appendChild(node);
    pidRows.push(refs);
  }
}

function buildAngleRows() {
  const template = document.querySelector("#angleRowTemplate");
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    const node = template.content.cloneNode(true);
    const row = node.querySelector("tr");
    const countsPerDegreeInput = row.querySelector(".angle-counts-per-degree");
    const encoderDegreeCell = row.querySelector(".angle-encoder-degree");

    row.querySelector(".angle-motor-label").textContent = `Motor ${i + 1}`;
    countsPerDegreeInput.addEventListener("change", () => {
      const value = Number(countsPerDegreeInput.value);
      angleConfigs[i].countsPerDegree = Number.isFinite(value) && value > 0 ? value : 0;
      syncAngleRow(i);
      syncMotorCard(i);
    });

    angleTableBody.appendChild(node);
    angleRows.push({ countsPerDegreeInput, encoderDegreeCell });
  }
}

function getTargetUnit() {
  if (targetUnitSelect.value === "degree") {
    return "degree";
  }
  if (targetUnitSelect.value === "radian") {
    return "radian";
  }
  return "count";
}

function getTrajectoryUnit() {
  if (trajectoryUnitSelect.value === "degree") {
    return "degree";
  }
  if (trajectoryUnitSelect.value === "radian") {
    return "radian";
  }
  return "count";
}

function getCountsPerDegree(index) {
  const value = angleConfigs[index]?.countsPerDegree ?? 0;
  return Number.isFinite(value) && value > 0 ? value : 0;
}

function canUseDegree(index) {
  return getCountsPerDegree(index) > 0;
}

function convertCountToDegree(index, count) {
  const countsPerDegree = getCountsPerDegree(index);
  if (countsPerDegree <= 0) {
    return null;
  }
  return count / countsPerDegree;
}

function convertDegreeToCount(index, degree) {
  const countsPerDegree = getCountsPerDegree(index);
  if (countsPerDegree <= 0) {
    return null;
  }
  return Math.round(degree * countsPerDegree);
}

function formatDegree(value) {
  if (!Number.isFinite(value)) {
    return "N/A";
  }
  return value.toFixed(3);
}

function formatRadian(value) {
  if (!Number.isFinite(value)) {
    return "N/A";
  }
  return value.toFixed(4);
}

function getDisplayTarget(index) {
  if (getTargetUnit() === "degree") {
    const degree = convertCountToDegree(index, targets[index]);
    return degree === null ? "" : formatDegree(degree);
  }
  if (getTargetUnit() === "radian") {
    const degree = convertCountToDegree(index, targets[index]);
    return degree === null ? "" : formatRadian(degree * Math.PI / 180);
  }
  return String(targets[index]);
}

function getDisplayJogStep() {
  const value = Number(jogStepInput.value);
  if (Number.isFinite(value) && value > 0) {
    return value;
  }
  if (getTargetUnit() === "degree") {
    return 1;
  }
  if (getTargetUnit() === "radian") {
    return 0.05;
  }
  return 500;
}

function convertDisplayToCount(index, value) {
  if (!Number.isFinite(value)) {
    return 0;
  }

  if (getTargetUnit() === "degree") {
    const converted = convertDegreeToCount(index, value);
    if (converted === null) {
      return null;
    }
    return converted;
  }

  if (getTargetUnit() === "radian") {
    const converted = convertDegreeToCount(index, value * 180 / Math.PI);
    if (converted === null) {
      return null;
    }
    return converted;
  }

  return Math.round(value);
}

function getSliderConfig(index) {
  if (getTargetUnit() === "degree") {
    const countsPerDegree = getCountsPerDegree(index);
    if (countsPerDegree > 0) {
      return {
        min: -COUNT_LIMIT / countsPerDegree,
        max: COUNT_LIMIT / countsPerDegree,
        step: 0.1,
        value: convertCountToDegree(index, targets[index]) ?? 0
      };
    }
  }

  if (getTargetUnit() === "radian") {
    const countsPerDegree = getCountsPerDegree(index);
    if (countsPerDegree > 0) {
      const minDegree = -COUNT_LIMIT / countsPerDegree;
      const maxDegree = COUNT_LIMIT / countsPerDegree;
      return {
        min: minDegree * Math.PI / 180,
        max: maxDegree * Math.PI / 180,
        step: 0.01,
        value: (convertCountToDegree(index, targets[index]) ?? 0) * Math.PI / 180
      };
    }
  }

  return {
    min: -COUNT_LIMIT,
    max: COUNT_LIMIT,
    step: 100,
    value: targets[index]
  };
}

function renderEncoderTable() {
  encoderTableBody.innerHTML = "";
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    const row = document.createElement("tr");
    row.innerHTML = `
      <td>Motor ${i + 1}</td>
      <td id="target-cell-${i}">${targets[i]}</td>
      <td id="encoder-cell-${i}">${encoderState[i]}</td>
      <td id="encoder-degree-cell-${i}">${formatDegree(convertCountToDegree(i, encoderState[i]))}</td>
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
  const sliderConfig = getSliderConfig(index);
  motorCards[index].targetInput.value = getDisplayTarget(index);
  motorCards[index].targetInput.step = getTargetUnit() === "count"
    ? "1"
    : getTargetUnit() === "degree"
      ? "0.1"
      : "0.01";
  motorCards[index].targetSlider.min = String(sliderConfig.min);
  motorCards[index].targetSlider.max = String(sliderConfig.max);
  motorCards[index].targetSlider.step = String(sliderConfig.step);
  motorCards[index].targetSlider.value = String(sliderConfig.value);
  motorCards[index].unitBadge.textContent = getTargetUnit() === "count"
    ? "Raw Count"
    : getTargetUnit() === "degree"
      ? "Degree"
      : "Radian";
  motorCards[index].encoderValue.textContent = String(encoder);
  motorCards[index].encoderDegreeValue.textContent = formatDegree(convertCountToDegree(index, encoder));
  motorCards[index].errorValue.textContent = String(target - encoder);
  motorCards[index].pwmValue.textContent = `${pwmState[index]} / ${pidConfigs[index].maxPwm}`;
  document.querySelector(`#target-cell-${index}`).textContent = getTargetUnit() === "count"
    ? String(target)
    : getTargetUnit() === "degree"
      ? `${getDisplayTarget(index)} deg`
      : `${getDisplayTarget(index)} rad`;
  document.querySelector(`#encoder-cell-${index}`).textContent = String(encoder);
  document.querySelector(`#encoder-degree-cell-${index}`).textContent = formatDegree(convertCountToDegree(index, encoder));
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
  row.motorInvert.checked = Boolean(config.motorInvert);
  row.encoderInvert.checked = Boolean(config.encoderInvert);
}

function syncAllPidRows() {
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    syncPidRow(i);
  }
}

function syncAngleRow(index) {
  angleRows[index].countsPerDegreeInput.value = angleConfigs[index].countsPerDegree > 0
    ? String(angleConfigs[index].countsPerDegree)
    : "";
  angleRows[index].encoderDegreeCell.textContent = formatDegree(convertCountToDegree(index, encoderState[index]));
}

function syncAllAngleRows() {
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    syncAngleRow(i);
  }
}

function clonePidConfigs(configs) {
  return configs.map((config) => ({
    kp: Number(config.kp),
    ki: Number(config.ki),
    kd: Number(config.kd),
    holdPwm: Number(config.holdPwm),
    holdMs: Number(config.holdMs),
    deadband: Number(config.deadband),
    backlash: Number(config.backlash),
    maxPwm: Number(config.maxPwm),
    motorInvert: Boolean(config.motorInvert),
    encoderInvert: Boolean(config.encoderInvert)
  }));
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
    maxPwm: Number(row.maxPwm.value),
    motorInvert: row.motorInvert.checked,
    encoderInvert: row.encoderInvert.checked
  };
}

function readAllPidRows() {
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    readPidRow(i);
  }
}

function loadPidPresets() {
  try {
    const raw = localStorage.getItem(PID_PRESET_STORAGE_KEY);
    if (!raw) {
      pidPresets = [];
      return;
    }

    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) {
      pidPresets = [];
      return;
    }

    pidPresets = parsed
      .filter((preset) =>
        preset &&
        typeof preset.name === "string" &&
        Array.isArray(preset.configs) &&
        preset.configs.length === MOTOR_COUNT
      )
      .map((preset) => ({
        name: preset.name,
        savedAt: typeof preset.savedAt === "string" ? preset.savedAt : new Date().toISOString(),
        configs: clonePidConfigs(preset.configs)
      }));
  } catch (error) {
    pidPresets = [];
    appendSerialLog(`[warn] preset load failed: ${error.message}`);
  }
}

function persistPidPresets() {
  localStorage.setItem(PID_PRESET_STORAGE_KEY, JSON.stringify(pidPresets));
}

function renderPidPresetOptions() {
  const previousValue = presetSelect.value;
  presetSelect.innerHTML = "";

  const placeholder = document.createElement("option");
  placeholder.value = "";
  placeholder.textContent = pidPresets.length === 0 ? "No saved preset" : "Select a preset";
  presetSelect.appendChild(placeholder);

  for (const preset of pidPresets) {
    const option = document.createElement("option");
    option.value = preset.name;
    option.textContent = `${preset.name} (${formatPresetTime(preset.savedAt)})`;
    presetSelect.appendChild(option);
  }

  if (pidPresets.some((preset) => preset.name === previousValue)) {
    presetSelect.value = previousValue;
  }
}

function formatPresetTime(isoText) {
  const date = new Date(isoText);
  if (Number.isNaN(date.getTime())) {
    return "time unknown";
  }
  return date.toLocaleString("zh-TW", { hour12: false });
}

function saveCurrentPidPreset() {
  const presetName = presetNameInput.value.trim();
  if (!presetName) {
    alert("Please enter a preset name first.");
    return;
  }

  readAllPidRows();
  const existingIndex = pidPresets.findIndex((preset) => preset.name === presetName);
  if (existingIndex >= 0) {
    const confirmed = confirm(`Preset "${presetName}" already exists. Overwrite it?`);
    if (!confirmed) {
      return;
    }
  }

  const record = {
    name: presetName,
    savedAt: new Date().toISOString(),
    configs: clonePidConfigs(pidConfigs)
  };

  if (existingIndex >= 0) {
    pidPresets[existingIndex] = record;
  } else {
    pidPresets.push(record);
  }

  pidPresets.sort((a, b) => a.name.localeCompare(b.name, "zh-Hant"));
  persistPidPresets();
  renderPidPresetOptions();
  presetSelect.value = presetName;
  pidStatus.textContent = `PID preset "${presetName}" saved locally in this browser.`;
}

function loadSelectedPidPreset() {
  const presetName = presetSelect.value;
  if (!presetName) {
    alert("Please select a saved preset first.");
    return;
  }

  const preset = pidPresets.find((item) => item.name === presetName);
  if (!preset) {
    alert("Selected preset was not found.");
    return;
  }

  const cloned = clonePidConfigs(preset.configs);
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    pidConfigs[i] = cloned[i];
  }
  syncAllPidRows();
  syncAllCards();
  presetNameInput.value = preset.name;
  pidStatus.textContent = `PID preset "${preset.name}" loaded into the table. Press Apply All to send it to STM32.`;
}

function deleteSelectedPidPreset() {
  const presetName = presetSelect.value;
  if (!presetName) {
    alert("Please select a saved preset to delete.");
    return;
  }

  const confirmed = confirm(`Delete PID preset "${presetName}"?`);
  if (!confirmed) {
    return;
  }

  pidPresets = pidPresets.filter((preset) => preset.name !== presetName);
  persistPidPresets();
  renderPidPresetOptions();
  if (presetNameInput.value.trim() === presetName) {
    presetNameInput.value = "";
  }
  pidStatus.textContent = `PID preset "${presetName}" deleted.`;
}

function loadAngleConfigs() {
  try {
    const raw = localStorage.getItem(ANGLE_CONFIG_STORAGE_KEY);
    if (!raw) {
      return;
    }

    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed) || parsed.length !== MOTOR_COUNT) {
      return;
    }

    for (let i = 0; i < MOTOR_COUNT; i += 1) {
      const value = Number(parsed[i]?.countsPerDegree);
      angleConfigs[i].countsPerDegree = Number.isFinite(value) && value > 0
        ? value
        : DEFAULT_COUNTS_PER_DEGREE[i];
    }
  } catch (error) {
    appendSerialLog(`[warn] angle config load failed: ${error.message}`);
  }
}

function persistAngleConfigs() {
  localStorage.setItem(ANGLE_CONFIG_STORAGE_KEY, JSON.stringify(angleConfigs));
}

function saveAngleConfigs() {
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    const value = Number(angleRows[i].countsPerDegreeInput.value);
    angleConfigs[i].countsPerDegree = Number.isFinite(value) && value > 0 ? value : 0;
  }
  persistAngleConfigs();
  syncAllAngleRows();
  syncAllCards();
  angleStatus.textContent = "Angle calibration saved in this browser.";
}

function resetAngleConfigs() {
  const confirmed = confirm("Reset all counts/degree values back to the default theoretical values?");
  if (!confirmed) {
    return;
  }

  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    angleConfigs[i].countsPerDegree = DEFAULT_COUNTS_PER_DEGREE[i];
  }
  persistAngleConfigs();
  syncAllAngleRows();
  syncAllCards();
  angleStatus.textContent = "Angle calibration reset to the default theoretical values.";
}

function setTarget(index, value, autoSend = true) {
  const safeValue = Number.isFinite(value) ? Math.round(value) : 0;
  targets[index] = safeValue;
  if (autoSend) {
    maybeAutoSend();
  }
}

function clampTarget(value) {
  return Math.max(-COUNT_LIMIT, Math.min(COUNT_LIMIT, Math.round(value)));
}

function handleTargetUnitChange() {
  if (getTargetUnit() === "degree" || getTargetUnit() === "radian") {
    const hasMissingCalibration = angleConfigs.some((config) => !(config.countsPerDegree > 0));
    if (hasMissingCalibration) {
      angleStatus.textContent = `${getTargetUnit() === "degree" ? "Degree" : "Radian"} mode selected. Please fill counts/degree for every motor.`;
      if (!jogStepInput.value || Number(jogStepInput.value) > 50) {
        jogStepInput.value = getTargetUnit() === "degree" ? "1" : "0.05";
      }
    }
  } else if (!jogStepInput.value || Number(jogStepInput.value) < 5) {
    jogStepInput.value = "500";
  }

  syncAllCards();
  renderEncoderTable();
  syncAllCards();
}

async function connectSerial() {
  if (!("serial" in navigator)) {
    alert("Web Serial is not available. Please use Microsoft Edge or Chrome.");
    return;
  }

  if (port) {
    appendSerialLog("[system] Serial is already connected.");
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
    syncAngleRow(i);
  }

  if (isLogging) {
    pushLogSample(parsed.source, line);
  }
}

function parseConfigLine(line) {
  const match = line.match(/^CFG,(\d+),([-\d.]+),([-\d.]+),([-\d.]+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),([01]),([01])$/);
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
    maxPwm: Number(match[9]),
    motorInvert: match[10] === "1",
    encoderInvert: match[11] === "1"
  };
  syncPidRow(motorIndex);
  syncMotorCard(motorIndex);
  pidStatus.textContent = `Motor ${motorIndex + 1} config loaded.`;
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
      appendSerialLog("[warn] Serial is not connected. Target frame was not sent.");
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
    appendSerialLog("[warn] Serial is not connected. Command was not sent.");
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
  await sendTextCommand(
    `!MOTORINV,${index},${cfg.motorInvert ? 1 : 0}`,
    `motor-invert-${index}`
  );
  await sendTextCommand(
    `!ENCINV,${index},${cfg.encoderInvert ? 1 : 0}`,
    `encoder-invert-${index}`
  );
  pidStatus.textContent = `Motor ${index + 1} config applied.`;
}

async function applyAllConfigs() {
  for (let i = 0; i < MOTOR_COUNT; i += 1) {
    await applyConfig(i);
  }
  pidStatus.textContent = "All motor configs applied.";
}

function startLogging() {
  isLogging = true;
  if (!sessionStart) {
    sessionStart = new Date();
  }
  logStatus.textContent = "Encoder logging is active. Export CSV after the test.";
}

function stopLogging() {
  isLogging = false;
  logStatus.textContent = `Logging stopped. ${logSamples.length} samples collected.`;
}

function clearLogs() {
  logSamples.length = 0;
  rawLines.length = 0;
  serialLog.textContent = "Waiting for serial data...";
  updateLogSummary();
  logStatus.textContent = "Logs cleared.";
}

function exportCsv() {
  if (logSamples.length === 0) {
    alert("No encoder samples yet. Start logging first, then export CSV.");
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
    alert("No raw serial log yet. Connect the board and collect some data first.");
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
      trajectoryStatus.textContent = `Loaded ${trajectoryPoints.length} trajectory points from ${file.name}.`;
    } catch (error) {
      trajectoryPoints = [];
      trajectoryIndex = 0;
      updateTrajectorySummary();
      trajectoryStatus.textContent = `Trajectory parse failed: ${error.message}`;
    }
  });
}

function parseTrajectoryText(text) {
  const unit = getTrajectoryUnit();
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
        throw new Error(`Line ${index + 1} must contain exactly ${MOTOR_COUNT} numbers.`);
      }

      if (unit === "degree") {
        return tokens.map((value, motorIndex) => {
          const converted = convertDegreeToCount(motorIndex, value);
          if (converted === null) {
            throw new Error(`Motor ${motorIndex + 1} is missing counts/degree for degree trajectory input.`);
          }
          return converted;
        });
      }

      if (unit === "radian") {
        return tokens.map((value, motorIndex) => {
          const converted = convertDegreeToCount(motorIndex, value * 180 / Math.PI);
          if (converted === null) {
            throw new Error(`Motor ${motorIndex + 1} is missing counts/degree for radian trajectory input.`);
          }
          return converted;
        });
      }

      return tokens.map((value) => Math.round(value));
    });

  if (points.length === 0) {
    throw new Error("Trajectory file is empty.");
  }
  return points;
}

function playTrajectory() {
  if (trajectoryPoints.length === 0) {
    alert("Load a trajectory file first.");
    return;
  }

  stopTrajectory();
  trajectoryIndex = 0;
  const intervalMs = Math.max(1, Math.round(Number(trajectoryIntervalMsInput.value) || 5));
  trajectoryStatus.textContent = `Trajectory playback started at ${intervalMs} ms per point.`;

  trajectoryTimer = setInterval(() => {
    if (trajectoryIndex >= trajectoryPoints.length) {
      stopTrajectory();
      trajectoryStatus.textContent = `Trajectory playback finished after ${trajectoryPoints.length} points.`;
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
  trajectoryStatus.textContent = "Trajectory cleared.";
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



