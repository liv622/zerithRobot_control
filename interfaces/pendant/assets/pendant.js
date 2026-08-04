"use strict";

const axes = ["X", "Y", "Z", "Rx", "Ry", "Rz"];
let jointNames = [];
let auxNames = [];
let state = null;
let editing = false;
let pending = 0;
let teachPage = 0;
let selectedPointId = null;
let activeHold = null;
let holdSerial = 0;
let robotControlsBuilt = false;
let activePage = "home";
let polling = false;
const $ = id => document.getElementById(id);

function buildStaticControls() {
  $("targetFields").innerHTML = axes.map((axis, index) => `
    <div class="target-field"><label>${axis} (${index < 3 ? "m" : "deg"})</label>
    <input id="t${index}" type="number" step="${index < 3 ? ".001" : ".1"}"></div>
  `).join("");
  $("cartJog").innerHTML = axes.map((axis, index) => `
    <div class="jog-axis">
      <button ${holdEvents("cartesian", index, -1)}>−</button>
      <button class="axis">${axis}</button>
      <button ${holdEvents("cartesian", index, 1)}>＋</button>
    </div>
  `).join("");
  document.querySelectorAll(".nav").forEach(button => {
    button.addEventListener("click", () => showPage(button.dataset.page));
  });
  document.querySelectorAll(".move-tabs button").forEach(button => {
    button.addEventListener("click", () => showMoveMode(button.dataset.mode));
  });
  document.addEventListener("focusin", event => {
    if (event.target.matches("input,select")) editing = true;
  });
  document.addEventListener("focusout", () => {
    setTimeout(() => { editing = false; }, 250);
  });
}

function holdEvents(mode, index, direction) {
  return `onpointerdown="startHold(event,'${mode}',${index},${direction})" ` +
    `onpointerup="endHold(event)" onpointerleave="endHold(event)" ` +
    `onpointercancel="endHold(event)"`;
}

function buildRobotControls(robot) {
  jointNames = robot.arm_joint_names;
  auxNames = robot.aux_joint_names;
  $("jointList").innerHTML = jointNames.map((name, index) => `
    <div class="joint">
      <b>J${index + 1}</b>
      <button ${holdEvents("joint", index, -1)}>−</button>
      <span id="j${index}" class="value">0.00°</span>
      <button ${holdEvents("joint", index, 1)}>＋</button>
    </div>
  `).join("");
  $("nullJointList").innerHTML = jointNames.map((name, index) => `
    <div class="joint">
      <b>J${index + 1}</b>
      <span id="nj${index}" class="value">0.00°</span>
    </div>
  `).join("");
  $("auxList").innerHTML = auxNames.map((name, index) => `
    <div class="joint">
      <b>${escapeHtml(robot.auxiliary_labels[name] || name)}</b>
      <button onclick="jogAux('${escapeHtml(name)}',-1)">−</button>
      <span id="a${index}" class="value">0.000</span>
      <button onclick="jogAux('${escapeHtml(name)}',1)">＋</button>
    </div>
  `).join("");
  robotControlsBuilt = true;
}

function showPage(name) {
  if (name === activePage) return;
  const leavingMove = activePage === "move" && name !== "move";
  if (leavingMove) endHold();
  activePage = name;
  document.querySelector(".content-stage").classList.toggle("move-active", name === "move");
  document.querySelectorAll(".page").forEach(page => page.classList.remove("active"));
  document.querySelectorAll(".nav").forEach(button => button.classList.toggle("active", button.dataset.page === name));
  $(`page-${name}`).classList.add("active");
  if (name === "move") loadSimulationFrame();
  if (state) render(state);
}

function loadSimulationFrame() {
  const frame = $("viserFrame");
  if (frame.getAttribute("src") === "about:blank") {
    frame.src = frame.dataset.src;
  }
}

function showMoveMode(name) {
  document.querySelectorAll(".move-mode").forEach(panel => panel.classList.remove("active"));
  document.querySelectorAll(".move-tabs button").forEach(button => button.classList.toggle("active", button.dataset.mode === name));
  $(`move-${name}`).classList.add("active");
}

async function api(path, body) {
  const options = body ? {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body),
  } : {};
  const response = await fetch(path, options);
  const data = await response.json();
  if (!response.ok || !data.ok) throw Error(data.error || `HTTP ${response.status}`);
  return data;
}

async function cmd(action, extra = {}) {
  pending++;
  try {
    const result = await api("/api/command", {action, ...extra});
    setMessage(result.message || "命令已执行");
    await poll();
    return result;
  } catch (error) {
    setMessage(error.message, true);
    return null;
  } finally {
    pending--;
  }
}

function setMessage(text, bad = false) {
  $("message").textContent = text;
  $("message").className = `message${bad ? " bad" : ""}`;
  $("diagnosticMessage").textContent = `${new Date().toLocaleTimeString("zh-CN", {hour12: false})}　${text}`;
}

function cartesianStep() {
  return Math.max(.1, Math.min(30, Number($("cartStep").value) || 10));
}

function jointStep() {
  return Math.max(.1, Math.min(30, Number($("jointStep").value) || 5));
}

function jogStep(mode) {
  if (mode === "cartesian") return cartesianStep();
  if (mode === "joint") return jointStep();
  if (mode === "nullspace") {
    return Math.max(.1, Math.min(30, Number($("nullStep").value) || 5));
  }
  return cartesianStep();
}

function stepModeEnabled(mode) {
  const id = mode === "cartesian" ? "cartStepMode" : (
    mode === "joint" ? "jointStepMode" : "nullStepMode"
  );
  return $(id).checked;
}

async function startHold(event, mode, index, direction) {
  if (event.button !== undefined && event.button !== 0) return;
  event.preventDefault();
  const extra = {mode, direction, step: jogStep(mode)};
  if (mode === "cartesian") extra.axis = index;
  else if (mode === "joint") extra.joint = jointNames[index];
  if (stepModeEnabled(mode)) {
    await cmd("jog_step", extra);
    return;
  }
  const serial = ++holdSerial;
  activeHold = serial;
  try { event.currentTarget.setPointerCapture?.(event.pointerId); } catch (_) {}
  await cmd("start_continuous_jog", extra);
  if (activeHold !== serial) cmd("stop_continuous_jog");
}

function endHold(event) {
  event?.preventDefault?.();
  if (activeHold === null) return;
  activeHold = null;
  cmd("stop_continuous_jog");
}

function jogAux(name, direction) {
  cmd("jog_aux", {joint: name, delta: direction * cartesianStep() / 1000});
}

function connectHardware() {
  cmd("connect_hardware", {ip: $("hardwareIp").value});
}

function values(prefix, count) {
  return Array.from({length: count}, (_, index) => Number($(prefix + index).value));
}

function applyTarget() {
  cmd("set_target", {values: values("t", 6)});
}

function solverPayload() {
  return {
    live: $("live").checked,
    orientation_lock: $("orient").checked,
    auto_recovery: $("recovery").checked,
    recovery_count: Number($("seeds").value),
    guide_enabled: $("guideOn").checked,
    guide_strength: Number($("strength").value),
  };
}

function applySolverSettings() {
  cmd("settings", solverPayload());
}

function setDrag() {
  if (!state || $("dragToggle").checked !== state.drag_unlocked) cmd("toggle_drag");
}

async function applyMotionSettings() {
  await cmd("motion_settings", {
    speed_percent: Number($("speedPercent").value),
    max_linear_speed_mm_s: Number($("maxLinear").value),
    max_angular_speed_deg_s: Number($("maxAngular").value),
    max_joint_speed_deg_s: Number($("maxJoint").value),
    command_delay_s: Number($("commandDelay").value),
  });
  await cmd("set_teach_program_settings", {
    duration: Number($("configDuration").value),
    frequency: Number($("configFrequency").value),
    loop: $("loopProgram").checked,
  });
  await cmd("settings", solverPayload());
}

async function speedPreset(value) {
  $("speedPercent").value = value;
  await applyMotionSettings();
}

async function adjustSpeed(delta) {
  const current = state?.settings.speed_percent || 30;
  $("speedPercent").value = Math.max(1, Math.min(100, current + delta));
  await applyMotionSettings();
}

async function saveProfile() {
  const name = $("profileName").value.trim();
  if (!name) {
    setMessage("请输入配置文件名称", true);
    return;
  }
  await applyMotionSettings();
  const result = await cmd("save_configuration", {name});
  if (result) $("profileName").value = "";
}

function loadProfile(name) {
  cmd("load_configuration", {name});
}

function deleteProfile(name) {
  cmd("delete_configuration", {name});
}

function renderProfiles(configuration) {
  const active = configuration.active;
  const label = active ? `当前：${active}` : "未调用配置";
  $("activeProfile").textContent = label;
  $("activeProfileHome").textContent = label;
  $("profileList").innerHTML = configuration.profiles.length
    ? configuration.profiles.map(name => `
      <div class="profile-item ${name === active ? "active" : ""}">
        <b>${escapeHtml(name)}</b>
        <button onclick="loadProfile('${escapeJs(name)}')">调用</button>
        <button class="danger" onclick="deleteProfile('${escapeJs(name)}')">删除</button>
      </div>`).join("")
    : `<p class="hint">尚未保存配置文件</p>`;
}

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, character => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
  })[character]);
}

function escapeJs(value) {
  return String(value).replace(/\\/g, "\\\\").replace(/'/g, "\\'");
}

function pointById(id) {
  return state?.teach_program.points.find(point => point.point_id === id);
}

function pointCode(point) {
  return `P${String(point.point_id).padStart(3, "0")}`;
}

function jointSummary(point) {
  return point.joint_values.map(value => Number(value).toFixed(1)).join("  ");
}

function cartesianSummary(point) {
  return point.cartesian_values.map((value, index) =>
    Number(value).toFixed(index < 3 ? 3 : 1)
  ).join("  ");
}

function renderTeachTable() {
  const points = state?.teach_program.points || [];
  const pageSize = 7;
  const pageCount = Math.max(1, Math.ceil(points.length / pageSize));
  teachPage = Math.max(0, Math.min(teachPage, pageCount - 1));
  const shown = points.slice(teachPage * pageSize, teachPage * pageSize + pageSize);
  let html = `<div class="teach-head"><span>选</span><span>点位</span><span>指令</span><span>关节角度 (deg)</span><span>笛卡尔位姿 (m / deg)</span><span>运动</span><span>编辑</span></div>`;
  html += shown.map(point => `
    <div class="teach-point ${point.point_id === selectedPointId ? "selected" : ""}" onclick="selectPoint(${point.point_id})">
      <input type="checkbox" ${point.checked ? "checked" : ""} onclick="event.stopPropagation();togglePoint(${point.point_id},this.checked)">
      <span><b>${pointCode(point)}</b><small class="point-name">${escapeHtml(point.name)}</small></span>
      <select class="point-motion ${point.motion_type.toLowerCase()}" onclick="event.stopPropagation()" onchange="changePointMotion(${point.point_id},this.value)"><option value="MOVL" ${point.motion_type === "MOVL" ? "selected" : ""}>MOVL</option><option value="MOVJ" ${point.motion_type === "MOVJ" ? "selected" : ""}>MOVJ</option></select>
      <span class="teach-data" title="${escapeHtml(jointSummary(point))}">${escapeHtml(jointSummary(point))}</span>
      <span class="teach-data" title="${escapeHtml(cartesianSummary(point))}">${escapeHtml(cartesianSummary(point))}</span>
      <button class="run" onclick="event.stopPropagation();moveTeachPoint(${point.point_id})">运动</button>
      <button onclick="event.stopPropagation();selectPoint(${point.point_id})">修改</button>
    </div>`).join("");
  for (let index = shown.length; index < pageSize; index++) {
    html += `<div class="teach-point"><span></span><span class="point-name">--</span></div>`;
  }
  $("teachTable").innerHTML = html;
  $("pageInfo").textContent = points.length ? `${teachPage + 1} / ${pageCount}` : "0 / 0";
  renderPointEditor();
}

function changePage(delta) {
  teachPage += delta;
  renderTeachTable();
}

function selectPoint(id) {
  selectedPointId = id;
  renderTeachTable();
}

function togglePoint(id, checked) {
  cmd("set_teach_point_checked", {point_id: id, checked});
}

function pointPayload(point, motionType = point.motion_type) {
  return {
    point_id: point.point_id,
    name: point.name,
    motion_type: motionType,
    joint_values: point.joint_values,
    cartesian_values: point.cartesian_values,
  };
}

function changePointMotion(id, motionType) {
  const point = pointById(id);
  if (point) cmd("update_teach_point", pointPayload(point, motionType));
}

function moveTeachPoint(id) {
  cmd("move_teach_point", {point_id: id});
}

function renderPointEditor() {
  const point = pointById(selectedPointId);
  if (!point) {
    selectedPointId = null;
    $("editorHint").textContent = "--";
    $("editName").value = "";
    $("editName").disabled = true;
    $("pointEditor").innerHTML = `<p class="hint">从左侧选择示教点</p>`;
    return;
  }
  $("editorHint").textContent = "关节角度 + 笛卡尔位姿";
  $("editName").disabled = false;
  $("editName").value = point.name;
  const joints = point.joint_values.map((value, index) => `<div class="editor-field"><label>J${index + 1}</label><input id="editJoint${index}" type="number" value="${Number(value)}" step=".1"></div>`).join("");
  const cartesian = point.cartesian_values.map((value, index) => `<div class="editor-field"><label>${axes[index]}</label><input id="editCartesian${index}" type="number" value="${Number(value)}" step="${index < 3 ? ".001" : ".1"}"></div>`).join("");
  $("pointEditor").innerHTML = `<div class="editor-section"><b>关节角度</b><div class="editor-grid">${joints}</div></div><div class="editor-section"><b>笛卡尔位姿</b><div class="editor-grid">${cartesian}</div></div>`;
}

function saveCurrentPoint() {
  cmd("save_teach_point", {
    motion_type: $("newPointType").value,
    name: $("newPointName").value,
  }).then(result => {
    if (result) $("newPointName").value = "";
  });
}

function updatePoint() {
  const point = pointById(selectedPointId);
  if (!point) {
    setMessage("请先选择要修改的示教点", true);
    return;
  }
  cmd("update_teach_point", {
    point_id: point.point_id,
    name: $("editName").value,
    motion_type: point.motion_type,
    joint_values: Array.from({length: jointNames.length}, (_, index) => Number($(`editJoint${index}`).value)),
    cartesian_values: Array.from({length: 6}, (_, index) => Number($(`editCartesian${index}`).value)),
  });
}

function deletePoint() {
  const point = pointById(selectedPointId);
  if (!point) {
    setMessage("请先选择示教点", true);
    return;
  }
  cmd("delete_teach_point", {point_id: point.point_id});
  selectedPointId = null;
}

function runPoints() {
  cmd("run_teach_points", {
    duration: Number($("duration").value),
    frequency: Number($("frequency").value),
    loop: $("loopProgram").checked,
  });
}

function stopAll() {
  endHold();
  cmd("stop_teach_points");
}

function renderHomeJoints(arm) {
  $("homeJoints").innerHTML = arm.map((value, index) => {
    const normalized = Math.max(0, Math.min(100, 50 + value / 7.2));
    return `<div class="joint-bar"><b>J${index + 1}</b><span>${value.toFixed(2)}°</span><i style="--value:${normalized}%"></i></div>`;
  }).join("");
}

function render(nextState) {
  state = nextState;
  if (!robotControlsBuilt) buildRobotControls(state.robot);
  $("lamp").className = "lamp on";
  $("connection").textContent = "仿真在线";
  const hardware = state.hardware || {};
  const hardwareText = !hardware.connected ? "右臂未连接" : (
    hardware.enabled ? "右臂已上使能" : "右臂已连接·下使能"
  );
  $("hardwareState").textContent = hardware.enabled
    ? `${hardwareText} · ${hardware.control_mode || "PD 前馈"}`
    : hardwareText;
  $("backend").textContent = `IK ${state.backend}`;
  $("reach").textContent = state.reachable === null ? "等待" : (state.reachable ? "可达" : "不可达");
  $("reach").style.color = state.reachable === false ? "var(--red)" : "var(--green)";
  $("homeStatus").textContent = state.program_status;
  $("programStatus").textContent = state.program_status;
  $("speedBadge").textContent = `${state.settings.speed_percent.toFixed(0)}%`;
  $("footerSpeed").textContent = `${state.settings.speed_percent.toFixed(0)}%`;
  $("poserr").textContent = state.position_error_mm?.toFixed(3) ?? "--";
  $("orierr").textContent = state.orientation_error_degrees?.toFixed(3) ?? "--";
  $("attempts").textContent = state.attempts ?? "--";
  $("dragState").textContent = state.drag_unlocked ? "解锁" : "锁定";
  if (editing || pending) return;

  if (hardware.ip) $("hardwareIp").value = hardware.ip;

  const pose = [...state.target.position_m, ...state.target.rpy_degrees];
  pose.forEach((value, index) => {
    $(`t${index}`).value = value.toFixed(index < 3 ? 4 : 2);
    $(`h${axes[index].toLowerCase()}`).textContent = value.toFixed(index < 3 ? 4 : 2);
  });
  state.arm_degrees.forEach((value, index) => {
    $(`j${index}`).textContent = `${value.toFixed(2)}°`;
    $(`nj${index}`).textContent = `${value.toFixed(2)}°`;
  });
  $("nullLamp").className = state.null_space_active ? "on" : "";
  $("nullState").textContent = state.null_space_active ? "TCP 已锁定 · 运动中" : "等待点动";
  auxNames.forEach((name, index) => $(`a${index}`).textContent = state.auxiliary[name].toFixed(3));
  renderHomeJoints(state.arm_degrees);

  const settings = state.settings;
  $("live").checked = settings.live;
  $("orient").checked = settings.orientation_lock;
  $("recovery").checked = settings.auto_recovery;
  $("dragToggle").checked = state.drag_unlocked;
  $("seeds").value = settings.recovery_count;
  $("guideOn").checked = settings.guide_enabled;
  $("strength").value = settings.guide_strength;
  $("speedPercent").value = settings.speed_percent;
  $("maxLinear").value = settings.max_linear_speed_mm_s;
  $("maxAngular").value = settings.max_angular_speed_deg_s;
  $("maxJoint").value = settings.max_joint_speed_deg_s;
  $("commandDelay").value = settings.command_delay_s;
  $("duration").value = state.teach_program.duration;
  $("frequency").value = state.teach_program.frequency;
  $("configDuration").value = state.teach_program.duration;
  $("configFrequency").value = state.teach_program.frequency;
  $("loopProgram").checked = state.teach_program.loop;
  $("homeSpeed").textContent = `${settings.speed_percent.toFixed(0)}%`;
  $("homeLinear").textContent = `${settings.max_linear_speed_mm_s.toFixed(0)} mm/s`;
  $("homeJointSpeed").textContent = `${settings.max_joint_speed_deg_s.toFixed(0)} °/s`;
  $("homeDelay").textContent = `${settings.command_delay_s.toFixed(1)} s`;
  if (activePage === "home" || activePage === "config") {
    renderProfiles(state.configuration);
  }
  if (activePage === "program") renderTeachTable();
}

async function poll() {
  if (polling) return;
  polling = true;
  try {
    const data = await api("/api/state");
    render(data.state);
  } catch (error) {
    $("lamp").className = "lamp";
    $("connection").textContent = "仿真离线";
    setMessage(`通信失败：${error.message}`, true);
  } finally {
    polling = false;
  }
}

buildStaticControls();
setInterval(() => {
  $("clock").textContent = new Date().toLocaleString("zh-CN", {hour12: false});
}, 500);
window.addEventListener("pointerup", endHold);
window.addEventListener("pointercancel", endHold);
window.addEventListener("blur", endHold);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) endHold();
});
poll();
setInterval(poll, 350);
