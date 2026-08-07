"use strict";

/* ───────────────────────────────────────────────────────────────
   Oscilloscope — real-time joint-state chart (SSE + Canvas)

   Features
   ────────
   • SSE streaming from simulator (/api/oscilloscope/stream)
   • Pause / resume — freeze chart without losing connection
   • Export current buffers to CSV
   • Import CSV to render stored data (no SSE needed)
   ─────────────────────────────────────────────────────────────── */

var COLOURS = [
  "#58a6ff", "#3fb950", "#d2991d", "#f85149",
  "#bc8cff", "#79c0ff", "#ffa657", "#56d364",
];

var TYPES = [
  { key: "positions",      label: "位置",   unit: "rad",    yTitle: "位置 (rad)" },
  { key: "velocities",     label: "速度",   unit: "rad/s",  yTitle: "速度 (rad/s)" },
  { key: "accelerations",  label: "加速度", unit: "rad/s²", yTitle: "加速度 (rad/s²)" },
];

var MAX_POINTS = 800;
var Y_MARGIN   = 0.15;
var GRID_ROWS  = 5;
var GRID_COLS  = 10;

// ── state ─────────────────────────────────────────────────────

var jointCount = 0;
var streamUrl  = "";
var es         = null;
var connected  = false;
var frameCount = 0;
var avgDt      = 0;
var lastDt     = 0;
var paused     = false;       // freeze buffer writes
var imported   = false;       // true when showing imported CSV data

var buffers  = {};
var timeBuf  = [];
var visJoints = new Set();
var visTypes  = new Set(["positions", "velocities"]);
var canvases = {};
var ctxs     = {};
var rafId    = null;
var dbg      = [];

/* ───────────────────────────────────────────────────────────────
   DOM helpers
   ─────────────────────────────────────────────────────────────── */

function $(id) { return document.getElementById(id); }

function debug(msg) {
  dbg.push((new Date()).toLocaleTimeString() + "  " + msg);
  if (dbg.length > 12) dbg.shift();
  var el = $("debugOut");
  if (el) el.textContent = dbg.join(" | ");
}

/* ───────────────────────────────────────────────────────────────
   Bootstrap
   ─────────────────────────────────────────────────────────────── */

function init() {
  debug("init");
  streamUrl = new URLSearchParams(window.location.search).get("stream")
           || document.body.getAttribute("data-stream-url")
           || "";
  if (!streamUrl) { showPlaceholder("缺少 stream URL"); return; }
  if (window.self !== window.top) document.body.classList.add("embedded");
  buildPanels();
  connect();
  if (!visJoints.size) { for (var j = 0; j < 7; j++) visJoints.add(j); }
}

function buildPanels() {
  var html = "";
  for (var i = 0; i < TYPES.length; i++) {
    var t = TYPES[i];
    html += '<div class="chart-panel" id="panel-' + t.key + '">'
         + '<div class="label">' + t.label + ' (' + t.unit + ')</div>'
         + '<canvas id="canvas-' + t.key + '"></canvas></div>';
  }
  $("charts").innerHTML = html;
  for (var i = 0; i < TYPES.length; i++) {
    var key = TYPES[i].key;
    canvases[key] = $("canvas-" + key);
    if (canvases[key]) ctxs[key] = canvases[key].getContext("2d");
  }
  applyTypeVis();
}

function buildControls() {
  var html = '<div class="group"><span class="group-label">关节</span>';
  for (var j = 0; j < jointCount; j++) {
    var c = COLOURS[j % COLOURS.length];
    html += '<label style="color:' + c + '">'
         + '<input type="checkbox" data-j="' + j + '"' + (visJoints.has(j) ? " checked" : "") + '>J' + (j + 1)
         + '</label>';
  }
  html += '</div><div class="group"><span class="group-label">数据</span>';
  for (var i = 0; i < TYPES.length; i++) {
    var t = TYPES[i];
    html += '<label class="type-cb">'
         + '<input type="checkbox" data-t="' + t.key + '"' + (visTypes.has(t.key) ? " checked" : "") + '>' + t.label
         + '</label>';
  }
  html += '</div>';
  $("controls").innerHTML = html;

  $("controls").addEventListener("change", function(e) {
    var cb = e.target;
    if (!cb || cb.tagName !== "INPUT") return;
    if (cb.dataset.j !== undefined) {
      cb.checked ? visJoints.add(+cb.dataset.j) : visJoints.delete(+cb.dataset.j);
    }
    if (cb.dataset.t !== undefined) {
      cb.checked ? visTypes.add(cb.dataset.t) : visTypes.delete(cb.dataset.t);
      applyTypeVis();
    }
  });
}

function applyTypeVis() {
  for (var i = 0; i < TYPES.length; i++) {
    var p = $("panel-" + TYPES[i].key);
    if (p) p.style.display = visTypes.has(TYPES[i].key) ? "" : "none";
  }
}

function showPlaceholder(msg) {
  $("charts").innerHTML = '<div class="placeholder">' + msg + '</div>';
  setStatus(false);
  debug("ERROR: " + msg);
}

/* ───────────────────────────────────────────────────────────────
   SSE client
   ─────────────────────────────────────────────────────────────── */

function connect() {
  if (es) { es.close(); es = null; }
  setStatus(false);
  debug("connect…");
  try { es = new EventSource(streamUrl); }
  catch (e) { debug("EventSource FAIL: " + e.message); return; }

  es.addEventListener("open", function() {
    connected = true;
    setStatus(true);
    startLoop();
    debug("SSE open");
  });

  es.addEventListener("error", function() {
    connected = false;
    setStatus(false);
    if (es && es.readyState === 2) {
      debug("SSE closed, reconnect 3s…");
      setTimeout(function() { if (!connected) { es.close(); connect(); } }, 3000);
    }
  });

  es.addEventListener("message", function(e) {
    try {
      var msg = JSON.parse(e.data);
      msg.type === "init" ? handleInit(msg) : handleFrame(msg);
    } catch (_) {}
  });
}

function handleInit(msg) {
  jointCount = msg.joint_count || 0;
  debug("init: " + jointCount + " joints");
  if (!jointCount) return;
  if (imported) return; // don't reset manually imported data

  if (visJoints.size === 0) {
    for (var j = 0; j < Math.min(jointCount, 7); j++) visJoints.add(j);
  }
  allocBuffers();
  buildControls();
  updateLabels();
  $("pauseBtn").disabled = false;
  $("exportBtn").disabled = false;
}

function handleFrame(msg) {
  if (msg.t === undefined || paused) return;   // ← skip when paused

  imported = false;  // live data overrides import
  setPauseBadge(false);

  timeBuf.push(msg.t);
  if (timeBuf.length > MAX_POINTS) timeBuf.shift();
  if (msg.dt > 0) { lastDt = msg.dt; avgDt = avgDt ? avgDt * 0.95 + msg.dt * 0.05 : msg.dt; }

  for (var ti = 0; ti < TYPES.length; ti++) {
    var arr = msg[TYPES[ti].key];
    if (!arr || !Array.isArray(arr)) continue;
    var n = Math.min(arr.length, jointCount);
    for (var j = 0; j < n; j++) {
      var buf = buffers[TYPES[ti].key + ":" + j];
      if (!buf) continue;
      buf.push(arr[j]);
      if (buf.length > MAX_POINTS) buf.shift();
    }
  }
  frameCount++;
  if (frameCount % 30 === 0) { setStatus(true); updateLabels(); }
}

/* ───────────────────────────────────────────────────────────────
   Buffer allocation
   ─────────────────────────────────────────────────────────────── */

function allocBuffers() {
  buffers = {};
  for (var ti = 0; ti < TYPES.length; ti++) {
    for (var j = 0; j < jointCount; j++) {
      buffers[TYPES[ti].key + ":" + j] = [];
    }
  }
  timeBuf    = [];
  frameCount = 0;
  avgDt      = 0;
  lastDt     = 0;
}

/* ───────────────────────────────────────────────────────────────
   Pause / Resume
   ─────────────────────────────────────────────────────────────── */

function togglePause() {
  paused = !paused;
  var btn = $("pauseBtn");
  if (paused) {
    btn.textContent = "▶ 继续";
    btn.classList.add("paused");
  } else {
    btn.textContent = "⏸ 暂停";
    btn.classList.remove("paused");
  }
  setPauseBadge(paused);
  debug(paused ? "已暂停" : "已继续");
}

function setPauseBadge(show) {
  var badge = $("pauseBadge");
  if (badge) badge.style.display = show ? "" : "none";
}

/* ───────────────────────────────────────────────────────────────
   Export CSV
   ─────────────────────────────────────────────────────────────── */

function exportCSV() {
  if (!timeBuf.length) { debug("无数据可导出"); return; }

  // Header row
  var header = ["t"];
  for (var j = 0; j < jointCount; j++) {
    for (var ti = 0; ti < TYPES.length; ti++) {
      header.push("J" + (j + 1) + "_" + TYPES[ti].key);
    }
  }

  // Data rows — use shortest buffer length to stay aligned
  var rows = timeBuf.length;
  for (var j = 0; j < jointCount; j++) {
    for (var ti = 0; ti < TYPES.length; ti++) {
      var b = buffers[TYPES[ti].key + ":" + j];
      if (b && b.length < rows) rows = b.length;
    }
  }
  if (rows < 2) { debug("数据不足"); return; }

  var lines = [header.join(",")];
  // timeBuf may be longer or shorter — align from the end
  var tOff = timeBuf.length - rows;
  for (var i = 0; i < rows; i++) {
    var row = [timeBuf[tOff + i].toFixed(6)];
    var suffix = [];
    for (var jj = 0; jj < jointCount; jj++) {
      for (var ti2 = 0; ti2 < TYPES.length; ti2++) {
        var buf2 = buffers[TYPES[ti2].key + ":" + jj];
        var off2 = buf2.length - rows;
        var v = (buf2 && i + off2 >= 0) ? buf2[i + off2] : NaN;
        suffix.push(isFinite(v) ? v.toFixed(8) : "");
      }
    }
    row.push.apply(row, suffix);  // avoid stack overflow from .concat on large arrays
    lines.push(row.join(","));
  }

  var blob = new Blob([lines.join("\n")], { type: "text/csv;charset=utf-8" });
  var url  = URL.createObjectURL(blob);
  var a    = document.createElement("a");
  a.href     = url;
  a.download = "robot-scope-" + new Date().toISOString().replace(/[:.]/g, "-") + ".csv";
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
  debug("导出 " + rows + " 行 × " + header.length + " 列");
}

/* ───────────────────────────────────────────────────────────────
   Import CSV
   ─────────────────────────────────────────────────────────────── */

function importCSV(event) {
  var file = event.target.files && event.target.files[0];
  if (!file) return;

  var reader = new FileReader();
  reader.onload = function(e) {
    try {
      var text = e.target.result;
      var lines = text.split(/\r?\n/).filter(function(l) { return l.trim(); });
      if (lines.length < 2) { debug("CSV 行数不足"); return; }

      var header = lines[0].split(",").map(function(h) { return h.trim(); });
      // Parse: "t", "J1_positions", "J1_velocities", "J1_accelerations", "J2_positions", …
      // Build a column map: col_index → { joint, typeKey }
      var colMap = [];
      var maxJoint = 0;
      for (var c = 1; c < header.length; c++) {
        var m = header[c].match(/^J(\d+)_(positions|velocities|accelerations)$/);
        if (m) {
          var jIdx = parseInt(m[1], 10) - 1;
          if (jIdx > maxJoint) maxJoint = jIdx;
          colMap.push({ joint: jIdx, type: m[2] });
        } else {
          colMap.push(null);
        }
      }
      if (!colMap.length || maxJoint < 0) { debug("CSV 头无法解析"); return; }

      // Initialise for imported data
      jointCount = maxJoint + 1;
      allocBuffers();

      // Parse rows (limit to MAX_POINTS for display)
      var startRow = Math.max(1, lines.length - MAX_POINTS);
      var prevT = 0;
      for (var r = startRow; r < lines.length; r++) {
        var cols = lines[r].split(",");
        var t = parseFloat(cols[0]);
        if (!isFinite(t)) continue;

        timeBuf.push(t);
        if (r > startRow && timeBuf.length >= 2) {
          lastDt = t - prevT;
          avgDt  = avgDt ? avgDt * 0.9 + lastDt * 0.1 : lastDt;
        }
        prevT = t;

        for (var ci = 0; ci < colMap.length; ci++) {
          var map = colMap[ci];
          if (!map) continue;
          var val = parseFloat(cols[ci + 1]);
          if (!isFinite(val)) val = 0;
          var buf = buffers[map.type + ":" + map.joint];
          if (buf) buf.push(val);
        }

        if (timeBuf.length > MAX_POINTS) timeBuf.shift();
      }

      // Trim all buffers to MAX_POINTS
      for (var key in buffers) {
        if (buffers[key] && buffers[key].length > MAX_POINTS) {
          buffers[key] = buffers[key].slice(buffers[key].length - MAX_POINTS);
        }
      }

      imported  = true;
      paused    = false;
      setPauseBadge(false);
      var pbtn = $("pauseBtn");
      if (pbtn) { pbtn.textContent = "⏸ 暂停"; pbtn.classList.remove("paused"); pbtn.disabled = false; }
      $("exportBtn").disabled = false;

      // Default visibility: all joints
      visJoints.clear();
      for (var j = 0; j < Math.min(jointCount, 7); j++) visJoints.add(j);

      buildControls();
      applyTypeVis();
      updateLabels();
      setStatus(true);
      startLoop();
      debug("导入 " + timeBuf.length + " 行, " + jointCount + " 关节");
    } catch (err) {
      debug("CSV 解析失败: " + err.message);
    }
  };
  reader.readAsText(file);
  // Reset the file input so the same file can be re-imported
  event.target.value = "";
}

/* ───────────────────────────────────────────────────────────────
   Render loop
   ─────────────────────────────────────────────────────────────── */

function startLoop() {
  if (rafId) return;
  var lastDraw = 0, drawFps = 0;
  function draw() {
    rafId = requestAnimationFrame(draw);
    var now = performance.now();
    if (lastDraw) {
      var delta = now - lastDraw;
      drawFps = drawFps ? drawFps * 0.95 + (1000 / delta) * 0.05 : 1000 / delta;
      if (Math.floor(now / 300) !== Math.floor(lastDraw / 300)) {
        var fo = $("fpsOut");
        if (fo) fo.textContent = drawFps.toFixed(0) + " fps";
      }
    }
    lastDraw = now;
    paint();
  }
  rafId = requestAnimationFrame(draw);
  debug("loop started");
}

function paint() {
  if (timeBuf.length < 2) return;
  for (var ti = 0; ti < TYPES.length; ti++) {
    var t = TYPES[ti];
    if (!visTypes.has(t.key)) continue;
    var canvas = canvases[t.key];
    var ctx    = ctxs[t.key];
    if (!canvas || !ctx) continue;
    var rect = canvas.parentElement.getBoundingClientRect();
    var w = rect.width, h = rect.height;
    if (w < 1 || h < 1) continue;
    var dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.floor(w * dpr) || canvas.height !== Math.floor(h * dpr)) {
      canvas.width  = Math.floor(w * dpr);
      canvas.height = Math.floor(h * dpr);
      canvas.style.width  = w + "px";
      canvas.style.height = h + "px";
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    drawGrid(ctx, w, h);
    drawTraces(ctx, t, w, h);
  }
}

function drawGrid(ctx, w, h) {
  ctx.strokeStyle = "#21262d"; ctx.lineWidth = 0.5;
  for (var r = 0; r <= GRID_ROWS; r++) {
    var y = (h / GRID_ROWS) * r;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }
  for (var c = 0; c <= GRID_COLS; c++) {
    var x = (w / GRID_COLS) * c;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
  }
  ctx.strokeStyle = "#30363d"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();
}

function drawTraces(ctx, type, w, h) {
  var joints = Array.from(visJoints).sort(function(a, b) { return a - b; });
  if (!joints.length) return;

  var yMin = Infinity, yMax = -Infinity;
  for (var ji = 0; ji < joints.length; ji++) {
    var buf = buffers[type.key + ":" + joints[ji]];
    if (!buf) continue;
    for (var i = 0; i < buf.length; i++) {
      var v = buf[i];
      if (isFinite(v)) { if (v < yMin) yMin = v; if (v > yMax) yMax = v; }
    }
  }
  if (!isFinite(yMin)) { yMin = -1; yMax = 1; }
  if (yMax - yMin < 1e-9) { yMax = yMin + 1; }
  var margin = (yMax - yMin) * Y_MARGIN;
  yMin -= margin; yMax += margin;

  var xMap = function(i) { return (i / (MAX_POINTS - 1)) * w; };
  var yMap = function(v) { return h - ((v - yMin) / (yMax - yMin)) * h; };

  for (var ji = 0; ji < joints.length; ji++) {
    var j   = joints[ji];
    var buf = buffers[type.key + ":" + j];
    if (!buf || buf.length < 2) continue;
    ctx.strokeStyle = COLOURS[j % COLOURS.length];
    ctx.lineWidth   = 1.2;
    ctx.beginPath();
    var offset = MAX_POINTS - buf.length, first = true;
    for (var i = 0; i < buf.length; i++) {
      var v = buf[i];
      if (!isFinite(v)) { first = true; continue; }
      if (first) { ctx.moveTo(xMap(offset + i), yMap(v)); first = false; }
      else ctx.lineTo(xMap(offset + i), yMap(v));
    }
    ctx.stroke();
  }

  ctx.fillStyle = "#8b949e";
  ctx.font = "10px 'SF Mono', Consolas, monospace";
  ctx.textAlign = "right";
  for (var r = 0; r <= GRID_ROWS; r++) {
    var val = yMin + (yMax - yMin) * (1 - r / GRID_ROWS);
    ctx.fillText(val.toFixed(3), w - 6, (h / GRID_ROWS) * r - 3);
  }
}

/* ───────────────────────────────────────────────────────────────
   Labels & status
   ─────────────────────────────────────────────────────────────── */

function updateLabels() {
  var dtMs = (avgDt * 1000).toFixed(1);
  for (var i = 0; i < TYPES.length; i++) {
    var el = document.querySelector("#panel-" + TYPES[i].key + " .label");
    if (el) el.textContent = TYPES[i].label + " (" + TYPES[i].unit + ")  ·  Δt " + dtMs + " ms";
  }
  var ft = $("footerDt");
  if (ft) ft.textContent = "Δt " + dtMs + " ms";
  var rd = $("rateDisplay");
  if (rd) rd.textContent = (connected ? (imported ? "已导入 " : "") + frameCount + " 帧" : (imported ? "CSV 数据" : "离线"));
}

function setStatus(live) {
  var dot  = $("dot");
  var text = $("statusText");
  if (dot)  dot.className  = "dot " + (live ? "live" : "dead");
  if (text) text.textContent = live ? (imported ? "CSV" : "在线") : (imported ? "CSV" : "离线");
}

/* ───────────────────────────────────────────────────────────────
   Lifecycle
   ─────────────────────────────────────────────────────────────── */

window.addEventListener("DOMContentLoaded", function() {
  debug("DOM ready");
  init();
});
window.addEventListener("beforeunload", function() {
  if (es) es.close();
  if (rafId) cancelAnimationFrame(rafId);
});
