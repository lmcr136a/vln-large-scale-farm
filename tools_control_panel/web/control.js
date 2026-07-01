// ═══════════════════════════════════════════════════════════════
//  control.js
//  ★ All setInterval image polling removed
//  ★ SocketIO push: front_frame / back_frame / map_image / robot_pose
//  ★ Konva.js: map zoom/pan + canvas layering (map layer / dynamic layer)
// ═══════════════════════════════════════════════════════════════

const host   = window.location.hostname;
const socket = io(`http://${host}:8000`, {
  transports: ['polling'],
});

// ── State ─────────────────────────────────────────────────────
const keyState     = {};
let isAutoMode     = false;
let pathNodes      = [];
let currentMapMeta = null;
let isFirstMap     = true;
let robotPose      = null;

// ── Konva Stage / Layer Initialization ───────────────────────
let stage, mapLayer, dynLayer, mapImage;

const ZOOM_MIN  = 0.1;
const ZOOM_MAX  = 20;
const ZOOM_STEP = 1.15;

function onWheel(e) {
  e.preventDefault();
  const oldScale = stage.scaleX();
  const pointer  = stage.getPointerPosition();
  const mousePointTo = {
    x: (pointer.x - stage.x()) / oldScale,
    y: (pointer.y - stage.y()) / oldScale,
  };
  const newScale = e.deltaY < 0
    ? Math.min(oldScale * ZOOM_STEP, ZOOM_MAX)
    : Math.max(oldScale / ZOOM_STEP, ZOOM_MIN);
  stage.scale({ x: newScale, y: newScale });
  stage.position({
    x: pointer.x - mousePointTo.x * newScale,
    y: pointer.y - mousePointTo.y * newScale,
  });
  stage.batchDraw();
}

function initStage() {
  const container = document.getElementById('map-konva-container');
  if (!container) {
    console.error('[Konva] map-konva-container not found in DOM');
    return;
  }
  const stageW = container.clientWidth  || 800;
  const stageH = container.clientHeight || 600;

  stage = new Konva.Stage({
    container: 'map-konva-container',
    width:  stageW,
    height: stageH,
    draggable: true,
  });

  mapLayer = new Konva.Layer();
  dynLayer = new Konva.Layer();
  stage.add(mapLayer);
  stage.add(dynLayer);

  mapImage = new Konva.Image({ x: 0, y: 0, listening: false });
  mapLayer.add(mapImage);

  stage.container().addEventListener('wheel', onWheel, { passive: false });
  stage.on('click', onMapClick);

  window.addEventListener('resize', () => {
    stage.width(container.clientWidth);
    stage.height(container.clientHeight);
    stage.batchDraw();
  });
}

initStage();

// ── Util: World → Stage pixel ─────────────────────────────────
function worldToStagePixel(wx, wy) {
  if (!currentMapMeta) return null;
  const { resolution, origin_x, origin_y, height, rot_angle } = currentMapMeta;
  const c =  Math.cos(rot_angle), s = Math.sin(rot_angle);
  const rx =  c * wx + s * wy;
  const ry = -s * wx + c * wy;
  const px = (rx - origin_x) / resolution;
  const py = height - (ry - origin_y) / resolution;
  return { x: px, y: py };
}

// ── Util: Stage click → World ─────────────────────────────────
function stageClickToWorld(stageX, stageY) {
  if (!currentMapMeta) return null;
  const { resolution, origin_x, origin_y, height } = currentMapMeta;
  const wx = origin_x + stageX * resolution;
  const wy = origin_y + (height - stageY) * resolution;
  return { x: wx, y: wy };
}

function onMapClick(e) {
  // Disabled on control panel — use /path_plan.html
}

// ═══════════════════════════════════════════════════════════════
//  Socket Connection
// ═══════════════════════════════════════════════════════════════
socket.on('connect', () => {
  console.log('Socket.IO connected:', socket.id);
  loadMission();
});
socket.on('disconnect',    () => console.log('Socket.IO disconnected'));
socket.on('connect_error', (e) => console.error('Connection error:', e));

setInterval(() => socket.emit('heartbeat'), 500);

// ── Keyboard Input ────────────────────────────────────────────
const _keyTimers = {};  // repeat timers while key is held

window.addEventListener('blur', () => {
  for (const key in keyState) {
    if (keyState[key]) {
      keyState[key] = false;
      socket.emit('keyup', key);
      clearInterval(_keyTimers[key]);
      delete _keyTimers[key];
    }
  }
});
window.addEventListener('keydown', (e) => {
  if (document.activeElement.tagName === 'INPUT') return;
  if (isAutoMode) return;
  const key = e.key;
  if (keyState[key]) return;  // already held
  keyState[key] = true;
  socket.emit('keydown', key);
  // Repeat often (100ms) so a few dropped/late packets over a jittery link don't
  // let the commander's KB_DECAY watchdog expire and cut manual driving.
  _keyTimers[key] = setInterval(() => socket.emit('keydown', key), 100);
});
window.addEventListener('keyup', (e) => {
  if (document.activeElement.tagName === 'INPUT') return;
  if (isAutoMode) return;
  const key = e.key;
  if (!keyState[key]) return;
  keyState[key] = false;
  socket.emit('keyup', key);
  clearInterval(_keyTimers[key]);
  delete _keyTimers[key];
});

// ═══════════════════════════════════════════════════════════════
//  SocketIO Push Events
// ═══════════════════════════════════════════════════════════════

// ── Top image = back camera over radio (low-bandwidth) ───────

// 1×1 black JPEG used when no radio signal
const _BLACK_FRAME = 'data:image/jpeg;base64,/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8UHRofHh0aHBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/wAALCAABAAEBAREA/8QAFAABAAAAAAAAAAAAAAAAAAAACf/EABQQAQAAAAAAAAAAAAAAAAAAAAD/2gAIAQEAAD8AVIP/2Q==';

let _lastRadioFrameTime = 0;
const RADIO_FRAME_TIMEOUT = 9;  // seconds (= 3 × RADIO_INTERVAL)

function _applyBlackIfNoSignal() {
  const img = document.getElementById('rgb-image');
  if (!img) return;
  const elapsed = (Date.now() - _lastRadioFrameTime) / 1000;
  if (_lastRadioFrameTime === 0 || elapsed > RADIO_FRAME_TIMEOUT) {
    img.src = _BLACK_FRAME;
  }
}
setInterval(_applyBlackIfNoSignal, 1000);

// Bottom RGB stream bar: visibility follows internet availability
const RGB_CAM_IMG = {
  left:  'left-rgb-image',
  front: 'front-rgb-image',
  right: 'right-rgb-image',
  back:  'back-rgb-image',
};

function setRgbBarVisible(visible) {
  const bar = document.getElementById('rgb-stream-bar');
  if (bar) bar.style.display = visible ? 'grid' : 'none';
}

// Keep the left RGB column pinned just below #top-panel's *actual* rendered
// height (which varies with content), so it never overlaps the
// safety-checker / radio-camera widgets above.
function syncLeftColumnTop() {
  const topPanel = document.getElementById('top-panel');
  const bar       = document.getElementById('rgb-stream-bar');
  if (!topPanel || !bar) return;
  bar.style.top = `${topPanel.getBoundingClientRect().height}px`;
}

// Init: radio top image, bottom RGB bar hidden until internet is up
(function initCameras() {
  const frontTop = document.getElementById('rgb-image');
  if (frontTop) {
    frontTop.classList.add('radio-mode');
    frontTop.src = _BLACK_FRAME;
  }
  setRgbBarVisible(false);
  syncLeftColumnTop();
  const topPanel = document.getElementById('top-panel');
  if (topPanel && typeof ResizeObserver !== 'undefined') {
    new ResizeObserver(syncLeftColumnTop).observe(topPanel);
  }
  window.addEventListener('resize', syncLeftColumnTop);
})();

socket.on('radio_frame', (data) => {
  const img = document.getElementById('rgb-image');
  if (img) img.src = 'data:image/jpeg;base64,' + data.data;
  _lastRadioFrameTime = Date.now();
});

// ── RGB ↔ GPS frame matching (replay only) ───────────────────────
// Each RGB frame and each pose carries its sim-time `t`. We hold the displayed
// robot pose to the timestamp of the latest FRONT frame, so the map marker shows
// where the robot was when that scene was captured. Stale/out-of-order frames
// are dropped so nothing queues up. Live mode (no `t`) keeps the old behavior.
let poseBuffer = [];            // [{t, data}] recent timestamped poses
const _lastFrameT = {};         // camera -> last shown sim-time
const SYNC_CAM = 'front';       // camera whose timestamp drives the pose

function _poseAt(t) {
  let best = null, bestDt = Infinity;
  for (const p of poseBuffer) {
    const dt = Math.abs(p.t - t);
    if (dt < bestDt) { bestDt = dt; best = p; }
  }
  return best ? best.data : null;
}

function _bindRgbCam(camera, elemId) {
  socket.on(`${camera}_frame`, (data) => {
    // Drop stale / out-of-order frames so the panel never shows a backlog.
    if (data.t != null) {
      if (_lastFrameT[camera] != null && data.t <= _lastFrameT[camera]) return;
      _lastFrameT[camera] = data.t;
    }
    const img = document.getElementById(elemId);
    if (img) img.src = 'data:image/jpeg;base64,' + data.data;
    // Sync the map's robot pose to THIS frame's moment (front cam = clock).
    if (data.t != null && camera === SYNC_CAM) {
      const p = _poseAt(data.t);
      if (p) { robotPose = p; redrawDynLayer(); }
    }
  });
}
for (const [camera, elemId] of Object.entries(RGB_CAM_IMG)) {
  _bindRgbCam(camera, elemId);
}

socket.on('map_updated', (data) => {
  currentMapMeta = {
    resolution: data.resolution,
    origin_x:   data.origin_x,
    origin_y:   data.origin_y,
    width:      data.width,
    height:     data.height,
    rot_angle:  data.rot_angle ?? 0,
  };

  if (!mapImage) {
    console.warn('[map] Konva not ready yet, skipping render');
    return;
  }

  const applyImage = (img) => {
    mapImage.image(img);
    mapImage.width(data.width);
    mapImage.height(data.height);
    mapLayer.batchDraw();

    if (isFirstMap) {
      isFirstMap = false;
      const scale = Math.min(
        stage.width()  / data.width,
        stage.height() / data.height,
      ) * 0.9;
      stage.scale({ x: scale, y: scale });
      stage.position({
        x: (stage.width()  - data.width  * scale) / 2,
        y: (stage.height() - data.height * scale) / 2,
      });
      stage.batchDraw();
    }
    redrawDynLayer();
  };

  const img = new window.Image();
  img.onload = () => applyImage(img);
  if (data.image_data) {
    img.src = 'data:image/png;base64,' + data.image_data;
  } else {
    img.src = `/saved_map.png?v=${data.version}`;
  }
  console.log(`[map] ${data.width}\u00d7${data.height} rot=${(data.rot_angle*180/Math.PI).toFixed(1)}deg ${data.image_data ? '(live)' : '(saved)'}`);
});

socket.on('robot_pose', (data) => {
  const isFirst = robotPose === null;
  if (data.t != null) {
    // Replay: buffer the timestamped pose; the matching RGB frame drives the
    // displayed marker (see _bindRgbCam). Seed the marker once so it appears.
    poseBuffer.push({ t: data.t, data });
    if (poseBuffer.length > 400) poseBuffer.shift();
    if (!isFirst) return;
  }
  robotPose = data;
  if (isFirst && currentMapMeta && pathNodes.length === 0) {
    pathNodes.push({ worldX: data.x, worldY: data.y, reached: false });
  }
  redrawDynLayer();
});

socket.on('sysmon', (data) => {
  if (!data || typeof data !== 'object') return;
  const { cpu, used_gb, total_gb } = data;
  const cpuEl = document.getElementById('info-cpu');
  const ssdEl = document.getElementById('info-ssd');
  if (cpuEl) cpuEl.textContent = typeof cpu === 'number' ? `${cpu.toFixed(1)}%` : '—';
  if (ssdEl && typeof used_gb === 'number' && typeof total_gb === 'number') {
    const usedTB  = (used_gb  / 1024).toFixed(1);
    const totalTB = (total_gb / 1024).toFixed(1);
    ssdEl.textContent = `${usedTB} / ${totalTB} TB`;
  }
});

// Path finished — return the button to idle (no auto-restart; Run is manual).
socket.on('auto_mode_completed', () => { setAutoButton(false); });
// Sent by the server on (re)connect so a refreshed panel reflects the run that
// is still going on the robot, instead of reverting the button to ▶ RUN.
socket.on('auto_state', (data) => { setAutoButton(!!data?.running); });

socket.on('robot_status', (data) => {
  const div = document.getElementById('robot-status');
  if (data.status) { div.textContent = data.status; div.style.display = 'block'; }
  else             { div.style.display = 'none'; }
});

socket.on('robot_telemetry', (data) => {
  const set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v ?? '—'; };

  set('info-battery', data.battery >= 0 ? `${data.battery.toFixed(1)}%` : null);
  set('info-mode',    data.mode);

  // RGB streaming bar is only meaningful when internet is up
  setRgbBarVisible(!!data.internet);

  const estop = document.getElementById('estop-badge');
  if (estop) {
    estop.textContent = data.estop ? '🔴' : '🟢';
    estop.title = data.estop ? 'E-STOP active' : 'OK';
  }

  if (data.sensors) {
    const s = document.getElementById('info-sensors');
    if (s) s.textContent = Object.entries(data.sensors)
      .map(([k, v]) => `${k}:${v ? '✓' : '✗'}`).join('  ');
    updateCameraAlert(data.sensors);
  }

  // WiFi name + internet quality %
  const inetQ    = data.internet_quality != null ? ` (${data.internet_quality}%)` : '';
  const wifiName = (data.wifi && data.wifi !== '—') ? data.wifi : '—';
  set('info-wifi', wifiName + inetQ);

  // WiFi TX alive check — find the WiFi interface and show % utilization next to SSID
  if (data.net) {
    const wifiEl = document.getElementById('info-wifi');
    const netEl  = document.getElementById('info-net');
    // WiFi iface heuristic: starts with 'w' (wlan0, wlP1p1s0, wlp3s0 etc.)
    const wifiIface = Object.entries(data.net).find(([k]) => k.startsWith('w'));
    if (wifiIface) {
      const [, s] = wifiIface;
      const total = (s.tx_kbps || 0) + (s.rx_kbps || 0);
      // Show as % of a rough 10 MB/s WiFi ceiling, capped at 99
      const pct   = Math.min(99, Math.round(total / (10 * 1024) * 100));
      const color = s.alive ? '' : 'color:#ff5050';
      const tag   = s.alive ? '' : ' ⚠';
      if (netEl) netEl.innerHTML = `<span style="${color}">${pct}%${tag}</span>`;
    } else if (netEl) {
      netEl.textContent = '—';
    }
  }

  // Radio quality + one-way delay
  updateRadioScore('score-radio', data.radio_quality);
  const radioRttEl = document.getElementById('radio-delay');
  if (radioRttEl) {
    const ow = data.radio_rtt_ms != null ? Math.round(data.radio_rtt_ms / 2) : null;
    radioRttEl.textContent = ow != null ? `${ow} ms` : '—';
  }
  updateRadioScore('score-gnss',  data.gps_status?.gnss_radio_quality);

  // GPS panel
  if (data.gps_status) updateGpsPanel(data.gps_status);

  // Tmux panel — support both new process_status and legacy tmux_status
  const procStatus = data.process_status || data.tmux_status;
  if (procStatus) updateTmuxPanel(procStatus);

  // Safety checker
  if (data.safety_status) updateSafetyPanel(data.safety_status);

  // Base (Scout) CAN link / fault — surfaced via telemetry, which travels over
  // radio too, so this banner shows even when internet is down.
  updateBaseAlert(data.base_status);

  // System-ready badge above RUN.
  updateReadyBadge(data);
});

// Sticky "Ready" / "OK" badge. Shows white "Ready" when every system is healthy
// (all cameras, lidar/imu/gps, RTK Float-or-Fixed, Scout link) and "OK" while
// driving. Driven only by telemetry we actually receive: when internet drops no
// telemetry arrives, so the last state simply persists (never auto-cleared). It
// only disappears when a telemetry update reports a system is NOT ready.
function updateReadyBadge(data) {
  const el = document.getElementById('ready-badge');
  if (!el) return;
  if (data.system_ready === undefined || data.system_ready === null) return; // sticky
  if (data.system_ready) {
    el.textContent = (data.mode === 'auto') ? 'OK' : 'Ready';
    el.style.display = 'block';
  } else {
    el.style.display = 'none';
  }
}

// AgileX Scout error_code is a bitfield. Bit 0x04 = handheld RC transmitter not
// connected, which is the NORMAL state when driving from the panel / autonomously
// (the RC is off). It's not a base fault, so mask it out of the red banner —
// genuine faults (under-voltage 0x01/0x02, motor-driver comm 0x08/0x10/0x20/0x40,
// etc.) still raise it.
const BASE_BENIGN_ERROR_BITS = 0x04;   // RC transmitter disconnected
function updateBaseAlert(bs) {
  const el = document.getElementById('base-alert');
  if (!el) return;
  // No base_status yet, or this Jetson build doesn't monitor the base → hide.
  if (!bs || !bs.monitored) { el.style.display = 'none'; return; }
  let msg = '';
  const realFault = (bs.error_code || 0) & ~BASE_BENIGN_ERROR_BITS;
  if (!bs.comm) {
    msg = '⚠ BASE NOT RESPONDING — CAN link down (robot may not move)';
  } else if (realFault) {
    msg = '⚠ BASE FAULT — error code 0x' + Number(bs.error_code).toString(16).toUpperCase();
  }
  if (msg) { el.textContent = msg; el.style.display = 'block'; }
  else     { el.style.display = 'none'; }
}

// Red blinking banner when any expected camera stops streaming. A camera that
// fails to open (including Argus errors) never publishes frames, so its
// telemetry flag (sensors.zed_<name>) goes false — that's what we surface here.
const _CAM_ORDER = ['front', 'right', 'back', 'left'];
function updateCameraAlert(sensors) {
  const el = document.getElementById('camera-alert');
  if (!el) return;
  // Collect every zed_<name> the Jetson reports; expected cameras are seeded
  // there even before they stream, so a never-opened camera shows as down too.
  const down = Object.keys(sensors)
    .filter(k => k.startsWith('zed_') && !sensors[k])
    .map(k => k.slice(4));
  if (down.length === 0) { el.style.display = 'none'; return; }
  down.sort((a, b) => {
    const ia = _CAM_ORDER.indexOf(a), ib = _CAM_ORDER.indexOf(b);
    return (ia < 0 ? 99 : ia) - (ib < 0 ? 99 : ib);
  });
  const names = down.map(n => n.toUpperCase()).join(', ');
  el.textContent = `⚠ CAMERA DOWN — ${names} (no stream / Argus failure)`;
  el.style.display = 'block';
}

socket.on('waypoint_reached', (data) => {
  const idx = data.index;
  if (idx >= 0 && idx < pathNodes.length) {
    pathNodes[idx].reached = true;
    redrawDynLayer();
  }
  updateProgress();
});

// Waypoint completion shown above Recording, e.g. "3 / 30 (10%)".
function updateProgress() {
  const row = document.getElementById('run-progress');
  const val = document.getElementById('run-progress-val');
  if (!row || !val) return;
  const total = pathNodes.length;
  if (!total) { row.style.display = 'none'; return; }
  const done = pathNodes.filter(n => n.reached).length;
  const pct  = Math.round(done / total * 100);
  val.textContent = `${done} / ${total} (${pct}%)`;
  row.style.display = '';
}

// ═══════════════════════════════════════════════════════════════
//  Draw Dynamic Layer (robot + waypoints)
// ═══════════════════════════════════════════════════════════════
function redrawDynLayer() {
  if (!dynLayer) return;
  dynLayer.destroyChildren();

  const visibleNodes = pathNodes.map(n => {
    const p = worldToStagePixel(n.worldX, n.worldY);
    return p ? { ...n, stageX: p.x, stageY: p.y } : null;
  }).filter(Boolean);

  if (visibleNodes.length > 1) {
    const pts = [];
    visibleNodes.forEach(n => pts.push(n.stageX, n.stageY));
    dynLayer.add(new Konva.Line({
      points: pts,
      stroke: 'rgba(0,0,0,0.95)',
      strokeWidth: 3,
      dash: [8, 4],
      listening: false,
    }));
  }

  visibleNodes.forEach((n, i) => {
    const isStart = i === 0;
    const isEnd   = i === visibleNodes.length - 1;
    if (isStart || isEnd) {
      const S = 5;
      dynLayer.add(new Konva.Rect({
        x: n.stageX - S / 2, y: n.stageY - S / 2,
        width: S, height: S,
        fill: isStart ? 'white' : '#4499ff',
        stroke: 'black', strokeWidth: 1,
        listening: false,
      }));
    } else {
      dynLayer.add(new Konva.Circle({
        x: n.stageX, y: n.stageY,
        radius: n.reached ? 2.5 : 2,
        fill: n.reached ? 'rgb(255,220,0)' : 'rgb(200,200,200)',
        stroke: 'black', strokeWidth: 1,
        listening: false,
      }));
    }
    // Order number (1-based) — explicit drive sequence, not edit order.
    dynLayer.add(new Konva.Text({
      x: n.stageX + 4, y: n.stageY - 5, text: String(i + 1),
      fontSize: 9, fill: 'black', stroke: 'white', strokeWidth: 0.4, listening: false,
    }));
  });

  if (robotPose && currentMapMeta) {
    const p = worldToStagePixel(robotPose.x, robotPose.y);
    if (p) {
      const res      = currentMapMeta.resolution;   // m/px from map_state.json
      const r_px     = 0.3  / res;                  // ROBOT_HALF_M = 0.3m (60x60cm footprint)
      const arrow_px = 0.46 / res;                  // ROBOT_ARROW_LEN_M = 0.46m
      const lw       = Math.max(1, r_px * 0.25);

      const yawRad = robotPose.yaw - currentMapMeta.rot_angle;
      const dx =  Math.cos(yawRad) * arrow_px;
      const dy = -Math.sin(yawRad) * arrow_px;

      dynLayer.add(new Konva.Circle({
        x: p.x, y: p.y,
        radius: r_px,
        fill: 'rgba(255,100,0,0.95)',
        stroke: 'white',
        strokeWidth: lw,
        listening: false,
      }));
      dynLayer.add(new Konva.Arrow({
        points: [p.x, p.y, p.x + dx, p.y + dy],
        pointerLength: r_px * 0.5,
        pointerWidth:  r_px * 0.4,
        fill:   'rgba(255,100,0,0.95)',
        stroke: 'rgba(255,100,0,0.95)',
        strokeWidth: lw,
        listening: false,
      }));
    }
  }

  dynLayer.batchDraw();
}

// ═══════════════════════════════════════════════════════════════
//  Auto Mode
// ═══════════════════════════════════════════════════════════════
// Reflect run state on the big RUN/Stop button.
function setAutoButton(active) {
  isAutoMode = active;
  const btn = document.getElementById('btn-run-big');
  if (!btn) return;
  btn.innerHTML = active ? '⏹ STOP' : '▶ RUN';
  btn.classList.toggle('active-auto', active);
}

function _currentWaypoints() {
  return pathNodes.map(n => ({ x: n.worldX, y: n.worldY }));
}

// RUN button: first press starts from the beginning; pressing again stops
// (same as the old E-STOP — soft stop of path following).
function runNow() {
  if (isAutoMode) {
    setAutoButton(false);
    socket.emit('stop_autonomous');
    return;
  }
  pathNodes.forEach(n => n.reached = false);   // fresh run → progress back to 0%
  updateProgress();
  redrawDynLayer();
  startRunTimer();                              // 2h countdown, anchored to RUN
  setAutoButton(true);
  socket.emit('start_autonomous', { waypoints: _currentWaypoints() });
}

// ── 2-hour mission countdown ──────────────────────────────────────────────
// Anchored to the RUN press only. Resume never restarts it (it just keeps
// counting down from the original RUN), matching "처음 run 누른거 기준".
const RUN_TIMER_SECONDS = 2 * 60 * 60;
let _runTimerEnd      = 0;
let _runTimerInterval = null;

function startRunTimer() {
  _runTimerEnd = Date.now() + RUN_TIMER_SECONDS * 1000;
  if (_runTimerInterval) clearInterval(_runTimerInterval);
  _tickRunTimer();
  _runTimerInterval = setInterval(_tickRunTimer, 1000);
}

function _tickRunTimer() {
  const el = document.getElementById('run-timer');
  if (!el) return;
  let rem = Math.round((_runTimerEnd - Date.now()) / 1000);
  if (rem <= 0) {
    rem = 0;
    if (_runTimerInterval) { clearInterval(_runTimerInterval); _runTimerInterval = null; }
  }
  const hh = String(Math.floor(rem / 3600)).padStart(2, '0');
  const mm = String(Math.floor((rem % 3600) / 60)).padStart(2, '0');
  const ss = String(rem % 60).padStart(2, '0');
  el.textContent = `${hh}:${mm}:${ss}`;
  el.classList.toggle('run-timer-low', rem > 0 && rem < 300);
  el.style.display = '';
}

// Resume button: continue from a specific point number (the "from pt" input,
// 1-based, matching the marker labels). Left blank → nearest upcoming waypoint.
function resumeRun() {
  setAutoButton(true);
  const raw = document.getElementById('resume-index')?.value;
  const n   = raw ? parseInt(raw, 10) : NaN;
  const msg = { waypoints: _currentWaypoints() };
  if (Number.isInteger(n) && n >= 1) msg.start_index = n - 1;   // 1-based → 0-based
  socket.emit('resume_autonomous', msg);
}

function resetMapZoom() {
  if (!stage) return;
  stage.scale({ x: 1, y: 1 });
  stage.position({ x: 0, y: 0 });
  stage.batchDraw();
}

// ═══════════════════════════════════════════════════════════════
//  Recording
// ═══════════════════════════════════════════════════════════════
function setRecordingState(active, dirname) {
  const statusEl  = document.getElementById('recordingStatus');
  const inputEl   = document.getElementById('dirnameInput');
  const dirnameEl = document.getElementById('dirnameDisplay');
  statusEl.textContent = active ? 'Recording' : 'Stopped';
  statusEl.classList.toggle('active', active);
  inputEl.style.display   = active ? 'none' : '';
  dirnameEl.style.display = active ? '' : 'none';
  if (active) dirnameEl.textContent = dirname || inputEl.value || '—';
  document.getElementById('btn-rec-start').style.display = active ? 'none' : '';
  document.getElementById('btn-rec-stop').style.display  = active ? '' : 'none';
}

function startRecording() {
  const dirname = document.getElementById('dirnameInput').value.trim();
  if (!dirname) { alert('Please enter a directory name.'); return; }
  socket.emit('start_recording', dirname);
  setRecordingState(true, dirname);
}

function stopRecording() {
  socket.emit('stop_recording');
  setRecordingState(false);
}

socket.on('recording_status', (data) => { setRecordingState(data.active, data.dirname); });

function clearEstop() {
  socket.emit('command', { cmd: 'clear_estop' });
}

async function loadMission() {
  try {
    const r = await fetch('/mission');
    if (!r.ok) return;
    const data = await r.json();
    const start = data.start;
    const wps   = data.waypoints || [];
    pathNodes = (start ? [start, ...wps] : wps).map(w => ({
      worldX: w.x, worldY: w.y, reached: false,
    }));
    if (data.isLoop && pathNodes.length > 1)
      pathNodes.push({ ...pathNodes[0] });
    redrawDynLayer();
    updateProgress();
  } catch {}
}

// ═══════════════════════════════════════════════════════════════
//  Radio Quality Score (colored text)
// ═══════════════════════════════════════════════════════════════
function _scoreColor(score) {
  if (score == null) return 'rgba(255,255,255,0.3)';
  if (score >= 80) return '#00d26e';
  if (score >= 60) return '#7ecf40';
  if (score >= 40) return '#f0c040';
  if (score >= 20) return '#f08030';
  return '#ff5050';
}

// ── Safety checker SVG init ──────────────────────────────────
(function initSafetySVG() {
  const NS = 'http://www.w3.org/2000/svg';
  const g  = document.getElementById('sc-zones');
  if (!g) return;

  // Screen space: forward=up(-Y_svg), right=right(+X_svg)
  // θ_svg measured CCW from +X_svg; SVG Y is flipped so we negate sin
  // Zone wedge angles in degrees (CCW from right = +X_svg)
  const ZONES_DEF = [
    { id: 'sc-front',       t1:  67.5, t2: 112.5 },
    { id: 'sc-front-left',  t1: 112.5, t2: 157.5 },
    { id: 'sc-left',        t1: 157.5, t2: 202.5 },
    { id: 'sc-back-left',   t1: 202.5, t2: 247.5 },
    { id: 'sc-back',        t1: 247.5, t2: 292.5 },
    { id: 'sc-back-right',  t1: 292.5, t2: 337.5 },
    { id: 'sc-right',       t1: 337.5, t2: 382.5 },
    { id: 'sc-front-right', t1:  22.5, t2:  67.5 },
  ];

  // 3 bands: (r_inner, r_outer) normalised so outermost = 0.95
  // Robot body radius = 0.30, band edges at 0.40, 0.50, 0.60 in robot units
  // Scale: 0.60 → 0.95  ⟹  factor = 0.95/0.60
  const S = 0.95 / 0.60;
  const BANDS = [
    { name: 'green',  r0: 0.50 * S, r1: 0.60 * S },
    { name: 'yellow', r0: 0.40 * S, r1: 0.50 * S },
    { name: 'red',    r0: 0.30 * S, r1: 0.40 * S },
  ];

  function pt(r, deg) {
    const rad = deg * Math.PI / 180;
    return [r * Math.cos(rad), -r * Math.sin(rad)];  // SVG Y flipped
  }

  function wedgePath(r0, r1, t1, t2) {
    const [x0, y0] = pt(r0, t1), [x1, y1] = pt(r1, t1);
    const [x2, y2] = pt(r1, t2), [x3, y3] = pt(r0, t2);
    const large = (t2 - t1) > 180 ? 1 : 0;
    return `M${x0},${y0} L${x1},${y1} A${r1},${r1} 0 ${large},0 ${x2},${y2} L${x3},${y3} A${r0},${r0} 0 ${large},1 ${x0},${y0} Z`;
  }

  // Outermost band drawn first (under), innermost on top
  for (const band of BANDS) {
    for (const zone of ZONES_DEF) {
      const el = document.createElementNS(NS, 'path');
      el.setAttribute('d', wedgePath(band.r0, band.r1, zone.t1, zone.t2));
      el.setAttribute('id', `${zone.id}-${band.name}`);
      el.setAttribute('class', 'sc-wedge');
      el.setAttribute('stroke', 'rgba(0,0,0,0.3)');
      el.setAttribute('stroke-width', '0.01');
      g.appendChild(el);
    }
  }
})();

function updateSafetyPanel(s) {
  const BANDS = ['red', 'yellow', 'green'];
  const ZONES = ['front', 'front_right', 'right', 'back_right',
                 'back',  'back_left',   'left',  'front_left'];
  const PRIORITY = { null: 0, green: 1, yellow: 2, red: 3 };

  for (const zone of ZONES) {
    const color = s[zone] || null;
    for (const band of BANDS) {
      const el = document.getElementById(`sc-${zone.replace('_','-')}-${band}`);
      if (!el) continue;
      el.className.baseVal = 'sc-wedge';
      if (color && PRIORITY[color] >= PRIORITY[band]) {
        el.classList.add(`sc-${color}`);
      }
    }
  }
}

// ── Safety checker on/off toggle (center button of the circles) ──────────────
let _safetyEnabled = true;
function applySafetyToggleUI() {
  const body  = document.getElementById('sc-body');
  const label = document.getElementById('sc-toggle-label');
  const svg   = document.getElementById('sc-svg');
  if (body)  body.setAttribute('fill', _safetyEnabled ? '#2e7d32' : '#7a2222');
  if (label) label.textContent = _safetyEnabled ? 'ON' : 'OFF';
  if (svg)   svg.style.opacity = _safetyEnabled ? '1' : '0.45';
}
function toggleSafety() {
  _safetyEnabled = !_safetyEnabled;
  socket.emit('set_safety_enabled', { enabled: _safetyEnabled });
  applySafetyToggleUI();
}
applySafetyToggleUI();

// Flip the GPS heading North↔South (robot mounted opposite to travel).
function flipHeading() {
  socket.emit('flip_heading');
}

function updateRadioScore(id, score) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = score != null ? `${score}%` : '—';
  el.style.color = _scoreColor(score);
}

// ═══════════════════════════════════════════════════════════════
//  GPS Panel
// ═══════════════════════════════════════════════════════════════
function updateGpsPanel(g) {
  // Accurate Float (hAcc ≤ 10 cm) is good enough to drive on; inaccurate Float
  // (> 10 cm) wanders — flag it red. Must match gps.float_accuracy_limit_m.
  const FLOAT_ACC_LIMIT = 0.10;
  const floatAccurate = g.rtk_float && g.h_acc != null && g.h_acc <= FLOAT_ACC_LIMIT;
  const badge = document.getElementById('gps-mode-badge');
  if (badge) {
    if (g.rtk_fixed) {
      badge.textContent = g.rtk_mode || 'RTK Fixed';
      badge.className = 'gps-mode-badge gps-fixed';
    } else if (g.rtk_float) {
      badge.textContent = floatAccurate ? 'RTK Float ✓' : 'RTK Float ⚠';
      badge.className = 'gps-mode-badge ' + (floatAccurate ? 'gps-float' : 'gps-float-bad');
    } else if (g.rtk_mode === 'DGPS' || g.rtk_mode === '3D Fix') {
      badge.textContent = g.rtk_mode;
      badge.className = 'gps-mode-badge gps-dgps';
    } else {
      badge.textContent = g.rtk_mode || 'No Fix';
      badge.className = 'gps-mode-badge gps-nofix';
    }
  }

  const set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v ?? '—'; };

  set('gps-sv',   g.sv);
  set('gps-hdop', g.hdop != null ? g.hdop.toFixed(1) : null);

  // RTK fix-quality diagnostics — explains why the receiver is Float vs Fixed.
  // hAcc: Fixed ~0.01-0.03m, Float ~0.2-1m+ | carr: None/Float/Fixed
  // cr (carrier-range used) & L2 (dual-band) drive ambiguity fixing.
  const hAcc = g.h_acc;
  const haccEl = document.getElementById('gps-hacc');
  if (haccEl) {
    haccEl.textContent = hAcc != null ? `${hAcc.toFixed(2)}m` : '—';
    haccEl.style.color = hAcc == null ? '' : (hAcc <= 0.05 ? '#7fffb0'
                                            : hAcc <= 0.30 ? '#ffd27f' : '#ff8f8f');
  }
  set('gps-strong', g.strong_sv);
  const carrEl = document.getElementById('gps-carr');
  if (carrEl) {
    carrEl.textContent = g.carr_soln || '—';
    carrEl.style.color = g.carr_soln === 'Fixed' ? '#7fffb0'
                       : g.carr_soln === 'Float' ? '#ffd27f' : '';
  }
  set('gps-cruse', g.cr_used);
  const l2El = document.getElementById('gps-l2cr');
  if (l2El) {
    l2El.textContent = g.l2_cr ?? '—';
    // 0 L2 signals → fixing is slow/unlikely; flag it.
    l2El.style.color = (g.l2_cr === 0) ? '#ff8f8f' : '';
  }
  set('gps-baseline', g.baseline_m != null ? `${g.baseline_m}m` : null);
  set('gps-rtcm', g.rtcm_count != null
        ? `${g.rtcm_count}${g.rtcm_bad ? ` (bad ${g.rtcm_bad})` : ''}` : null);
  const gq = g.gnss_radio_quality;
  const gqEl = document.getElementById('gps-gnssq');
  if (gqEl) {
    gqEl.textContent = gq != null ? `${gq}%` : '—';
    gqEl.style.color = gq == null ? '' : (gq >= 60 ? '#7fffb0'
                                        : gq >= 30 ? '#ffd27f' : '#ff8f8f');
  }
  set('gps-lat',  g.lat  != null ? g.lat.toFixed(6)  : null);
  set('gps-lon',  g.lon  != null ? g.lon.toFixed(6)  : null);
  set('gps-alt',  g.alt  != null ? `${g.alt.toFixed(1)}m` : null);

  const baseLine = document.getElementById('gps-base-line');
  if (baseLine) {
    if (g.base_lat != null) {
      baseLine.innerHTML =
        `Base Fixed<br>` +
        `<span class="lbl">Lat </span><span class="gps-coord">${g.base_lat.toFixed(6)}</span> ` +
        `<span class="lbl">Lon </span><span class="gps-coord">${g.base_lon.toFixed(6)}</span> ` +
        `<span class="lbl">Alt </span><span class="gps-coord">${g.base_alt != null ? g.base_alt.toFixed(1) + 'm' : '—'}</span>`;
    } else {
      // "Base not fixed" at inherited 10pt (no small class)
      baseLine.textContent = 'Base not fixed';
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  Tmux Panel
// ═══════════════════════════════════════════════════════════════
const TMUX_WINDOW_LABELS = {
  jetson_agent:  'Main',
  lidar:         'LiDAR',
  imu:           'IMU',
  gps:           'GPS Node',
};

function updateTmuxPanel(status) {
  const container = document.getElementById('tmux-rows');
  if (!container) return;
  const keys = Object.keys(TMUX_WINDOW_LABELS).filter(k => k in status);
  container.innerHTML = keys.map(key => {
    const alive = status[key];
    const dot = alive === true ? 'tdot-ok' : alive === false ? 'tdot-fail' : 'tdot-unkn';
    return `<div class="tmux-row">
      <div class="tmux-dot ${dot}"></div>
      <div class="tmux-lbl">${TMUX_WINDOW_LABELS[key]}</div>
      <button class="tmux-btn" onclick="restartWindow('${key}')">↺</button>
    </div>`;
  }).join('');
}

function restartWindow(key) {
  if (!confirm(`Restart "${TMUX_WINDOW_LABELS[key] || key}"?`)) return;
  socket.emit('command', { cmd: 'restart_window', window: key });
}

// Network recovery — sent as a generic command, which the lab server relays over
// the radio bridge too, so it reaches the Jetson even when wifi/tailscale are down.
function reconnectNetwork() {
  if (!confirm('Tell the Jetson to reconnect its network (wifi + tailscale)?')) return;
  socket.emit('command', { cmd: 'network_reconnect' });
}

function sendJetsonCmd() {
  const el = document.getElementById('jetson-cmd-input');
  if (!el) return;
  const command = el.value.trim();
  if (!command) return;
  if (!confirm(`Run on Jetson:\n${command}`)) return;
  socket.emit('command', { cmd: 'shell', command });
  el.value = '';
}