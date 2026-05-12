// ═══════════════════════════════════════════════════════════════
//  control.js  -  Optimized version
//  ★ All setInterval image polling removed
//  ★ SocketIO push: front_frame / back_frame / map_image / robot_pose
//  ★ Konva.js: map zoom/pan + canvas layering (map layer / dynamic layer)
// ═══════════════════════════════════════════════════════════════

const host   = window.location.hostname;
const socket = io(`http://${host}:8000`, {
  transports: ['polling'],
});

// ── State ─────────────────────────────────────────────────────
const keyState    = {};
let isAutoMode    = false;
let pathNodes     = [];
let currentMapMeta = null;   // { resolution, origin_x, origin_y, width, height }
let isFirstMap     = true;   // center stage on first map load
let robotPose      = null;   // { x, y, yaw }

// ── Konva Stage / Layer Initialization ───────────────────────
// Declared here, initialized in DOMContentLoaded to ensure container has real dimensions
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
  console.log(`[Konva] Stage init: ${stageW}x${stageH}`);

  stage = new Konva.Stage({
    container: 'map-konva-container',
    width:  stageW,
    height: stageH,
    draggable: true,
  });

  // Bottom Layer: map image (redrawn only when updated)
  mapLayer = new Konva.Layer();
  // Top Layer: robot icon + waypoints (redrawn on every update)
  dynLayer = new Konva.Layer();
  stage.add(mapLayer);
  stage.add(dynLayer);

  mapImage = new Konva.Image({ x: 0, y: 0, listening: false });
  mapLayer.add(mapImage);

  // ── Mouse Wheel Zoom ────────────────────────────────────────
  stage.container().addEventListener('wheel', onWheel, { passive: false });

  // ── Map click → waypoint ────────────────────────────────────
  stage.on('click', onMapClick);

  // ── Resize handler ──────────────────────────────────────────
  window.addEventListener('resize', () => {
    stage.width(container.clientWidth);
    stage.height(container.clientHeight);
    stage.batchDraw();
  });
}

// Script is loaded at bottom of <body>, so DOM is already ready - call directly
initStage();

// ── Util: World coordinates → Stage pixel coordinates ────────
// Image saved with np.flipud → row 0 = world Y_max (canvas top).
// So world +Y → canvas up (decreasing py).
//   px =         (wx - origin_x) / resolution
//   py = height - (wy - origin_y) / resolution   ← Y flip
function worldToStagePixel(wx, wy) {
  if (!currentMapMeta) return null;
  const { resolution, origin_x, origin_y, height, rot_angle } = currentMapMeta;
  // Apply same PCA rotation as save_map_glim._rotate_xy
  const c =  Math.cos(rot_angle), s = Math.sin(rot_angle);
  const rx =  c * wx + s * wy;
  const ry = -s * wx + c * wy;
  const px = (rx - origin_x) / resolution;
  const py = height - (ry - origin_y) / resolution;
  return { x: px, y: py };
}

// ── Util: Stage click coordinates → World coordinates ────────
function stageClickToWorld(stageX, stageY) {
  if (!currentMapMeta) return null;
  const { resolution, origin_x, origin_y, height } = currentMapMeta;
  const wx = origin_x + stageX * resolution;
  const wy = origin_y + (height - stageY) * resolution;  // Y flip
  return { x: wx, y: wy };
}

// ── Map Click — disabled for path editing (use /path_plan.html) ──
function onMapClick(e) {
  // Map click is intentionally disabled on the control panel.
  // Use the Path Planner page to set start point and waypoints.
}

// ═══════════════════════════════════════════════════════════════
//  Socket Connection
// ═══════════════════════════════════════════════════════════════
socket.on('connect', () => {
  console.log('Socket.IO connected:', socket.id);
  loadMission();
  if (typeof initSchedule === 'function') initSchedule();
});
socket.on('disconnect', () => console.log('Socket.IO disconnected'));
socket.on('connect_error', (e) => console.error('Connection error:', e));

setInterval(() => socket.emit('heartbeat'), 500);

// ── Keyboard Input (blocked during autonomous mode) ───────────
window.addEventListener('blur', () => {
  for (const key in keyState) {
    if (keyState[key]) { keyState[key] = false; socket.emit('keyup', key); }
  }
});
window.addEventListener('keydown', (e) => {
  if (document.activeElement.tagName === 'INPUT') return;
  if (isAutoMode) return;
  const key = e.key;
  if (!keyState[key]) { keyState[key] = true; socket.emit('keydown', key); }
});
window.addEventListener('keyup', (e) => {
  if (document.activeElement.tagName === 'INPUT') return;
  if (isAutoMode) return;
  const key = e.key;
  if (keyState[key]) { keyState[key] = false; socket.emit('keyup', key); }
});

// ═══════════════════════════════════════════════════════════════
//  ★ SocketIO Push Events (fully replaces HTTP polling)
// ═══════════════════════════════════════════════════════════════

// ── RGB Camera Frames ─────────────────────────────────────────
socket.on('front_frame', (data) => {
  document.getElementById('rgb-image').src = 'data:image/jpeg;base64,' + data.data;
});
socket.on('back_frame', (data) => {
  document.getElementById('back-rgb-image').src = 'data:image/jpeg;base64,' + data.data;
});

// ── Map Image (server pushes only when changed) ──────────────
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
    // Jetson online: image embedded directly in the socket event
    img.src = 'data:image/png;base64,' + data.image_data;
  } else {
    // Jetson offline: fetch cached saved_map.png from lab PC
    img.src = `/saved_map.png?v=${data.version}`;
  }

  console.log(`[map] ${data.width}\u00d7${data.height} rot=${(data.rot_angle*180/Math.PI).toFixed(1)}deg ${data.image_data ? '(live)' : '(saved)'}`);
});

// ── Robot Pose (10Hz) ─────────────────────────────────────────
socket.on('robot_pose', (data) => {
  const yawDeg = data.yaw != null ? (data.yaw * 180 / Math.PI).toFixed(1) : 'N/A';
  const rotDeg = currentMapMeta ? (currentMapMeta.rot_angle * 180 / Math.PI).toFixed(1) : 'N/A';
  console.log(`[robot_pose] x=${data.x?.toFixed(2)} y=${data.y?.toFixed(2)} yaw=${yawDeg}deg  rot_angle=${rotDeg}deg`);

  const isFirst = robotPose === null;
  robotPose = data;

  if (isFirst && currentMapMeta && pathNodes.length === 0) {
    pathNodes.push({ worldX: data.x, worldY: data.y, reached: false });
  }

  redrawDynLayer();
});

// ── sysmon ────────────────────────────────────────────────────
const sysmonDiv = document.getElementById('sysmon'); // kept for fallback
socket.on('sysmon', (data) => {
  if (!data || typeof data !== 'object') return;
  const { cpu, mem, used_gb, total_gb, used_pct } = data;

  const cpuEl  = document.getElementById('info-cpu');
  const ssdEl  = document.getElementById('info-ssd');

  if (cpuEl)  cpuEl.textContent  = typeof cpu  === 'number' ? `${cpu.toFixed(1)}%` : '—';
  if (ssdEl && typeof used_gb === 'number' && typeof total_gb === 'number') {
    const usedTB  = (used_gb  / 1024).toFixed(1);
    const totalTB = (total_gb / 1024).toFixed(1);
    ssdEl.textContent = `${usedTB} / ${totalTB} TB`;
  }
});

socket.on('auto_mode_completed', () => { if (isAutoMode) runNow(); });

socket.on('robot_status', (data) => {
  const div = document.getElementById('robot-status');
  if (data.status) { div.textContent = data.status; div.style.display = 'block'; }
  else             { div.style.display = 'none'; }
});

socket.on('robot_telemetry', (data) => {
  const battEl    = document.getElementById('info-battery');
  const modeEl    = document.getElementById('info-mode');
  const estopEl   = document.getElementById('estop-indicator');
  const sensorsEl = document.getElementById('info-sensors');
  const wifiEl    = document.getElementById('info-wifi');

  if (battEl)  battEl.textContent  = data.battery >= 0 ? `${data.battery.toFixed(1)}%` : '—';
  if (modeEl)  modeEl.textContent  = data.mode || '—';
  if (estopEl) {
    estopEl.textContent = data.estop ? '🔴 E-STOP' : '🟢 OK';
    estopEl.style.color = data.estop ? '#ff4444' : '#44ff88';
  }
  if (sensorsEl && data.sensors) {
    sensorsEl.textContent = Object.entries(data.sensors)
      .map(([k, v]) => `${k}:${v ? '✓' : '✗'}`).join('  ');
  }
  if (wifiEl) {
    const inet = data.internet === true ? '🌐✓' : data.internet === false ? '🌐✗' : '';
    wifiEl.textContent = (data.wifi && data.wifi !== '—' ? data.wifi : '—') + (inet ? '  ' + inet : '');
  }
});

socket.on('waypoint_reached', (data) => {
  const idx = data.index;
  if (idx >= 0 && idx < pathNodes.length) {
    pathNodes[idx].reached = true;
    redrawDynLayer();
  }
});

// ═══════════════════════════════════════════════════════════════
//  Draw Dynamic Layer (robot + waypoints)
// ═══════════════════════════════════════════════════════════════
function redrawDynLayer() {
  if (!dynLayer) return;  // Konva not yet initialized - skip
  dynLayer.destroyChildren();

  // Always recompute pixel positions from world coords so map resizes stay correct
  const visibleNodes = pathNodes.map(n => {
    const p = worldToStagePixel(n.worldX, n.worldY);
    return p ? { ...n, stageX: p.x, stageY: p.y } : null;
  }).filter(Boolean);

  // ── Waypoint lines ─────────────────────────────────────────
  if (visibleNodes.length > 1) {
    const pts = [];
    visibleNodes.forEach(n => pts.push(n.stageX, n.stageY));
    dynLayer.add(new Konva.Line({
      points: pts,
      stroke: 'rgba(200,200,200,0.5)',
      strokeWidth: 1,
      dash: [6, 3],
      listening: false,
    }));
  }

  // ── Waypoint markers ───────────────────────────────────────
  visibleNodes.forEach((n, i) => {
    const isStart = i === 0;
    const isEnd   = i === visibleNodes.length - 1;

    if (isStart || isEnd) {
      // Square: start=white, end=blue
      const S = 10;
      dynLayer.add(new Konva.Rect({
        x: n.stageX - S / 2, y: n.stageY - S / 2,
        width: S, height: S,
        fill: isStart ? 'white' : '#4499ff',
        stroke: 'black', strokeWidth: 1,
        listening: false,
      }));
    } else {
      // Intermediate destination: small gray circle
      dynLayer.add(new Konva.Circle({
        x: n.stageX, y: n.stageY,
        radius: n.reached ? 5 : 4,
        fill: n.reached ? 'rgb(255,220,0)' : 'rgb(200,200,200)',
        stroke: 'black', strokeWidth: 1,
        listening: false,
      }));
    }
  });

  // ── Robot icon: dot + heading line ────────────────────────
  if (robotPose && currentMapMeta) {
    const p = worldToStagePixel(robotPose.x, robotPose.y);
    if (p) {
      // yaw: subtract map rotation to get heading in rotated (image) frame
      const yawRad = robotPose.yaw - currentMapMeta.rot_angle;
      const headLen = 6;
      const dx =  Math.cos(yawRad) * headLen;
      const dy = -Math.sin(yawRad) * headLen;

      // Center dot
      dynLayer.add(new Konva.Circle({
        x: p.x, y: p.y,
        radius: 3,
        fill: 'rgba(255,100,0,0.95)',
        stroke: 'white',
        strokeWidth: 1,
        listening: false,
      }));

      // Heading arrow
      dynLayer.add(new Konva.Arrow({
        points: [p.x, p.y, p.x + dx, p.y + dy],
        pointerLength: 4,
        pointerWidth: 3,
        fill: 'rgba(255,100,0,0.95)',
        stroke: 'rgba(255,100,0,0.95)',
        strokeWidth: 1.5,
        listening: false,
      }));
    }
  }

  dynLayer.batchDraw();
}

// ═══════════════════════════════════════════════════════════════
//  Auto Mode Toggle
// ═══════════════════════════════════════════════════════════════
function runNow() {
  if (isAutoMode) {
    isAutoMode = false;
    document.getElementById('autoButton').textContent = '▶ Run Now';
    document.getElementById('autoButton').classList.remove('active-auto');
    socket.emit('stop_autonomous');
    return;
  }
  isAutoMode = true;
  document.getElementById('autoButton').textContent = '⏹ Stop';
  document.getElementById('autoButton').classList.add('active-auto');
  socket.emit('start_autonomous', {
    waypoints: pathNodes.map(n => ({ x: n.worldX, y: n.worldY })),
  });
}

function resetMapZoom() {
  if (!stage) return;
  stage.scale({ x: 1, y: 1 });
  stage.position({ x: 0, y: 0 });
  stage.batchDraw();
}

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

function emergencyStop() {
  socket.emit('estop');
  if (isAutoMode) {
    isAutoMode = false;
    const btn = document.getElementById('autoButton');
    if (btn) { btn.textContent = '▶ Run Now'; btn.classList.remove('active-auto'); }
  }
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
  } catch {}
}

function updateAutoButton() {
  const btn = document.getElementById('autoButton');
  if (!btn) return;
  btn.textContent = isAutoMode ? '⏹ Stop' : '▶ Run Now';
  btn.classList.toggle('active-auto', isAutoMode);
}