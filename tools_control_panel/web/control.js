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
  if (typeof initSchedule === 'function') initSchedule();
});
socket.on('disconnect',    () => console.log('Socket.IO disconnected'));
socket.on('connect_error', (e) => console.error('Connection error:', e));

setInterval(() => socket.emit('heartbeat'), 500);

// ── Keyboard Input ────────────────────────────────────────────
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
//  SocketIO Push Events
// ═══════════════════════════════════════════════════════════════

socket.on('front_frame', (data) => {
  document.getElementById('rgb-image').src = 'data:image/jpeg;base64,' + data.data;
});
socket.on('back_frame', (data) => {
  document.getElementById('back-rgb-image').src = 'data:image/jpeg;base64,' + data.data;
});

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

socket.on('auto_mode_completed', () => { if (isAutoMode) runNow(); });

socket.on('robot_status', (data) => {
  const div = document.getElementById('robot-status');
  if (data.status) { div.textContent = data.status; div.style.display = 'block'; }
  else             { div.style.display = 'none'; }
});

socket.on('robot_telemetry', (data) => {
  const set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v ?? '—'; };

  set('info-battery', data.battery >= 0 ? `${data.battery.toFixed(1)}%` : null);
  set('info-mode',    data.mode);

  const estop = document.getElementById('estop-badge');
  if (estop) {
    estop.textContent = data.estop ? '🔴' : '🟢';
    estop.title = data.estop ? 'E-STOP active' : 'OK';
  }

  if (data.sensors) {
    const s = document.getElementById('info-sensors');
    if (s) s.textContent = Object.entries(data.sensors)
      .map(([k, v]) => `${k}:${v ? '✓' : '✗'}`).join('  ');
  }

  // WiFi name + internet quality %
  const inetQ    = data.internet_quality != null ? ` (${data.internet_quality}%)` : '';
  const wifiName = (data.wifi && data.wifi !== '—') ? data.wifi : '—';
  set('info-wifi', wifiName + inetQ);

  // Radio quality scores
  updateRadioScore('score-radio', data.radio_quality);
  updateRadioScore('score-gnss',  data.gps_status?.gnss_radio_quality);

  // GPS panel
  if (data.gps_status) updateGpsPanel(data.gps_status);

  // Tmux panel
  if (data.tmux_status) updateTmuxPanel(data.tmux_status);
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
      stroke: 'rgba(200,200,200,0.5)',
      strokeWidth: 1,
      dash: [6, 3],
      listening: false,
    }));
  }

  visibleNodes.forEach((n, i) => {
    const isStart = i === 0;
    const isEnd   = i === visibleNodes.length - 1;
    if (isStart || isEnd) {
      const S = 10;
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
        radius: n.reached ? 5 : 4,
        fill: n.reached ? 'rgb(255,220,0)' : 'rgb(200,200,200)',
        stroke: 'black', strokeWidth: 1,
        listening: false,
      }));
    }
  });

  if (robotPose && currentMapMeta) {
    const p = worldToStagePixel(robotPose.x, robotPose.y);
    if (p) {
      const yawRad = robotPose.yaw - currentMapMeta.rot_angle;
      const headLen = 6;
      const dx =  Math.cos(yawRad) * headLen;
      const dy = -Math.sin(yawRad) * headLen;

      dynLayer.add(new Konva.Circle({
        x: p.x, y: p.y,
        radius: 3,
        fill: 'rgba(255,100,0,0.95)',
        stroke: 'white', strokeWidth: 1,
        listening: false,
      }));
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
//  Auto Mode
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
  const badge = document.getElementById('gps-mode-badge');
  if (badge) {
    badge.textContent = g.rtk_mode || 'No Fix';
    badge.className = 'gps-mode-badge ' + (
      g.rtk_fixed                                         ? 'gps-fixed' :
      g.rtk_float                                         ? 'gps-float' :
      (g.rtk_mode === 'DGPS' || g.rtk_mode === '3D Fix') ? 'gps-dgps'  :
                                                            'gps-nofix'
    );
  }

  const set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v ?? '—'; };

  set('gps-sv',   g.sv);
  set('gps-hdop', g.hdop != null ? g.hdop.toFixed(1) : null);
  set('gps-lat',  g.lat  != null ? g.lat.toFixed(6)  : null);
  set('gps-lon',  g.lon  != null ? g.lon.toFixed(6)  : null);
  set('gps-alt',  g.alt  != null ? `${g.alt.toFixed(1)}m` : null);

  const baseLine = document.getElementById('gps-base-line');
  if (baseLine) {
    if (g.base_lat != null) {
      // "Base Fixed" label at inherited 10pt; coordinate values small via .gps-coord
      baseLine.innerHTML =
        `Base Fixed &nbsp;` +
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
  slam:      'SLAM',
  map_saver: '2D Map',
  obstacle:  'Safety',
  gps:       'GPS Node',
  lidar:     'LiDAR',
  imu:       'IMU',
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