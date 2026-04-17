// ═══════════════════════════════════════════════════════════════
//  control.js  -  Optimized version
//  ★ All setInterval image polling removed
//  ★ SocketIO push: front_frame / back_frame / map_image / robot_pose
//  ★ Konva.js: map zoom/pan + canvas layering (map layer / dynamic layer)
// ═══════════════════════════════════════════════════════════════

const host   = window.location.hostname;
const socket = io(`http://${host}:8000`, {
  transports: ['polling'],   // WebSocket disabled - Tailscale mangles WS headers
});

// ── State ─────────────────────────────────────────────────────
const keyState    = {};
let isPathMode    = true;   // always in path mode
let isAutoMode    = false;
let pathNodes     = [];
let followRobot   = false;
let currentMapMeta = null;   // { resolution, origin_x, origin_y, width, height }
let isFirstMap     = true;   // center stage on first map load
let robotPose      = null;   // { x, y, yaw }
let frontSemantics = null;

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
  const { resolution, origin_x, origin_y, height } = currentMapMeta;
  const px = (wx - origin_x) / resolution;
  const py = height - (wy - origin_y) / resolution;
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

// ── Map Click → Add Waypoint ──────────────────────────────────
function onMapClick(e) {
  if (!currentMapMeta) return;
  if (isAutoMode)      return;
  if (stage.isDragging()) return;

  const pos    = stage.getRelativePointerPosition();
  const stageX = pos.x;
  const stageY = pos.y;

  const world = stageClickToWorld(stageX, stageY);
  if (!world) return;

  socket.emit('map_clicked', {
    img_x:   Math.round(stageX),
    img_y:   Math.round(stageY),
    world_x: world.x,
    world_y: world.y,
  });

  if (isPathMode) {
    pathNodes.push({
      stageX, stageY,
      worldX:  world.x,
      worldY:  world.y,
      reached: false,
    });
    redrawDynLayer();
  }
}

// ═══════════════════════════════════════════════════════════════
//  Socket Connection
// ═══════════════════════════════════════════════════════════════
socket.on('connect',    () => console.log('Socket.IO connected:', socket.id));
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
  const src = 'data:image/jpeg;base64,' + data.data;
  const frontEl = document.getElementById('rgb-image');
  if (frontEl) frontEl.src = src;

  const semanticEl = document.getElementById('semantic-front-image');
  if (semanticEl) {
    if (!semanticEl.dataset.overlayBound) {
      semanticEl.addEventListener('load', drawFrontSemanticOverlay);
      window.addEventListener('resize', drawFrontSemanticOverlay);
      semanticEl.dataset.overlayBound = 'true';
    }
    semanticEl.src = src;
    requestAnimationFrame(drawFrontSemanticOverlay);
  }
});
socket.on('back_frame', (data) => {
  document.getElementById('back-rgb-image').src = 'data:image/jpeg;base64,' + data.data;
});

socket.on('front_semantics', (data) => {
  frontSemantics = data;
  drawFrontSemanticOverlay();
});

// ── Map Image (server pushes only when changed) ──────────────
socket.on('map_updated', (data) => {
  // Store metadata from socket notification
  currentMapMeta = {
    resolution: data.resolution,
    origin_x:   data.origin_x,
    origin_y:   data.origin_y,
    width:      data.width,
    height:     data.height,
  };

  if (!mapImage) {
    console.warn('[map] Konva not ready yet, skipping render');
    return;
  }

  // Fetch image via HTTP GET (avoids sending large blobs through socket.io polling)
  const img = new window.Image();
  img.onload = () => {
    mapImage.image(img);
    mapImage.width(data.width);
    mapImage.height(data.height);
    mapLayer.batchDraw();

    // 첫 로드 시 맵을 화면 중앙에 배치
    if (isFirstMap) {
      isFirstMap = false;
      const scale = Math.min(
        stage.width()  / data.width,
        stage.height() / data.height,
      ) * 0.9;  // 90% so there's a small margin
      stage.scale({ x: scale, y: scale });
      stage.position({
        x: (stage.width()  - data.width  * scale) / 2,
        y: (stage.height() - data.height * scale) / 2,
      });
      stage.batchDraw();
    }

    redrawDynLayer();
  };
  // Cache-bust with version token so browser doesn't serve stale PNG
  img.src = `/map_latest.png?v=${data.version}`;

  console.log(`[map] received ${data.width}\u00d7${data.height}`);
});

// ── Robot Pose (10Hz) ─────────────────────────────────────────
socket.on('robot_pose', (data) => {
  const isFirst = robotPose === null;
  robotPose = data;   // { x, y, yaw }

  // On first pose received, auto-add robot position as first waypoint
  if (isFirst && currentMapMeta && pathNodes.length === 0) {
    pathNodes.push({ worldX: data.x, worldY: data.y, reached: false });
  }

  // Follow robot mode: move stage so robot stays centered
  if (followRobot && currentMapMeta) {
    const p = worldToStagePixel(data.x, data.y);
    if (p) {
      const sc = stage.scaleX();
      stage.position({
        x: stage.width() / 2 - p.x * sc,
        y: stage.height() / 2 - p.y * sc,
      });
    }
  }

  redrawDynLayer();
});

// ── sysmon ────────────────────────────────────────────────────
const sysmonDiv = document.getElementById('sysmon'); // kept for fallback
socket.on('sysmon', (data) => {
  if (!data || typeof data !== 'object') return;
  const { cpu, mem, used_gb, total_gb, used_pct, linear_mps, wifi } = data;

  const wifiEl = document.getElementById('info-wifi');
  const cpuEl  = document.getElementById('info-cpu');
  const ssdEl  = document.getElementById('info-ssd');

  if (wifiEl) wifiEl.textContent = wifi || '—';
  if (cpuEl)  cpuEl.textContent  = typeof cpu  === 'number' ? `${cpu.toFixed(1)}%` : '—';
  if (ssdEl && typeof used_gb === 'number' && typeof total_gb === 'number') {
    const usedTB  = (used_gb  / 1024).toFixed(1);
    const totalTB = (total_gb / 1024).toFixed(1);
    ssdEl.textContent = `${usedTB} / ${totalTB} TB`;
  }
});

socket.on('auto_mode_completed', () => { if (isAutoMode) toggleAutoMode(); });

socket.on('robot_status', (data) => {
  const div = document.getElementById('robot-status');
  if (data.status) { div.textContent = data.status; div.style.display = 'block'; }
  else             { div.style.display = 'none'; }
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
    if (visibleNodes.length > 2) pts.push(visibleNodes[0].stageX, visibleNodes[0].stageY);

    dynLayer.add(new Konva.Line({
      points: pts,
      stroke: 'rgb(0,200,200)',
      strokeWidth: 2,
      listening: false,
    }));
  }

  // ── Waypoint dots ──────────────────────────────────────────
  visibleNodes.forEach((n, i) => {
    let fill = 'rgb(0,100,150)';
    if (n.reached)    fill = 'rgb(255,255,0)';
    else if (i === 0) fill = 'rgb(255,0,255)';

    const circle = new Konva.Circle({
      x: n.stageX, y: n.stageY,
      radius: 2,          // same as line strokeWidth
      fill,
      stroke: null,
    });

    // Click on waypoint → remove it (except first/starting point)
    circle.on('click', (e) => {
      e.cancelBubble = true;
      if (i === 0) return;
      pathNodes.splice(i, 1);
      redrawDynLayer();
    });
    // Slightly larger hit area for easier clicking
    circle.hitFunc((ctx) => {
      ctx.beginPath();
      ctx.arc(0, 0, 8, 0, Math.PI * 2, true);
      ctx.closePath();
    });
    circle.on('mouseenter', () => { stage.container().style.cursor = 'pointer'; });
    circle.on('mouseleave', () => { stage.container().style.cursor = 'grab'; });

    dynLayer.add(circle);
  });

  // ── Robot icon: dot + heading line ────────────────────────
  if (robotPose && currentMapMeta) {
    const p = worldToStagePixel(robotPose.x, robotPose.y);
    if (p) {
      // yaw: ROS convention CCW positive. Canvas Y is flipped vs world Y.
      // world +Y = canvas up → negate sin component.
      const yawRad = robotPose.yaw;
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
function toggleAutoMode() {
  if (pathNodes.length < 2) { alert('Please create at least 2 waypoints'); return; }

  isAutoMode = !isAutoMode;
  const button = document.getElementById('autoButton');

  if (isAutoMode) {
    button.textContent = 'Stop Autonomous';
    button.classList.add('active-auto');
    socket.emit('start_autonomous', {
      robot_x:   robotPose ? robotPose.x   : 0,
      robot_y:   robotPose ? robotPose.y   : 0,
      robot_yaw: robotPose ? robotPose.yaw : 0,
      waypoints: pathNodes.map(n => ({ x: n.worldX, y: n.worldY })),
    });
  } else {
    button.textContent = 'Start Autonomous';
    button.classList.remove('active-auto');
    socket.emit('stop_autonomous');
  }
}

// ═══════════════════════════════════════════════════════════════
//  Reset Map Zoom / Follow Robot Toggle
// ═══════════════════════════════════════════════════════════════
function resetMapZoom() {
  stage.scale({ x: 1, y: 1 });
  stage.position({ x: 0, y: 0 });
  stage.batchDraw();
}

function toggleFollowRobot() {
  followRobot = !followRobot;
  document.getElementById('followBtn').textContent =
    `Follow Robot: ${followRobot ? 'ON' : 'OFF'}`;
}

// ═══════════════════════════════════════════════════════════════
//  Streaming Toggle
// ═══════════════════════════════════════════════════════════════
function toggleStreaming(enabled) {
  socket.emit('toggle_streaming', { enabled });
}

// ═══════════════════════════════════════════════════════════════
//  Recording
// ═══════════════════════════════════════════════════════════════
function setRecordingState(active, dirname) {
  const statusEl   = document.getElementById('recordingStatus');
  const inputEl    = document.getElementById('dirnameInput');
  const dirnameEl  = document.getElementById('dirnameDisplay');

  statusEl.textContent = active ? 'Recording' : 'Stopped';
  statusEl.classList.toggle('active', active);

  inputEl.style.display   = active ? 'none' : '';
  dirnameEl.style.display = active ? ''     : 'none';
  if (active) {
    // Use provided dirname, or fall back to what's in the input box
    dirnameEl.textContent = dirname || inputEl.value || '—';
  }

  document.getElementById('btn-rec-start').style.display = active ? 'none' : '';
  document.getElementById('btn-rec-stop').style.display  = active ? ''     : 'none';
}

function startRecording() {
  const dirname = document.getElementById('dirnameInput').value.trim();
  if (!dirname) { alert('Please enter the dir name to save.'); return; }
  setTimeout(() => {
    socket.emit('start_recording', dirname);
    setRecordingState(true, dirname);
  }, 100);
}

function stopRecording() {
  socket.emit('stop_recording');
  setRecordingState(false);
}

// Sync recording state on reconnect
socket.on('recording_status', (data) => {
  setRecordingState(data.active, data.dirname);
});

function ensureFrontSemanticCanvas() {
  const imageEl = document.getElementById('semantic-front-image');
  const canvasEl = document.getElementById('front-semantic-canvas');
  if (!imageEl || !canvasEl) return null;

  const width = imageEl.clientWidth || imageEl.naturalWidth || 0;
  const height = imageEl.clientHeight || imageEl.naturalHeight || 0;
  if (!width || !height) return null;

  if (canvasEl.width !== width || canvasEl.height !== height) {
    canvasEl.width = width;
    canvasEl.height = height;
  }

  return {
    canvasEl,
    ctx: canvasEl.getContext('2d'),
    width,
    height,
  };
}

function drawDetectionLabel(ctx, text, x, y, color) {
  ctx.font = '600 11px sans-serif';
  const textWidth = Math.ceil(ctx.measureText(text).width);
  const labelHeight = 20;
  const labelWidth = textWidth + 14;
  const drawX = Math.max(4, Math.min(x, ctx.canvas.width - labelWidth - 4));
  const drawY = Math.max(4, y - labelHeight - 6);

  ctx.fillStyle = color;
  ctx.fillRect(drawX, drawY, labelWidth, labelHeight);
  ctx.fillStyle = '#ffffff';
  ctx.fillText(text, drawX + 7, drawY + 13);
}

function drawFrontSemanticOverlay() {
  const bundle = ensureFrontSemanticCanvas();
  if (!bundle) return;

  const { ctx, width, height } = bundle;
  ctx.clearRect(0, 0, width, height);

  if (!frontSemantics || !Array.isArray(frontSemantics.detections)) return;

  const frameWidth = Number(frontSemantics.frame_width) || width;
  const frameHeight = Number(frontSemantics.frame_height) || height;
  const scaleX = width / Math.max(frameWidth, 1);
  const scaleY = height / Math.max(frameHeight, 1);

  frontSemantics.detections.forEach((det) => {
    if (!det || !det.bbox) return;
    const color = det.compliance === 'compliant' ? '#22c55e' : '#ef4444';
    const x = Number(det.bbox.x_min) * scaleX;
    const y = Number(det.bbox.y_min) * scaleY;
    const w = Math.max(2, (Number(det.bbox.x_max) - Number(det.bbox.x_min)) * scaleX);
    const h = Math.max(2, (Number(det.bbox.y_max) - Number(det.bbox.y_min)) * scaleY);

    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = det.persisted ? 1.5 : 2.5;
    ctx.setLineDash(det.persisted ? [6, 4] : []);
    ctx.strokeRect(x, y, w, h);
    ctx.restore();

    const complianceLabel = det.compliance === 'compliant' ? 'COMPLIANT' : 'NON COMPLIANT';
    const labelParts = [String(det.label || 'unknown').toUpperCase(), complianceLabel];
    if (det.distance != null) labelParts.push(`${Number(det.distance).toFixed(1)}m`);
    if (det.persisted) labelParts.push('HOLD');
    drawDetectionLabel(ctx, labelParts.join(' • '), x, y, color);
  });
}

// ═══════════════════════════════════════════════════════════════
//  Obstacle Detection Overlay
// ═══════════════════════════════════════════════════════════════
const ACTION_COLORS = {
  GO:          '#22c55e',
  SLOW:        '#f59e0b',
  CRAWL:       '#f59e0b',
  CAUTION_CAP: '#f59e0b',
  STOP:        '#ef4444',
  YIELD:       '#ef4444',
  GAP_STEER:   '#06b6d4',
  FAILSAFE:    '#dc2626',
};
const COMPLIANCE_COLORS = {
  compliant:     '#22c55e',
  semi_compliant:'#f59e0b',
  non_compliant: '#ef4444',
  none:          '#4b5563',
};

socket.on('obstacle_decision', (rawJson) => {
  try {
    const d = typeof rawJson === 'string' ? JSON.parse(rawJson) : rawJson;
    updateObstacleOverlay(d);
  } catch (e) {}
});

function updateObstacleOverlay(d) {
  const compliance = d.primary_semantic_compliance || d.compliance || 'none';
  const action = d.action || 'GO';
  const badgeEl = document.getElementById('obs-compliance-badge');
  const actionEl = document.getElementById('obs-action');
  const layerEl = document.getElementById('obs-layer');
  const priorityEl = document.getElementById('obs-priority');
  const summaryCardEl = document.getElementById('obs-summary-card');
  const seenEl = document.getElementById('obs-seen');
  const streamEl = document.getElementById('obs-stream');
  const reactionEl = document.getElementById('obs-reaction');
  const detailEl = document.getElementById('obs-detail');
  if (!badgeEl || !actionEl || !seenEl || !reactionEl || !detailEl) return;

  const seenLabel = d.primary_semantic_label || d.dominant_label || null;
  const seenDistance = d.primary_semantic_distance ?? d.threat_distance;
  const seenConf = d.primary_semantic_confidence;
  const reactionMs = d.reaction_latency_ms;
  const semanticAgeMs = d.semantic_age_ms;
  const semanticStatus = d.semantic_status || 'offline';
  const semanticCount = d.semantic_detection_count ?? 0;
  const lidarAgeMs = d.lidar_age_ms;
  const source = d.sem_source || 'none';
  const layer = d.decision_layer || 'semantic';

  badgeEl.textContent = compliance === 'none' ? 'NONE' : compliance.replace('_', ' ').toUpperCase();
  badgeEl.style.background = COMPLIANCE_COLORS[compliance] || '#4b5563';

  actionEl.textContent = action;
  actionEl.style.background = ACTION_COLORS[action] || '#4b5563';
  actionEl.style.color = '#ffffff';

  if (layerEl) {
    layerEl.textContent = layer.toUpperCase();
    layerEl.classList.toggle('neutral', layer === 'semantic');
  }
  if (priorityEl) {
    const priorityStop = Boolean(d.priority_stop_active);
    priorityEl.classList.toggle('hidden', !priorityStop);
    if (priorityStop) {
      priorityEl.textContent = 'Priority Stop';
      priorityEl.style.background = '#991b1b';
      priorityEl.style.color = '#ffffff';
    } else {
      priorityEl.style.background = '';
      priorityEl.style.color = '';
    }
  }
  if (summaryCardEl) {
    summaryCardEl.style.borderColor = ACTION_COLORS[action] || 'rgba(255,255,255,0.16)';
    summaryCardEl.style.boxShadow = action === 'GO'
      ? 'none'
      : `0 0 0 1px ${ACTION_COLORS[action] || '#4b5563'} inset`;
  }

  if (seenLabel) {
    let text = seenLabel;
    if (seenDistance != null) text += ` @ ${Number(seenDistance).toFixed(1)} m`;
    if (seenConf != null) text += ` (${Math.round(Number(seenConf) * 100)}%)`;
    seenEl.textContent = text;
  } else {
    if (semanticStatus === 'offline') {
      seenEl.textContent = 'Semantic stream offline';
    } else if (semanticStatus === 'stale') {
      seenEl.textContent = 'Waiting for fresh semantic frame';
    } else if (source === 'lidar_fallback') {
      seenEl.textContent = 'LiDAR fallback only';
    } else {
      seenEl.textContent = 'Clear';
    }
  }
  seenEl.classList.toggle('alert', compliance === 'non_compliant' || Boolean(d.priority_stop_active));

  if (streamEl) {
    streamEl.textContent = semanticStatus === 'live'
      ? `LIVE (${semanticCount})`
      : semanticStatus.toUpperCase();
    streamEl.classList.toggle('neutral', semanticStatus !== 'live');
  }

  reactionEl.textContent = reactionMs != null ? `${Math.round(Number(reactionMs))} ms` : '—';
  reactionEl.classList.toggle('neutral', reactionMs == null);

  const parts = [];
  if (semanticStatus === 'offline') {
    parts.push('No semantic stream detected');
  } else if (semanticStatus === 'stale') {
    parts.push('Waiting for fresh semantic detections');
  } else if (source === 'yolo') {
    parts.push('YOLO live');
  } else if (source === 'lidar_fallback') {
    parts.push('LiDAR fallback');
  }
  if (d.priority_stop_active) parts.push('Priority stop');
  if (d.reason) parts.push(d.reason);
  if (d.cmd_age_ms != null) parts.push(`cmd ${Math.round(Number(d.cmd_age_ms))} ms`);
  if (semanticAgeMs != null) parts.push(`sem ${Math.round(Number(semanticAgeMs))} ms`);
  if (lidarAgeMs != null) parts.push(`lidar ${Math.round(Number(lidarAgeMs))} ms`);
  detailEl.textContent = parts.length ? parts.join(' • ') : 'Waiting for navigation decision...';
}
