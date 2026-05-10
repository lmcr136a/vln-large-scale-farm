// ═══════════════════════════════════════════════════════════════
//  path_plan.js
//  Click to place start point + waypoints on the live/saved map.
//  Saves mission to server → pushed to Jetson config.
// ═══════════════════════════════════════════════════════════════

const host   = window.location.hostname;
const socket = io(`http://${host}:8000`, { transports: ['polling'] });

let stage, mapLayer, dynLayer, mapImage;
let currentMapMeta = null;
let startPoint     = null;   // { x, y } world coords
let waypoints      = [];     // [{ x, y }, ...]
let mode           = 'waypoint';
let robotPose      = null;
let isFirstMap     = true;

const ZOOM_MIN = 0.1, ZOOM_MAX = 20, ZOOM_STEP = 1.15;

// ── Stage init ────────────────────────────────────────────────
function initStage() {
  const container = document.getElementById('map-konva-container');
  const W = container.clientWidth  || 800;
  const H = container.clientHeight || 600;

  stage = new Konva.Stage({ container: 'map-konva-container', width: W, height: H, draggable: true });
  mapLayer = new Konva.Layer();
  dynLayer  = new Konva.Layer();
  stage.add(mapLayer);
  stage.add(dynLayer);

  mapImage = new Konva.Image({ x: 0, y: 0, listening: false });
  mapLayer.add(mapImage);

  stage.container().addEventListener('wheel', onWheel, { passive: false });

  // Use DOM-level mousedown/mouseup instead of stage.on('click') —
  // Konva swallows click events on draggable stages in some versions.
  let _downPos      = null;
  let _downOnMarker = false;   // set when mousedown lands on an existing marker

  stage.container().addEventListener('mousedown', e => {
    _downPos = { x: e.clientX, y: e.clientY };
  });
  stage.container().addEventListener('mouseup', e => {
    if (!_downPos) return;
    const dx = e.clientX - _downPos.x;
    const dy = e.clientY - _downPos.y;
    _downPos = null;
    if (Math.sqrt(dx * dx + dy * dy) > 5) return;  // drag, not click
    if (_downOnMarker) { _downOnMarker = false; return; }  // marker handled by Konva
    onMapClick(e);
  });
  window.addEventListener('resize', () => {
    stage.width(container.clientWidth);
    stage.height(container.clientHeight);
    stage.batchDraw();
  });
}
initStage();

function onWheel(e) {
  e.preventDefault();
  const oldScale = stage.scaleX();
  const ptr = stage.getPointerPosition();
  const anchor = {
    x: (ptr.x - stage.x()) / oldScale,
    y: (ptr.y - stage.y()) / oldScale,
  };
  const newScale = e.deltaY < 0
    ? Math.min(oldScale * ZOOM_STEP, ZOOM_MAX)
    : Math.max(oldScale / ZOOM_STEP, ZOOM_MIN);
  stage.scale({ x: newScale, y: newScale });
  stage.position({ x: ptr.x - anchor.x * newScale, y: ptr.y - anchor.y * newScale });
  stage.batchDraw();
}

// ── Coordinate transforms ─────────────────────────────────────

// World → image pixel  (same rotation as save_map_glim._rotate_xy)
function worldToPixel(wx, wy) {
  if (!currentMapMeta) return null;
  const { resolution, origin_x, origin_y, height, rot_angle } = currentMapMeta;
  const c = Math.cos(rot_angle), s = Math.sin(rot_angle);
  const rx =  c * wx + s * wy;
  const ry = -s * wx + c * wy;
  return {
    x: (rx - origin_x) / resolution,
    y: height - (ry - origin_y) / resolution,
  };
}

// Image pixel → world  (inverse of above)
function pixelToWorld(px, py) {
  if (!currentMapMeta) return null;
  const { resolution, origin_x, origin_y, height, rot_angle } = currentMapMeta;
  const rx = origin_x + px * resolution;
  const ry = origin_y + (height - py) * resolution;
  // Inverse rotation: R^T where R = [[c,s],[-s,c]]
  const c = Math.cos(rot_angle), s = Math.sin(rot_angle);
  return {
    x:  c * rx - s * ry,
    y:  s * rx + c * ry,
  };
}

// ── Mode ──────────────────────────────────────────────────────
function setMode(m) {
  mode = m;
  document.getElementById('btn-start').classList.toggle('active',    m === 'start');
  document.getElementById('btn-waypoint').classList.toggle('active', m === 'waypoint');
}

// ── Map click ─────────────────────────────────────────────────
function onMapClick(e) {
  if (!currentMapMeta) return;

  // Compute image-space coordinates explicitly
  const rect  = stage.container().getBoundingClientRect();
  const scale = stage.scaleX();
  const sPos  = stage.position();
  const pos   = {
    x: (e.clientX - rect.left  - sPos.x) / scale,
    y: (e.clientY - rect.top   - sPos.y) / scale,
  };

  const world = pixelToWorld(pos.x, pos.y);
  if (!world) return;

  if (mode === 'start') {
    startPoint = world;
    setMode('waypoint');
  } else {
    waypoints.push(world);
  }
  redraw();
}

function undoLast() {
  if (waypoints.length > 0) waypoints.pop();
  else startPoint = null;
  redraw();
}

function clearAll() {
  if (!confirm('Clear all points?')) return;
  startPoint = null;
  waypoints  = [];
  redraw();
}

// ── Draw ──────────────────────────────────────────────────────
function redraw() {
  if (!dynLayer) return;
  dynLayer.destroyChildren();

  const allPoints = startPoint ? [startPoint, ...waypoints] : waypoints;

  // Path line (no loop-back)
  if (allPoints.length > 1) {
    const pts = allPoints.flatMap(p => {
      const px = worldToPixel(p.x, p.y);
      return px ? [px.x, px.y] : [];
    });
    if (pts.length >= 4) {
      dynLayer.add(new Konva.Line({
        points: pts,
        stroke: 'rgba(200,200,200,0.5)',
        strokeWidth: 1,
        dash: [6, 3],
        listening: false,
      }));
    }
  }

  // Start point — white square
  if (startPoint) {
    const p = worldToPixel(startPoint.x, startPoint.y);
    if (p) drawMarker(p.x, p.y, 'start', () => { startPoint = null; redraw(); });
  }

  // Waypoints — last = blue square (end), others = gray circle
  waypoints.forEach((wp, i) => {
    const p = worldToPixel(wp.x, wp.y);
    if (!p) return;
    const isEnd = i === waypoints.length - 1;
    drawMarker(p.x, p.y, isEnd ? 'end' : 'mid', () => { waypoints.splice(i, 1); redraw(); });
  });

  // Robot pose
  if (robotPose && currentMapMeta) {
    const p = worldToPixel(robotPose.x, robotPose.y);
    if (p) {
      const yaw = robotPose.yaw - currentMapMeta.rot_angle;
      const L   = 8;
      dynLayer.add(new Konva.Circle({
        x: p.x, y: p.y, radius: 4,
        fill: 'rgba(255,100,0,0.95)', stroke: 'white', strokeWidth: 1.5, listening: false,
      }));
      dynLayer.add(new Konva.Arrow({
        points: [p.x, p.y, p.x + Math.cos(yaw) * L, p.y - Math.sin(yaw) * L],
        pointerLength: 5, pointerWidth: 4,
        fill: 'rgba(255,100,0,0.95)', stroke: 'rgba(255,100,0,0.95)',
        strokeWidth: 2, listening: false,
      }));
    }
  }

  dynLayer.draw();
}

function drawMarker(x, y, type, onRemove) {
  const g = new Konva.Group({ x, y });

  if (type === 'start' || type === 'end') {
    const S = 10;
    g.add(new Konva.Rect({
      x: -S / 2, y: -S / 2, width: S, height: S,
      fill: type === 'start' ? 'white' : '#4499ff',
      stroke: 'black', strokeWidth: 1,
    }));
  } else {
    g.add(new Konva.Circle({
      radius: 5,
      fill: 'rgb(200,200,200)',
      stroke: 'black', strokeWidth: 1,
    }));
  }

  g.hitFunc(ctx => {
    ctx.beginPath(); ctx.arc(0, 0, 12, 0, Math.PI * 2); ctx.closePath();
  });
  g.on('mousedown', () => { _downOnMarker = true; });
  g.on('click',      e  => { e.cancelBubble = true; onRemove(); });
  g.on('mouseenter', () => stage.container().style.cursor = 'pointer');
  g.on('mouseleave', () => stage.container().style.cursor = 'grab');
  dynLayer.add(g);
}

// ── Save / Load ───────────────────────────────────────────────
async function saveMission() {
  if (!startPoint && waypoints.length === 0) {
    alert('No points to save. Set a start point and at least one waypoint.');
    return;
  }
  const status = document.getElementById('save-status');
  status.textContent = 'Saving…';
  try {
    const r = await fetch('/mission', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ start: startPoint, waypoints }),
    });
    const data = await r.json();
    if (data.ok) {
      status.textContent = `✓ Saved (${waypoints.length} wpts)`;
      setTimeout(() => status.textContent = '', 4000);
    } else {
      status.textContent = '✗ Save failed';
    }
  } catch (e) {
    status.textContent = '✗ Error';
    console.error(e);
  }
}

async function loadMission() {
  try {
    const r = await fetch('/mission');
    if (!r.ok) return;
    const data = await r.json();
    startPoint = data.start      || null;
    waypoints  = data.waypoints  || [];
    redraw();
    console.log(`[mission] loaded — start=${!!startPoint} waypoints=${waypoints.length}`);
  } catch (e) {
    console.warn('[mission] no existing mission');
  }
}

// ── Socket events ─────────────────────────────────────────────
socket.on('connect', () => {
  console.log('[planner] connected');
  loadMission();
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

  if (!mapImage) return;
  const img = new window.Image();
  img.onload = () => {
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
    redraw();
  };
  img.src = data.image_data
    ? 'data:image/png;base64,' + data.image_data
    : `/saved_map.png?v=${data.version}`;

  console.log(`[map] ${data.width}×${data.height} rot=${(data.rot_angle * 180 / Math.PI).toFixed(1)}deg`);
});

socket.on('robot_pose', (data) => {
  robotPose = data;
  redraw();
});