// ═══════════════════════════════════════════════════════════════
//  path_plan.js  —  Click to build a path on the live/saved map.
//
//  UX:
//    • Click empty map  →  add waypoint
//                          (first = white square / start)
//    • Click start marker (≥2 pts)  →  toggle closed loop
//    • Click any other marker  →  remove it
// ═══════════════════════════════════════════════════════════════

const host   = window.location.hostname;
const socket = io(`http://${host}:8000`, { transports: ['websocket', 'polling'] });

// ── State ─────────────────────────────────────────────────────
let stage, mapLayer, dynLayer, mapImage;
let currentMapMeta = null;
let waypoints      = [];     // [{x,y}]  waypoints[0] = start
let isLoop         = false;
let robotPose      = null;
let isFirstMap     = true;
let _downPos       = null;
let _downOnMarker  = false;
let _markerAction  = null;   // action queued from marker mousedown, executed on mouseup

const ZOOM_MIN = 0.1, ZOOM_MAX = 20, ZOOM_STEP = 1.15;

// ── Stage init ────────────────────────────────────────────────
function initStage() {
  const container = document.getElementById('map-konva-container');
  const W = container.clientWidth  || 800;
  const H = container.clientHeight || 600;

  stage    = new Konva.Stage({ container: 'map-konva-container', width: W, height: H, draggable: true });
  mapLayer = new Konva.Layer();
  dynLayer = new Konva.Layer();
  stage.add(mapLayer);
  stage.add(dynLayer);

  mapImage = new Konva.Image({ x: 0, y: 0, listening: false });
  mapLayer.add(mapImage);

  stage.container().addEventListener('wheel', onWheel, { passive: false });
  stage.container().addEventListener('mousedown', e => { _downPos = { x: e.clientX, y: e.clientY }; });
  stage.container().addEventListener('mouseup', e => {
    if (!_downPos) return;
    const dx = e.clientX - _downPos.x, dy = e.clientY - _downPos.y;
    _downPos = null;
    if (Math.sqrt(dx*dx + dy*dy) > 5) return;
    if (_downOnMarker) {
      _downOnMarker = false;
      const action = _markerAction;
      _markerAction = null;
      if (action) action();
      return;
    }
    onMapClick(e);
  });
  window.addEventListener('resize', () => {
    stage.width(container.clientWidth);
    stage.height(container.clientHeight);
    stage.batchDraw();
  });
}
initStage();

// ── Zoom ──────────────────────────────────────────────────────
function onWheel(e) {
  e.preventDefault();
  const s0  = stage.scaleX();
  const ptr = stage.getPointerPosition();
  const anc = { x: (ptr.x - stage.x()) / s0, y: (ptr.y - stage.y()) / s0 };
  const s1  = e.deltaY < 0 ? Math.min(s0 * ZOOM_STEP, ZOOM_MAX) : Math.max(s0 / ZOOM_STEP, ZOOM_MIN);
  stage.scale({ x: s1, y: s1 });
  stage.position({ x: ptr.x - anc.x * s1, y: ptr.y - anc.y * s1 });
  stage.batchDraw();
}

// ── Transforms ────────────────────────────────────────────────
function worldToPixel(wx, wy) {
  if (!currentMapMeta) return null;
  const { resolution, origin_x, origin_y, height, rot_angle } = currentMapMeta;
  const c = Math.cos(rot_angle), s = Math.sin(rot_angle);
  return {
    x: ( c*wx + s*wy - origin_x) / resolution,
    y: height - (-s*wx + c*wy - origin_y) / resolution,
  };
}

function pixelToWorld(px, py) {
  if (!currentMapMeta) return null;
  const { resolution, origin_x, origin_y, height, rot_angle } = currentMapMeta;
  const rx = origin_x + px * resolution;
  const ry = origin_y + (height - py) * resolution;
  const c = Math.cos(rot_angle), s = Math.sin(rot_angle);
  return { x: c*rx - s*ry, y: s*rx + c*ry };
}

// ── Map click → add waypoint ──────────────────────────────────
function onMapClick(e) {
  if (!currentMapMeta) return;
  const rect  = stage.container().getBoundingClientRect();
  const scale = stage.scaleX();
  const sPos  = stage.position();
  const world = pixelToWorld(
    (e.clientX - rect.left - sPos.x) / scale,
    (e.clientY - rect.top  - sPos.y) / scale,
  );
  if (!world) return;
  waypoints.push(world);
  redraw();
}

// ── Buttons ───────────────────────────────────────────────────
function undoLast() {
  if (!waypoints.length) return;
  waypoints.pop();
  if (waypoints.length < 2) isLoop = false;
  redraw();
}
function clearAll() {
  if (!waypoints.length) return;
  if (!confirm('Clear all points?')) return;
  waypoints = []; isLoop = false; redraw();
}

// ── Draw ──────────────────────────────────────────────────────
function redraw() {
  if (!dynLayer) return;
  dynLayer.destroyChildren();
  const n = waypoints.length;

  // Path line
  if (n > 1) {
    const pts = waypoints.flatMap(p => { const px = worldToPixel(p.x,p.y); return px ? [px.x,px.y] : []; });
    if (pts.length >= 4)
      dynLayer.add(new Konva.Line({ points: pts, stroke: 'rgba(200,200,200,0.5)', strokeWidth: 1, dash: [6,3], listening: false }));
    // Loop closing line
    if (isLoop) {
      const p0 = worldToPixel(waypoints[0].x, waypoints[0].y);
      const pN = worldToPixel(waypoints[n-1].x, waypoints[n-1].y);
      if (p0 && pN)
        dynLayer.add(new Konva.Line({ points: [pN.x,pN.y,p0.x,p0.y], stroke: 'rgba(0,220,180,0.6)', strokeWidth: 1, dash: [4,4], listening: false }));
    }
  }

  // Markers
  waypoints.forEach((wp, i) => {
    const p = worldToPixel(wp.x, wp.y);
    if (!p) return;
    const isStart = i === 0;
    const isEnd   = i === n-1 && n > 1 && !isLoop;
    // Start marker: toggle loop (if ≥2 pts), else remove
    const onAction = (isStart && n > 1)
      ? () => { isLoop = !isLoop; redraw(); }
      : () => { waypoints.splice(i,1); if (waypoints.length < 2) isLoop = false; redraw(); };
    drawMarker(p.x, p.y, isStart ? 'start' : isEnd ? 'end' : 'mid', onAction);
  });

  // Robot
  if (robotPose && currentMapMeta) {
    const p = worldToPixel(robotPose.x, robotPose.y);
    if (p) {
      const yaw = robotPose.yaw - currentMapMeta.rot_angle;
      dynLayer.add(new Konva.Circle({ x:p.x,y:p.y,radius:4,fill:'rgba(255,100,0,0.95)',stroke:'white',strokeWidth:1.5,listening:false }));
      dynLayer.add(new Konva.Arrow({ points:[p.x,p.y,p.x+Math.cos(yaw)*8,p.y-Math.sin(yaw)*8],pointerLength:5,pointerWidth:4,fill:'rgba(255,100,0,0.95)',stroke:'rgba(255,100,0,0.95)',strokeWidth:2,listening:false }));
    }
  }

  // Loop status label
  const el = document.getElementById('loop-status');
  if (el) el.textContent = isLoop ? '🔁 Loop: ON' : '↗ Loop: OFF';

  dynLayer.draw();
}

function drawMarker(x, y, type, onAction) {
  const g = new Konva.Group({ x, y });
  const S = 10;
  if (type === 'start')
    g.add(new Konva.Rect({ x:-S/2,y:-S/2,width:S,height:S,fill:'white',stroke:'black',strokeWidth:1 }));
  else if (type === 'end')
    g.add(new Konva.Rect({ x:-S/2,y:-S/2,width:S,height:S,fill:'#4499ff',stroke:'black',strokeWidth:1 }));
  else
    g.add(new Konva.Circle({ radius:5,fill:'rgb(200,200,200)',stroke:'black',strokeWidth:1 }));

  // Transparent hit area (Konva.Circle on Group child, not hitFunc)
  g.add(new Konva.Circle({ radius:14,fill:'transparent',stroke:null }));

  g.on('mousedown', () => { _downOnMarker = true; _markerAction = onAction; });
  g.on('mouseenter',() => stage.container().style.cursor = 'pointer');
  g.on('mouseleave',() => stage.container().style.cursor = 'grab');
  dynLayer.add(g);
}

// ── Save / Load ───────────────────────────────────────────────
async function saveMission() {
  if (waypoints.length < 2) { alert('At least 2 points required.'); return; }
  const status = document.getElementById('save-status');
  status.textContent = 'Saving…';
  try {
    const r = await fetch('/mission', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ start: waypoints[0], waypoints: waypoints.slice(1), isLoop }),
    });
    const d = await r.json();
    status.textContent = d.ok ? `✓ Saved (${waypoints.length} pts${isLoop?', loop':''})` : '✗ Error';
    if (d.ok) setTimeout(() => status.textContent = '', 4000);
  } catch { status.textContent = '✗ Error'; }
}

async function loadMission() {
  try {
    const r = await fetch('/mission');
    if (!r.ok) return;
    const d = await r.json();
    waypoints = d.start ? [d.start, ...(d.waypoints||[])] : (d.waypoints||[]);
    isLoop    = d.isLoop || false;
    redraw();
  } catch {}
}

// ── Socket ────────────────────────────────────────────────────
socket.on('connect', () => { console.log('[planner] connected'); loadMission(); });

socket.on('map_updated', (data) => {
  currentMapMeta = { resolution:data.resolution,origin_x:data.origin_x,origin_y:data.origin_y,width:data.width,height:data.height,rot_angle:data.rot_angle??0 };
  if (!mapImage) return;
  const img = new window.Image();
  img.onload = () => {
    mapImage.image(img); mapImage.width(data.width); mapImage.height(data.height);
    mapLayer.batchDraw();
    if (isFirstMap) {
      isFirstMap = false;
      const sc = Math.min(stage.width()/data.width, stage.height()/data.height) * 0.9;
      stage.scale({x:sc,y:sc});
      stage.position({x:(stage.width()-data.width*sc)/2, y:(stage.height()-data.height*sc)/2});
      stage.batchDraw();
    }
    redraw();
  };
  img.src = data.image_data ? 'data:image/png;base64,'+data.image_data : `/saved_map.png?v=${data.version}`;
});

socket.on('robot_pose', (data) => {
  console.log(`[robot_pose] x=${data.x?.toFixed(2)} y=${data.y?.toFixed(2)} yaw=${data.yaw != null ? (data.yaw * 180 / Math.PI).toFixed(1) + 'deg' : 'N/A'}`);
  robotPose = data;
  redraw();
});