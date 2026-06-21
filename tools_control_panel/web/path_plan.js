// ═══════════════════════════════════════════════════════════════
//  path_plan.js  —  Click to build a path on the live/saved map.
//
//  UX:
//    • Click empty map  →  add waypoint
//                          (first = white square / start)
//    • Click start marker (≥2 pts)  →  toggle closed loop
//    • Click any other marker  →  remove it
//    • Drag any marker  →  move that waypoint
//    • Undo  →  reverses the most recent add/delete/move (in that order)
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
let landmarks      = [];     // [{id, type, lat, lon, x_enu, y_enu, number, description}]
let history        = [];     // undo stack: {type:'add'|'delete'|'move', index, point, oldPoint}

function pushHistory(action) {
  history.push(action);
  if (history.length > 200) history.shift();
}

// Convert landmark lat/lon → ENU world coords using map origin
function landmarkToEnu(lm, meta) {
  // map_state has origin_lat/lon only if the server provides it.
  // For GPS maps (rot_angle=0) the map itself is in ENU; landmarks come pre-projected
  // from the server as enu x,y.  Fall back to rough pixel estimate if not available.
  if (lm.enu_x != null && lm.enu_y != null) return { x: lm.enu_x, y: lm.enu_y };
  return null;
}

const LM_COLOURS = {
  metal_box:  '#4040cc',
  trailer:    '#cc4040',
  vehicle:    '#cc8830',
  building:   '#7840b8',
  crop_stick: '#c8a000',
};
function lmColour(type) { return LM_COLOURS[type] || '#808080'; }

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
    if (Math.sqrt(dx*dx + dy*dy) > 5) { _downOnMarker = false; _markerAction = null; return; }
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
  pushHistory({ type: 'add', index: waypoints.length - 1 });
  redraw();
}

// ── Buttons ───────────────────────────────────────────────────
function undoLast() {
  const action = history.pop();
  if (!action) return;
  if (action.type === 'add') {
    waypoints.splice(action.index, 1);
  } else if (action.type === 'delete') {
    waypoints.splice(action.index, 0, action.point);
  } else if (action.type === 'move') {
    waypoints[action.index] = action.oldPoint;
  }
  if (waypoints.length < 2) isLoop = false;
  redraw();
}
function clearAll() {
  if (!waypoints.length) return;
  if (!confirm('Clear all points?')) return;
  waypoints = []; isLoop = false; history = []; redraw();
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
      dynLayer.add(new Konva.Line({ points: pts, stroke: 'rgba(0,0,0,0.95)', strokeWidth: 3, dash: [8,4], listening: false }));
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
      : () => {
          const removed = waypoints[i];
          waypoints.splice(i, 1);
          pushHistory({ type: 'delete', index: i, point: removed });
          if (waypoints.length < 2) isLoop = false;
          redraw();
        };
    drawMarker(p.x, p.y, isStart ? 'start' : isEnd ? 'end' : 'mid', onAction, i);
  });

  // Landmarks
  landmarks.forEach(lm => {
    if (lm.enu_x == null || lm.enu_y == null) return;
    const p = worldToPixel(lm.enu_x, lm.enu_y);
    if (!p) return;
    const col = lmColour(lm.type);
    if (lm.type === 'crop_stick') {
      dynLayer.add(new Konva.Circle({ x:p.x, y:p.y, radius:5, fill:col, stroke:'black', strokeWidth:1, listening:false }));
      if (lm.number) {
        dynLayer.add(new Konva.Text({ x:p.x+7, y:p.y-7, text:String(lm.number), fontSize:10, fill:col, listening:false }));
      }
    } else {
      const S = 14;
      dynLayer.add(new Konva.Rect({ x:p.x-S/2, y:p.y-S/2, width:S, height:S, stroke:col, strokeWidth:2, fill:'transparent', listening:false }));
      const label = lm.description || lm.type;
      dynLayer.add(new Konva.Text({ x:p.x+S/2+2, y:p.y-7, text:label.substring(0,20), fontSize:10, fill:col, listening:false }));
    }
  });

  // Robot — circle sized to the 0.6 x 0.6 m footprint, heading shown as an arrow
  if (robotPose && currentMapMeta) {
    const p = worldToPixel(robotPose.x, robotPose.y);
    if (p) {
      const res     = currentMapMeta.resolution;   // m/px from map_state.json
      const half_px = 0.3 / res;                   // ROBOT_HALF_M = 0.3m (60x60cm footprint)
      const yaw     = robotPose.yaw - currentMapMeta.rot_angle;
      dynLayer.add(new Konva.Circle({
        x:p.x, y:p.y, radius:half_px,
        fill:'rgba(255,100,0,0.95)', stroke:'white', strokeWidth:1.5, listening:false,
      }));
      dynLayer.add(new Konva.Arrow({ points:[p.x,p.y,p.x+Math.cos(yaw)*half_px*1.6,p.y-Math.sin(yaw)*half_px*1.6],pointerLength:5,pointerWidth:4,fill:'rgba(255,100,0,0.95)',stroke:'rgba(255,100,0,0.95)',strokeWidth:2,listening:false }));
    }
  }

  // Loop status label
  const el = document.getElementById('loop-status');
  if (el) el.textContent = isLoop ? '🔁 Loop: ON' : '↗ Loop: OFF';

  dynLayer.draw();
}

function drawMarker(x, y, type, onAction, idx) {
  const g = new Konva.Group({ x, y, draggable: true });
  const S = 5;
  if (type === 'start')
    g.add(new Konva.Rect({ x:-S/2,y:-S/2,width:S,height:S,fill:'white',stroke:'black',strokeWidth:1 }));
  else if (type === 'end')
    g.add(new Konva.Rect({ x:-S/2,y:-S/2,width:S,height:S,fill:'#4499ff',stroke:'black',strokeWidth:1 }));
  else
    g.add(new Konva.Circle({ radius:2.5,fill:'rgb(200,200,200)',stroke:'black',strokeWidth:1 }));

  // Order number — the waypoint's position in the drive sequence (1-based), so
  // the path order is explicit and visible (not inferred from edit time).
  g.add(new Konva.Text({ x:5, y:-5, text:String(idx + 1), fontSize:9,
                         fill:'black', stroke:'white', strokeWidth:0.4, listening:false }));

  // Transparent hit area — kept tight (≈ the visible marker) so only a click
  // landing ON the point deletes/toggles it. A click even slightly off falls
  // through to the map handler and adds a new waypoint there instead.
  g.add(new Konva.Circle({ radius:5,fill:'transparent',stroke:null }));

  g.on('mousedown', () => { _downOnMarker = true; _markerAction = onAction; });
  g.on('mouseenter',() => stage.container().style.cursor = 'pointer');
  g.on('mouseleave',() => stage.container().style.cursor = 'grab');

  // Drag to move this waypoint. The stage's own click/pan handling (in
  // initStage) already ignores gestures that moved >5px, so dragging here
  // never also triggers the click-to-delete/toggle action above.
  let oldPoint = null;
  g.on('dragstart', () => { oldPoint = { ...waypoints[idx] }; });
  g.on('dragend', () => {
    const w = pixelToWorld(g.x(), g.y());
    if (w) {
      waypoints[idx] = w;
      pushHistory({ type: 'move', index: idx, oldPoint });
    }
    redraw();
  });
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
    history   = [];
    redraw();
  } catch {}
}

// ── Socket ────────────────────────────────────────────────────
// Auto-load the saved mission only on the FIRST connect. Reconnects (network
// blip, server restart) must NOT clobber a path the user is drawing/has not
// saved yet — that silent overwrite is what made edits "revert" to the old path.
let missionLoaded = false;
socket.on('connect', () => {
  console.log('[planner] connected');
  if (!missionLoaded) { missionLoaded = true; loadMission(); }
});

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

socket.on('landmarks_updated', (data) => {
  landmarks = (data.landmarks || []).map(lm => ({
    ...lm,
    enu_x: lm.enu_x ?? null,
    enu_y: lm.enu_y ?? null,
  }));
  redraw();
});