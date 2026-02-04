const host = window.location.hostname;
console.log('Connecting to:', `http://${host}:8000`);
const socket = io(`http://${host}:8000`);
const keyState = {};
let frameInterval = null;
let currentMapInfo = null;
let isPathMode = false;
let isAutoMode = false;
let pathNodes = [];

// Connection status
socket.on('connect', () => {
  console.log('Socket.IO connected! ID:', socket.id);
});

socket.on('disconnect', () => {
  console.log('Socket.IO disconnected');
});

socket.on('connect_error', (error) => {
  console.error('Connection error:', error);
});

// Heartbeat
setInterval(() => {
    socket.emit('heartbeat');
}, 500);

// Window blur event - clear all keys
window.addEventListener('blur', () => {
    for (const key in keyState) {
        if (keyState[key]) {
            keyState[key] = false;
            socket.emit('keyup', key);
        }
    }
});

// Keydown event
window.addEventListener("keydown", (e) => {
  if (document.activeElement.tagName === "INPUT") return;
  if (isAutoMode) return;

  const key = e.key;
  if (!keyState[key]) {
    keyState[key] = true;
    socket.emit("keydown", key);
  }
});

// Keyup event
window.addEventListener("keyup", (e) => {
  if (document.activeElement.tagName === "INPUT") return;
  if (isAutoMode) return;

  const key = e.key;
  if (keyState[key]) {
    keyState[key] = false;
    socket.emit("keyup", key);
  }
});

// Socket event listeners
let frontFrameCount = 0;
let backFrameCount = 0;

const sysmonDiv = document.getElementById("sysmon");

socket.on("sysmon", (data) => {
  if (!data || typeof data !== "object") {
    sysmonDiv.textContent = "System info unavailable";
    return;
  }
  
  const cpu = data.cpu;
  const mem = data.mem;
  const used = data.used_gb;
  const total = data.total_gb;
  const usedTB = (used / 1024).toFixed(1);
  const totalTB = (total / 1024).toFixed(1);
  const pct = data.used_pct;
  const linear_mps = data.linear_mps;
  const linear_mph = data.linear_mph;
  const wifi = data.wifi || "Unknown";

  if (
    typeof cpu !== "number" ||
    typeof mem !== "number" ||
    typeof used !== "number" ||
    typeof total !== "number" ||
    typeof pct !== "number"
  ) {
    sysmonDiv.textContent = "Invalid system data";
    return;
  }

  sysmonDiv.textContent =
    `Wi-Fi: ${wifi} | CPU: ${cpu.toFixed(1)}% | SSD: ${usedTB} / ${totalTB} TB | Speed: ${linear_mps.toFixed(1)} m/s`;
});

socket.on('map_update', (data) => {
  currentMapInfo = data.info;
});

socket.on('auto_mode_completed', () => {
  if (isAutoMode) {
    toggleAutoMode();
  }
});

socket.on('robot_status', (data) => {
  const statusDiv = document.getElementById('robot-status');
  if (data.status) {
    statusDiv.textContent = data.status;
    statusDiv.style.display = 'block';
  } else {
    statusDiv.style.display = 'none';
  }
});

socket.on('waypoint_reached', (data) => {
  const index = data.index;
  if (index >= 0 && index < pathNodes.length) {
    pathNodes[index].reached = true;
    drawPathNodes();
  }
});

// Toggle path mode
function togglePathMode() {
  isPathMode = !isPathMode;
  const indicator = document.getElementById('modeIndicator');
  const button = event.target;
  const autoControl = document.getElementById('auto-mode-control');
  
  if (isPathMode) {
    indicator.textContent = 'Mode: PATH';
    indicator.style.backgroundColor = 'rgba(0, 200, 200, 0.6)';
    button.textContent = 'Stop Path Mode & Resume Mapping';
    autoControl.style.display = 'block';
    pathNodes = [];
    
    // Check if currentMapInfo is available
    if (!currentMapInfo) {
      console.warn('Map info not available yet');
      return;
    }
    
    // Check robot position
    if (currentMapInfo.robot_x === undefined || currentMapInfo.robot_y === undefined) {
      console.warn('Robot position not available');
      return;
    }
    
    const robotX = currentMapInfo.robot_x;
    const robotY = currentMapInfo.robot_y;
    
    const imgX = Math.round((robotX - currentMapInfo.origin_x) / currentMapInfo.resolution);
    const imgY = Math.round((robotY - currentMapInfo.origin_y) / currentMapInfo.resolution);
    
    const img = document.getElementById('map-image');
    const scaleX = img.clientWidth / currentMapInfo.width;
    const scaleY = img.clientHeight / currentMapInfo.height;
    
    const displayX = imgX * scaleX;
    const displayY = imgY * scaleY;
    
    pathNodes.push({
      imgX: imgX,
      imgY: imgY,
      worldX: robotX,
      worldY: robotY,
      displayX: displayX,
      displayY: displayY,
      reached: false
    });
    
    console.log('Initial robot position added:', pathNodes[0]);
    
    // Update canvas and draw
    setTimeout(() => {
      drawPathNodes();
    }, 50);
    
  } else {
    indicator.textContent = 'Mode: MAP';
    indicator.style.backgroundColor = 'rgba(100, 100, 100, 0.6)';
    button.textContent = 'Stop Creating Map & Start Path Mode';
    autoControl.style.display = 'none';
    pathNodes = [];
    drawPathNodes();
  }
}

// Toggle autonomous mode
function toggleAutoMode() {
  if (!isPathMode) {
    alert('Please enter Path Mode first');
    return;
  }
  
  if (pathNodes.length < 2) {
    alert('Please create at least 2 waypoints');
    return;
  }

  isAutoMode = !isAutoMode;
  const indicator = document.getElementById('autoIndicator');
  const button = document.getElementById('autoButton');
  
  if (isAutoMode) {
    indicator.textContent = 'AUTO: ON';
    indicator.style.backgroundColor = 'rgba(0, 255, 0, 0.8)';
    button.textContent = 'Stop Autonomous Driving';
    button.style.backgroundColor = 'rgba(200, 0, 0, 0.8)';
    
    const waypoints = pathNodes.map(node => ({
      x: node.worldX,
      y: node.worldY
    }));
    
    socket.emit('start_autonomous', {
      robot_x: currentMapInfo.robot_x,
      robot_y: currentMapInfo.robot_y,
      robot_yaw: currentMapInfo.robot_yaw,
      waypoints: waypoints
    });
  } else {
    indicator.textContent = 'AUTO: OFF';
    indicator.style.backgroundColor = 'rgba(0, 100, 0, 0.6)';
    button.textContent = 'Start Autonomous Driving';
    button.style.backgroundColor = 'rgba(0, 150, 0, 0.8)';
    
    socket.emit('stop_autonomous');
  }
}

// Draw path nodes on canvas
function drawPathNodes() {
  const canvas = document.getElementById('map-canvas');
  const img = document.getElementById('map-image');
  
  // Wait if image not loaded
  if (img.clientWidth === 0 || img.clientHeight === 0) {
    console.warn('Map image not loaded yet');
    return;
  }
  
  canvas.width = img.clientWidth;
  canvas.height = img.clientHeight;
  
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  
  if (pathNodes.length === 0) return;
  
  console.log(`Drawing ${pathNodes.length} nodes`);
  
  // Draw lines
  if (pathNodes.length > 1) {
    ctx.strokeStyle = 'rgb(0, 155, 155)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    
    for (let i = 0; i < pathNodes.length; i++) {
      const node = pathNodes[i];
      if (i === 0) {
        ctx.moveTo(node.displayX, node.displayY);
      } else {
        ctx.lineTo(node.displayX, node.displayY);
      }
    }
    
    if (pathNodes.length > 2) {
      ctx.lineTo(pathNodes[0].displayX, pathNodes[0].displayY);
    }
    
    ctx.stroke();
  }
  
  // Draw points
  for (let i = 0; i < pathNodes.length; i++) {
    const node = pathNodes[i];
    ctx.beginPath();
    if (node.reached) {
      ctx.fillStyle = 'rgb(255, 255, 0)';
    } else if (i === 0) {
      ctx.fillStyle = 'rgb(255, 0, 255)';  // First point is magenta
    } else {
      ctx.fillStyle = 'rgb(0, 255, 255)';
    }
    ctx.arc(node.displayX, node.displayY, 8, 0, 2 * Math.PI);
    ctx.fill();
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 1;
    ctx.stroke();
  }
}

// Map click handler
document.addEventListener('DOMContentLoaded', () => {
  document.getElementById('map-image').addEventListener('click', (e) => {
    if (!currentMapInfo) {
      return;
    }

    if (isAutoMode) {
      return;
    }

    const img = e.target;
    const rect = img.getBoundingClientRect();
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;

    const scaleX = currentMapInfo.width / img.clientWidth;
    const scaleY = currentMapInfo.height / img.clientHeight;

    const imgX = Math.round(clickX * scaleX);
    const imgY = Math.round(clickY * scaleY);

    const worldX = currentMapInfo.origin_x + imgX * currentMapInfo.resolution;
    const worldY = currentMapInfo.origin_y + imgY * currentMapInfo.resolution;
    
    if (isPathMode) {
      pathNodes.push({
        imgX: imgX,
        imgY: imgY,
        worldX: worldX,
        worldY: worldY,
        displayX: clickX,
        displayY: clickY,
        reached: false
      });
      drawPathNodes();
    }
    
    socket.emit('map_clicked', {
      img_x: imgX,
      img_y: imgY,
      world_x: worldX,
      world_y: worldY
    });
  });

  // Map image load event
  document.getElementById('map-image').addEventListener('load', () => {
    if (isPathMode && pathNodes.length > 0) {
      drawPathNodes();
    }
  });
});

// Recording functions
function startRecording() {
  const dirname = document.getElementById("dirnameInput").value.trim();
  if (!dirname) {
    alert("Please enter the dir name to save.");
    return;
  }
  
  setTimeout(() => {
    socket.emit("start_recording", dirname);
    document.getElementById("recordingStatus").textContent = "Recording";
  }, 100);
}

function stopRecording() {
  if (frameInterval) {
    clearInterval(frameInterval);
    frameInterval = null;
  }
  socket.emit("stop_recording");
  document.getElementById("recordingStatus").textContent = "Stopped";
}

// Map image auto-refresh
setInterval(() => {
  const mapImg = document.getElementById('map-image');
  mapImg.src = `http://${host}:8000/map_latest?t=${Date.now()}`;
}, 1000);

// RGB frame auto-refresh (front)
setInterval(() => {
  const img = document.getElementById('rgb-image');
  img.src = `http://${host}:8000/front_rgb?t=${Date.now()}`;
}, 100);  // 100ms = 10 FPS

// RGB frame auto-refresh (back)
setInterval(() => {
  const img = document.getElementById('back-rgb-image');
  img.src = `http://${host}:8000/back_rgb?t=${Date.now()}`;
}, 100);