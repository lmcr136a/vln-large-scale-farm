// ── Weekly schedule UI + countdown ───────────────────────────
const DAYS   = ['sun','mon','tue','wed','thu','fri','sat'];
const DAY_LBL = ['Sun','Mon','Tue','Wed','Thu','Fri','Sat'];
const TIMES  = ['5am','11am','5pm','11pm'];

let scheduleData = {enabled:true,sun:[],mon:[],tue:[],wed:[],thu:[],fri:[],sat:[]};
let _countdownTimer = null;

async function loadSchedule() {
  try {
    const r = await fetch('/schedule');
    scheduleData = await r.json();
  } catch {}
  renderCalendar();
  startCountdown();
}

async function saveSchedule() {
  try {
    await fetch('/schedule', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify(scheduleData),
    });
  } catch {}
}

function toggleCell(day, time) {
  const arr = scheduleData[day] || [];
  const idx = arr.indexOf(time);
  if (idx >= 0) arr.splice(idx, 1);
  else arr.push(time);
  scheduleData[day] = arr;
  renderCalendar();
  saveSchedule();
}

function toggleScheduleEnabled() {
  scheduleData.enabled = !scheduleData.enabled;
  saveSchedule();
  renderCalendar();
}

function renderCalendar() {
  const el = document.getElementById('weekly-calendar');
  if (!el) return;

  const enabled = scheduleData.enabled !== false;
  const onEl = document.getElementById('sched-enabled-lbl');
  if (onEl) onEl.textContent = enabled ? 'ON' : 'OFF';

  let html = '<table class="sched-table"><thead><tr><th></th>';
  DAY_LBL.forEach(d => { html += `<th>${d}</th>`; });
  html += '</tr></thead><tbody>';

  TIMES.forEach(t => {
    html += `<tr><td class="sched-time">${t}</td>`;
    DAYS.forEach(day => {
      const active = (scheduleData[day] || []).includes(t);
      html += `<td class="sched-cell${active ? ' active' : ''}" onclick="toggleCell('${day}','${t}')">${active ? t : '–'}</td>`;
    });
    html += '</tr>';
  });
  html += '</tbody></table>';
  el.innerHTML = html;
}

function findNextSchedule() {
  if (!scheduleData.enabled) return null;
  const now = new Date();
  let best  = null;

  for (let offset = 0; offset < 7; offset++) {
    const d    = new Date(now);
    d.setDate(d.getDate() + offset);
    const day  = DAYS[d.getDay()];
    const times = scheduleData[day] || [];

    for (const t of times) {
      const parsed = _parseTime(t);
      if (parsed == null) continue;
      const candidate = new Date(d);
      candidate.setHours(parsed.h, parsed.min, 0, 0);
      if (candidate > now) {
        if (!best || candidate < best.time) {
          best = {
            time: candidate,
            label: `${DAY_LBL[d.getDay()]} ${t.toUpperCase()}`,
          };
        }
      }
    }
    if (best && offset === 0) break;
  }
  return best;
}

function _parseTime(s) {
  const m = s.match(/^(\d+)(?::(\d{2}))?(am|pm)$/i);
  if (!m) return null;
  let h = parseInt(m[1]);
  const min = m[2] ? parseInt(m[2]) : 0;
  const pm = m[3].toLowerCase() === 'pm';
  if (pm && h !== 12) h += 12;
  if (!pm && h === 12) h = 0;
  return {h, min};
}

function startCountdown() {
  if (_countdownTimer) clearInterval(_countdownTimer);
  _countdownTimer = setInterval(_updateCountdown, 1000);
  _updateCountdown();
}

function _updateCountdown() {
  const el = document.getElementById('upcoming-label');
  if (!el) return;
  const next = findNextSchedule();
  if (!next) { el.textContent = 'No upcoming schedule'; return; }
  const diff = next.time - Date.now();
  if (diff <= 0) { el.textContent = `Starting: ${next.label}`; return; }
  const dd = String(Math.floor(diff / 86400000)).padStart(2, '0');
  const hh = String(Math.floor((diff % 86400000) / 3600000)).padStart(2, '0');
  const mm = String(Math.floor((diff % 3600000)  / 60000)).padStart(2, '0');
  const ss = String(Math.floor((diff % 60000)    / 1000)).padStart(2, '0');
  el.textContent = `Upcoming: ${next.label}  (${dd}:${hh}:${mm}:${ss})`;
}

// Called from control.js socket.on('connect') and on page load
function initSchedule() {
  loadSchedule();
}