// ── Schedule: upcoming list with live countdown ───────────────
const DAYS     = ['sun','mon','tue','wed','thu','fri','sat'];
const DAY_LBL  = ['Sun','Mon','Tue','Wed','Thu','Fri','Sat'];

let scheduleData = { enabled: true };
let _countdownTimer = null;

async function loadSchedule() {
  try {
    const r = await fetch('/schedule');
    scheduleData = await r.json();
  } catch {}
  _renderUpcoming();
  _startCountdown();
}

async function saveSchedule() {
  try {
    await fetch('/schedule', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(scheduleData),
    });
  } catch {}
}

function toggleScheduleEnabled() {
  scheduleData.enabled = !scheduleData.enabled;
  saveSchedule();
  _renderUpcoming();
}

// ── Build a sorted list of the next N upcoming datetimes ──────
function _getUpcoming(n = 10) {
  if (!scheduleData.enabled) return [];
  const now = new Date();
  const results = [];

  // Look ahead up to 14 days to gather enough entries
  for (let offset = 0; offset < 14 && results.length < n; offset++) {
    const d   = new Date(now);
    d.setDate(d.getDate() + offset);
    const day = DAYS[d.getDay()];
    const times = scheduleData[day] || [];

    for (const t of times) {
      const parsed = _parseTime(t);
      if (!parsed) continue;
      const dt = new Date(d);
      dt.setHours(parsed.h, parsed.min, 0, 0);
      if (dt > now) {
        results.push({ dt, day: DAY_LBL[d.getDay()], raw: t });
      }
    }
  }

  results.sort((a, b) => a.dt - b.dt);
  return results.slice(0, n);
}

function _parseTime(s) {
  const m = s.match(/^(\d+)(?::(\d{2}))?(am|pm)$/i);
  if (!m) return null;
  let h = parseInt(m[1]);
  const min = m[2] ? parseInt(m[2]) : 0;
  const pm  = m[3].toLowerCase() === 'pm';
  if (pm && h !== 12) h += 12;
  if (!pm && h === 12) h = 0;
  return { h, min };
}

// Format "5am" → "5:00AM", "11pm" → "11:00PM", "1:05am" → "1:05AM"
function _fmtTime(raw) {
  const m = raw.match(/^(\d+)(?::(\d{2}))?(am|pm)$/i);
  if (!m) return raw.toUpperCase();
  const h   = m[1];
  const min = m[2] || '00';
  const ampm = m[3].toUpperCase();
  return `${h}:${min}${ampm}`;
}

function _countdown(dt) {
  const diff = dt - Date.now();
  if (diff <= 0) return 'now';
  const d  = Math.floor(diff / 86400000);
  const hh = String(Math.floor((diff % 86400000) / 3600000)).padStart(2, '0');
  const mm = String(Math.floor((diff % 3600000)  / 60000)).padStart(2, '0');
  const ss = String(Math.floor((diff % 60000)    / 1000)).padStart(2, '0');
  return d > 0 ? `${d}d ${hh}:${mm}:${ss}` : `${hh}:${mm}:${ss}`;
}

// ── Render (static labels only, countdown updates separately) ─
function _renderUpcoming() {
  const enabled = scheduleData.enabled !== false;
  const lbl = document.getElementById('sched-enabled-lbl');
  if (lbl) lbl.textContent = enabled ? 'ON' : 'OFF';

  const nextEl   = document.getElementById('upcoming-next');
  const countEl  = document.getElementById('upcoming-countdown');
  const listEl   = document.getElementById('upcoming-list');
  if (!nextEl || !listEl) return;

  const items = _getUpcoming(10);

  if (!enabled || items.length === 0) {
    nextEl.textContent  = 'No upcoming schedule';
    countEl.textContent = '';
    listEl.innerHTML    = '';
    return;
  }

  const [first, ...rest] = items;
  nextEl.textContent  = `${first.day} ${_fmtTime(first.raw)}`;
  countEl.textContent = _countdown(first.dt);  // initial, timer updates this

  listEl.innerHTML = rest.map(
    item => `<div class="upcoming-item">${item.day} ${_fmtTime(item.raw)}</div>`
  ).join('');
}

// ── Live countdown tick (1 s) ─────────────────────────────────
function _startCountdown() {
  if (_countdownTimer) clearInterval(_countdownTimer);
  _countdownTimer = setInterval(() => {
    const items  = _getUpcoming(1);
    const countEl = document.getElementById('upcoming-countdown');
    if (!countEl) return;
    if (items.length === 0) { countEl.textContent = ''; return; }
    const cd = _countdown(items[0].dt);
    countEl.textContent = cd;
    // Refresh full list when the top entry changes (i.e. it just passed)
    if (cd === 'now') _renderUpcoming();
  }, 1000);
}

function initSchedule() {
  loadSchedule();
}