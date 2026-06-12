#!/usr/bin/env python3

import re
import sys
import time
import queue
import subprocess
import threading
from pathlib import Path
from datetime import datetime

# ── config ───────────────────────────────────────────────────────────────────
LOCAL_BASE       = Path.home() / "box/vln-large-scale-farm/data"
REMOTE_BASE      = "box:SCI/Projects/USDA-Nahyeon-Autonomous Navigation/Dataset/raw_data"
REFRESH_SEC      = 2
RECHECK_SEC      = 300    # re-verify a 'done' session this often
LOG_FILE         = Path.home() / "box_upload.log"
WIFI_CHECK_IFACE = None   # None = auto-detect

# ── state (single lock) ───────────────────────────────────────────────────────
lock = threading.Lock()
# key -> status dict: {"state": str, "pct", "speed", "eta", "xfr", "tot",
#                      "local_b", "remote_b", "checked_at"}
status       = {}
upload_q     = queue.Queue()   # sessions confirmed to need uploading
queued_keys  = set()           # keys in upload_q or being uploaded
check_q      = queue.Queue()   # sessions needing a size check
checking_keys = set()          # keys in check_q or being checked


def log(msg):
    line = f"{datetime.now().strftime('%H:%M:%S')}  {msg}"
    try:
        with open(LOG_FILE, "a") as f:
            f.write(line + "\n")
    except Exception:
        pass


# ── wifi ──────────────────────────────────────────────────────────────────────
def get_wifi_iface():
    try:
        out = subprocess.check_output(
            ["nmcli", "-t", "-f", "DEVICE,TYPE", "device", "status"], text=True
        )
        for line in out.splitlines():
            dev, typ = line.split(":", 1)
            if typ.strip() == "wifi":
                return dev.strip()
    except Exception:
        pass
    try:
        out = subprocess.check_output(["ip", "-o", "link", "show"], text=True)
        for line in out.splitlines():
            col = line.split(":")[1].strip().lower()
            if col.startswith("wl"):
                return line.split(":")[1].strip()
    except Exception:
        pass
    return None

def is_wifi_up(iface):
    if not iface:
        return False
    try:
        out = subprocess.check_output(
            ["nmcli", "-t", "-f", "DEVICE,STATE", "device", "status"], text=True
        )
        for line in out.splitlines():
            dev, state = line.split(":", 1)
            if dev.strip() == iface:
                return state.strip() in ("connected", "connected (externally)")
    except Exception:
        pass
    try:
        s = open(f"/sys/class/net/{iface}/operstate").read().strip()
        return s in ("up", "dormant")
    except Exception:
        return False


# ── size ──────────────────────────────────────────────────────────────────────
def parse_rclone_size(output: str):
    m = re.search(r'Total size:.*?\((\d+)\s*Byte', output)
    if m:
        return int(m.group(1))
    return None

def rclone_size_bytes(target: str) -> int:
    try:
        out = subprocess.check_output(
            ["rclone", "size", target], text=True, stderr=subprocess.STDOUT
        )
        b = parse_rclone_size(out)
        return b if b is not None else -1
    except subprocess.CalledProcessError as e:
        # remote folder not found -> rclone errors; treat as 0 bytes present
        if "directory not found" in (e.output or "").lower():
            return 0
        return -1
    except Exception:
        return -1


# ── scan ──────────────────────────────────────────────────────────────────────
def get_session_folders():
    result = {}
    if not LOCAL_BASE.exists():
        return result
    for site_dir in sorted(LOCAL_BASE.iterdir()):
        if not site_dir.is_dir():
            continue
        sessions = sorted([d for d in site_dir.iterdir() if d.is_dir()])
        if sessions:
            result[site_dir.name] = sessions
    return result


# ── check worker: fast size comparison, marks done or needs-upload ─────────────
def check_worker():
    while True:
        site, session = check_q.get()
        key = f"{site}/{session.name}"
        local_target  = str(session)
        remote_target = f"{REMOTE_BASE}/{site}/{session.name}"
        try:
            set_state(key, "checking")
            local_b  = rclone_size_bytes(local_target)
            remote_b = rclone_size_bytes(remote_target)
            update(key, local_b=local_b, remote_b=remote_b, checked_at=time.time())
            log(f"[{key}] check local={local_b} remote={remote_b}")
            if local_b >= 0 and local_b == remote_b:
                set_state(key, "done")
                log(f"[{key}] DONE (size match)")
            else:
                # needs uploading -> hand off to upload queue
                with lock:
                    if key not in queued_keys:
                        queued_keys.add(key)
                        status.setdefault(key, {})["state"] = "queued"
                        upload_q.put((site, session))
                        log(f"[{key}] queued for upload")
        except Exception as e:
            set_state(key, "failed")
            log(f"[{key}] CHECK EXCEPTION {e}")
        finally:
            with lock:
                checking_keys.discard(key)
            check_q.task_done()


# ── upload worker: one upload at a time ───────────────────────────────────────
def upload_worker():
    while True:
        site, session = upload_q.get()
        key = f"{site}/{session.name}"
        local_target  = str(session)
        remote_target = f"{REMOTE_BASE}/{site}/{session.name}"
        try:
            set_state(key, "uploading")
            log(f"[{key}] uploading")
            rc = do_upload(key, local_target, remote_target)
            log(f"[{key}] rclone exited rc={rc}")

            set_state(key, "checking")
            local_b  = rclone_size_bytes(local_target)
            remote_b = rclone_size_bytes(remote_target)
            update(key, local_b=local_b, remote_b=remote_b, checked_at=time.time())
            if local_b >= 0 and local_b == remote_b:
                set_state(key, "done")
                log(f"[{key}] DONE after upload")
            else:
                set_state(key, "failed")
                log(f"[{key}] FAILED verify local={local_b} remote={remote_b}")
        except Exception as e:
            set_state(key, "failed")
            log(f"[{key}] UPLOAD EXCEPTION {e}")
        finally:
            with lock:
                queued_keys.discard(key)
            upload_q.task_done()


def do_upload(key, local_target, remote_target) -> int:
    cmd = [
        "rclone", "copy", local_target, remote_target,
        "--stats=2s", "--stats-one-line", "--stats-log-level", "NOTICE",
    ]
    proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1
    )
    for line in proc.stdout:
        info = parse_progress_line(line)
        if info:
            update(key, **info)
    proc.wait()
    return proc.returncode

def parse_progress_line(line: str):
    # matches: "... NOTICE:  60.391 MiB / 10.719 GiB, 1%, 9.241 MiB/s, ETA 19m41s"
    m = re.search(
        r'([\d.]+\s*\S+)\s*/\s*([\d.]+\s*\S+),\s*(\d+)%,\s*([\d.]+\s*\S+/s),\s*ETA\s+(\S+)',
        line
    )
    if m:
        return {"xfr": m.group(1).strip(), "tot": m.group(2).strip(),
                "pct": int(m.group(3)), "speed": m.group(4).strip(),
                "eta": m.group(5)}
    return None


# ── state helpers ─────────────────────────────────────────────────────────────
def set_state(key, state):
    with lock:
        status.setdefault(key, {})["state"] = state

def update(key, **kw):
    with lock:
        status.setdefault(key, {}).update(kw)

def get_status(key):
    with lock:
        return dict(status.get(key, {}))


# ── display ───────────────────────────────────────────────────────────────────
def color(t, c): return f"\033[{c}m{t}\033[0m"
def green(t):  return color(t, "32")
def yellow(t): return color(t, "33")
def cyan(t):   return color(t, "36")
def red(t):    return color(t, "31")
def dim(t):    return color(t, "2")
def bold(t):   return color(t, "1")

def _bar(pct, width=14):
    if pct is None:
        return ""
    filled = int(width * pct / 100)
    return "[" + "█" * filled + "░" * (width - filled) + "]"

def fmt_gb(b):
    if b is None or b < 0:
        return "?"
    return f"{b / (1024**3):.1f}G"

def render(session_map, wifi_ok, iface):
    out = []
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    wifi_str = green(f"WiFi ▲ ({iface})") if wifi_ok else red("WiFi ✗ (offline)")
    out.append(bold(f"  Box Upload Monitor   {wifi_str}   {dim(ts)}"))
    out.append(dim("  " + "─" * 70))

    done_ct = total_ct = 0
    for site, sessions in session_map.items():
        out.append(f"\n  {bold(cyan('data/' + site))}")
        for s in sessions:
            key = f"{site}/{s.name}"
            st  = get_status(key)
            total_ct += 1
            state = st.get("state", "pending")
            lb = fmt_gb(st.get("local_b"))

            if state == "done":
                done_ct += 1
                tag = green(f"✓ done  {dim(lb)}")
            elif state == "checking":
                tag = cyan(f"… verifying {dim(lb)}  (remote {fmt_gb(st.get('remote_b'))})")
            elif state == "uploading":
                pct = st.get("pct")
                bar = _bar(pct)
                if pct is not None:
                    seg = f"{bar} {pct}%"
                    if st.get("xfr") and st.get("tot"):
                        seg += f"  {st['xfr']}/{st['tot']}"
                    if st.get("speed"):
                        seg += f"  {st['speed']}"
                    if st.get("eta"):
                        seg += f"  ETA {st['eta']}"
                    tag = yellow(f"↑ {seg}")
                else:
                    tag = yellow("↑ uploading…")
            elif state == "failed":
                tag = red(f"✗ failed  (local {lb} vs remote {fmt_gb(st.get('remote_b'))}) — will retry")
            elif state == "queued":
                tag = dim(f"○ queued  {lb}")
            else:
                base = "○ pending" if wifi_ok else "○ waiting for WiFi"
                tag = dim(f"{base}  {lb}" if st.get("local_b") else base)

            out.append(f"    └ {s.name:<18} {tag}")

    out.append("")
    out.append(dim(f"  {done_ct}/{total_ct} done   ·   log: {LOG_FILE}"))
    out.append("")
    return "\n".join(out)

def clear_and_print(text):
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.write(text)
    sys.stdout.flush()


# ── main ──────────────────────────────────────────────────────────────────────
def main():
    global WIFI_CHECK_IFACE
    if WIFI_CHECK_IFACE is None:
        WIFI_CHECK_IFACE = get_wifi_iface()

    log("=== monitor started ===")
    threading.Thread(target=upload_worker, daemon=True).start()
    # a couple of check workers so the initial scan finishes fast
    for _ in range(3):
        threading.Thread(target=check_worker, daemon=True).start()

    dispatched_check = {}  # key -> last time we sent it to check_q

    while True:
        iface   = WIFI_CHECK_IFACE or get_wifi_iface()
        wifi_ok = is_wifi_up(iface)
        session_map = get_session_folders()

        if wifi_ok:
            for site, sessions in session_map.items():
                for s in sessions:
                    key = f"{site}/{s.name}"
                    st  = get_status(key)
                    state = st.get("state", "pending")
                    checked_at = st.get("checked_at", 0)

                    with lock:
                        in_upload_q  = key in queued_keys
                        in_check_q   = key in checking_keys

                    # don't re-dispatch anything already moving through the pipeline
                    if (state in ("checking", "uploading", "queued", "done")
                            or in_upload_q or in_check_q):
                        continue

                    last_dispatch = dispatched_check.get(key, 0)
                    send_check = False
                    if state == "pending":
                        send_check = True
                    elif state == "failed" and (time.time() - last_dispatch) > 30:
                        send_check = True

                    if send_check:
                        dispatched_check[key] = time.time()
                        with lock:
                            checking_keys.add(key)
                        check_q.put((site, s))
        else:
            with lock:
                for key, st in status.items():
                    if st.get("state") in ("queued", "checking"):
                        st["state"] = "pending"
                checking_keys.clear()

        display = render(session_map, wifi_ok, iface)
        clear_and_print(display)
        time.sleep(REFRESH_SEC)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n  Stopped.")