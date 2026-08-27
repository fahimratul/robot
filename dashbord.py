"""
Line Follower Robot - Laptop Dashboard (with RPLidar C1 obstacle detection)
----------------------------------------------------------------------------
Two independent serial links:

  1) Teensy (robot control) - plain text commands, newline terminated:
        TABLE1   -> select Table 1 (robot turns LEFT at the next intersection)
        TABLE2   -> select Table 2 (robot turns RIGHT at the next intersection)
        START    -> begin running (only allowed after a table is selected)
        STOP     -> stop immediately
     The dashboard sends STOP/START itself (in addition to the buttons)
     whenever the LiDAR sees/clears an obstacle in front of the robot.

  2) RPLidar C1 (obstacle detection + live map) - talked to directly from
     the laptop over its own USB serial port, using the 'rplidarc1' library.
     Only points inside the FRONT 180 degrees (-90..+90 around heading 0)
     are considered for obstacle detection; the full 360 degree scan is
     still drawn on the map.

Requires:
    pip install pyserial rplidarc1 pyttsx3
    (rplidarc1 needs Python 3.10+ for asyncio.TaskGroup)
    (pyttsx3 is used for the "Please give side" voice alert; if it isn't
     installed the dashboard still runs, it just skips the voice alert)

Run:
    python robot_dashboard.py

NOTE: the exact import path for the RPLidar class depends on how the
'rplidarc1' package you installed is laid out. If the import below fails,
run:  python -c "import rplidarc1, pkgutil; print(pkgutil.walk_packages(rplidarc1.__path__))"
and adjust the "from ... import RPLidar" line to match.
"""

import asyncio
import http.server
import json
import math
import queue
import socket
import socketserver
import threading
import time
import tkinter as tk
from tkinter import ttk
from urllib.parse import urlparse

import serial
import serial.tools.list_ports

try:
    from rplidarc1.scanner import RPLidar as C1Lidar
except ImportError:
    try:
        from rplidarc1 import RPLidar as C1Lidar
    except ImportError:
        C1Lidar = None  # handled at connect-time with a friendly error

try:
    import pyttsx3
except ImportError:
    pyttsx3 = None  # voice alerts are skipped if this isn't installed

OBSTACLE_VOICE_MSG = "Please give side"

LIDAR_BAUDRATE = 460800
MIN_QUALITY = 5            # ignore very low-confidence points
MAP_MAX_RANGE_MM = 4000    # canvas display range (outer ring)
FRONT_HALF_ANGLE = 90      # "front 180" = heading +/- 90 degrees

PHONE_SERVER_PORT = 8765   # phone remote control - browse to http://<laptop-lan-ip>:8765
STALL_ALERT_SECONDS = 10   # alert the phone if stopped this long (line lost or obstacle)
MANUAL_COMMANDS = {"MFWD", "MBACK", "MLEFT", "MRIGHT", "MSTOP"}

# =====================================================================
# "HUD" theme - dark sci-fi palette used throughout the UI
# =====================================================================
BG_MAIN = "#050810"
BG_PANEL = "#0b1220"
BG_PANEL_ALT = "#101a2e"
BG_INSET = "#020508"
FG_TEXT = "#c9f4ff"
FG_DIM = "#5b7a94"
ACCENT = "#00e5ff"       # primary cyan
ACCENT_DIM = "#0a5f70"
ACCENT2 = "#ff2ea6"      # magenta highlight
SUCCESS = "#00ffa3"
DANGER = "#ff3b5c"
WARNING = "#ffb800"

FONT_MONO = ("Consolas", 10)
FONT_MONO_BOLD = ("Consolas", 10, "bold")
FONT_HEADER = ("Consolas", 15, "bold")
FONT_STATUS = ("Consolas", 10, "bold")
FONT_BTN = ("Consolas", 10, "bold")

# direction key -> (glyph, color, label) - used to visualize robot heading
DIRECTION_STYLES = {
    "left":    ("◀", ACCENT2, "TURN LEFT"),
    "right":   ("▶", ACCENT2, "TURN RIGHT"),
    "forward": ("▲", SUCCESS, "FORWARD"),
    "back":    ("▼", WARNING, "REVERSE"),
    "stop":    ("■", DANGER, "STOPPED"),
}


def angle_in_front(angle_deg):
    """True if angle_deg (0-360, 0 = straight ahead) is within the front 180."""
    a = angle_deg % 360
    return a <= FRONT_HALF_ANGLE or a >= (360 - FRONT_HALF_ANGLE)


# =====================================================================
# LiDAR background worker (runs its own asyncio loop in a separate thread)
# =====================================================================
class LidarWorker:
    def __init__(self, port, on_point, on_status):
        self.port = port
        self.on_point = on_point       # callback(angle_deg, distance_mm, quality)
        self.on_status = on_status     # callback(str) - thread-safe logging
        self.lidar = None
        self.thread = None
        self._stop_flag = threading.Event()

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            asyncio.run(self._main())
        except Exception as e:
            self.on_status(f"LiDAR worker ended: {e}")

    async def _main(self):
        if C1Lidar is None:
            self.on_status("ERROR: rplidarc1 package not found (pip install rplidarc1)")
            return

        self.lidar = C1Lidar(self.port, LIDAR_BAUDRATE)
        self.on_status(f"LiDAR connecting on {self.port} @ {LIDAR_BAUDRATE}...")

        consumer = asyncio.create_task(self._consume())
        try:
            await self.lidar.simple_scan()
        except Exception as e:
            self.on_status(f"LiDAR scan stopped: {e}")
        finally:
            self._stop_flag.set()
            await asyncio.gather(consumer, return_exceptions=True)

    async def _consume(self):
        self.on_status("LiDAR streaming scan data.")
        while not self._stop_flag.is_set():
            try:
                data = await asyncio.wait_for(self.lidar.output_queue.get(), timeout=0.5)
            except asyncio.TimeoutError:
                continue
            except Exception:
                break
            self.on_point(data["a_deg"], data["d_mm"], data["q"])

    def stop(self):
        self._stop_flag.set()
        if self.lidar:
            try:
                self.lidar.shutdown()
            except Exception:
                pass


# =====================================================================
# Voice alert worker (runs pyttsx3 on its own thread so speech never
# blocks the Tk UI thread; a queue serializes announcements)
# =====================================================================
class SpeechWorker:
    def __init__(self, on_status=None):
        self.on_status = on_status
        self._queue = queue.Queue()
        self._engine = None
        if pyttsx3 is not None:
            threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            self._engine = pyttsx3.init()
            self._engine.setProperty("rate", 170)
        except Exception as e:
            if self.on_status:
                self.on_status(f"Voice engine failed to start: {e}")
            return
        while True:
            text = self._queue.get()
            try:
                self._engine.say(text)
                self._engine.runAndWait()
            except Exception:
                pass

    def speak(self, text):
        if pyttsx3 is None:
            return
        self._queue.put(text)


# =====================================================================
# Phone remote control (LAN web page - select table / start / stop)
# =====================================================================
def get_lan_ip():
    """Best-effort LAN IP of this machine (no packets actually sent)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


PHONE_PAGE_HTML = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Remote</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  body { background:#050810; color:#c9f4ff; font-family: Consolas, monospace; margin:0; padding:16px; }
  h1 { color:#00e5ff; font-size:1.1rem; text-align:center; margin:0 0 12px; }
  .status { text-align:center; margin-bottom:16px; font-size:0.95rem; color:#5b7a94; }
  .status b { color:#00e5ff; }
  .grid { display:grid; grid-template-columns:1fr 1fr; gap:12px; }
  button { padding:26px 8px; font-size:1.15rem; font-weight:bold; border-radius:10px;
           border:1px solid #0a5f70; background:#101a2e; color:#00e5ff; touch-action:none; }
  button:active { background:#0a5f70; }
  button.start { color:#00ffa3; border-color:#00ffa3; }
  button.stop { color:#ff3b5c; border-color:#ff3b5c; }
  button.selected { background:#ff2ea6; color:#020508; }
  .obstacle { text-align:center; margin-top:16px; font-weight:bold; min-height:1.2em; }
  .alertbar { display:none; text-align:center; font-weight:bold; padding:12px; border-radius:10px;
              margin-bottom:14px; background:#3a0a14; color:#ff3b5c; border:1px solid #ff3b5c; }
  body.alerting .alertbar { animation: flash 0.6s step-start infinite; }
  @keyframes flash { 50% { background:#ff3b5c; color:#020508; } }
  .manual-wrap { margin-top:20px; text-align:center; }
  #manualBtn { width:100%; border-color:#ffb800; color:#ffb800; }
  .dpad { display:none; grid-template-columns: 1fr 1fr 1fr; gap:8px; margin-top:14px; }
  .dbtn { font-size:1.4rem; padding:20px 0; user-select:none; }
  .dbtn.stopbtn { color:#ff3b5c; border-color:#ff3b5c; }
  .hint { display:none; margin-top:10px; font-size:0.8rem; color:#5b7a94; }
</style>
</head>
<body>
  <h1>&#9670; ROBOT REMOTE &#9670;</h1>
  <div class="alertbar" id="alertbar"></div>
  <div class="status" id="status">connecting...</div>
  <div class="grid">
    <button id="t1" onclick="post('/api/table1')">TABLE 1<br>(LEFT)</button>
    <button id="t2" onclick="post('/api/table2')">TABLE 2<br>(RIGHT)</button>
    <button class="start" onclick="post('/api/start')">&#9654; START</button>
    <button class="stop" onclick="post('/api/stop')">&#9632; STOP</button>
  </div>
  <div class="obstacle" id="obstacle"></div>

  <div class="manual-wrap">
    <button id="manualBtn" onclick="post('/api/manual/on')">&#9998; TAKE MANUAL CONTROL</button>
    <div class="dpad" id="dpad">
      <div></div><button class="dbtn" data-cmd="fwd">&#9650;</button><div></div>
      <button class="dbtn" data-cmd="left">&#9664;</button>
      <button class="dbtn stopbtn" data-cmd="stop">&#9632;</button>
      <button class="dbtn" data-cmd="right">&#9654;</button>
      <div></div><button class="dbtn" data-cmd="back">&#9660;</button><div></div>
    </div>
    <div class="hint" id="manualHint">Manual control active. Steer around the obstacle / back onto
      the line, then press &#9654; START above to resume line-following.</div>
  </div>

<script>
function post(path) {
  fetch(path, {method:'POST'}).catch(function(){});
}

let dpadInterval = null;
function startMove(cmd) {
  post('/api/manual/' + cmd);
  if (dpadInterval) clearInterval(dpadInterval);
  dpadInterval = setInterval(function(){ post('/api/manual/' + cmd); }, 150);
}
function stopMove() {
  if (dpadInterval) { clearInterval(dpadInterval); dpadInterval = null; }
  post('/api/manual/stop');
}
document.querySelectorAll('.dbtn').forEach(function(btn) {
  const cmd = btn.dataset.cmd;
  if (cmd === 'stop') {
    btn.addEventListener('pointerdown', function(e) { e.preventDefault(); post('/api/manual/stop'); });
    return;
  }
  btn.addEventListener('pointerdown', function(e) { e.preventDefault(); startMove(cmd); });
  ['pointerup', 'pointerleave', 'pointercancel'].forEach(function(evt) {
    btn.addEventListener(evt, stopMove);
  });
});

let lastAlert = false;
function beep() {
  try {
    const ctx = new (window.AudioContext || window.webkitAudioContext)();
    const o = ctx.createOscillator();
    const g = ctx.createGain();
    o.connect(g); g.connect(ctx.destination);
    o.frequency.value = 880;
    g.gain.value = 0.25;
    o.start();
    setTimeout(function(){ o.stop(); ctx.close(); }, 400);
  } catch (e) {}
}
function triggerAlert() {
  document.body.classList.add('alerting');
  if (navigator.vibrate) navigator.vibrate([300, 150, 300, 150, 300]);
  beep();
}

async function poll() {
  try {
    const r = await fetch('/api/status');
    const s = await r.json();
    const table = s.table ? ('TABLE ' + s.table) : 'NONE';
    let state = s.running ? 'RUNNING' : 'STOPPED';
    if (s.manual_mode) state = 'MANUAL CONTROL';
    else if (s.auto_paused) state = 'WAITING (OBSTACLE)';
    document.getElementById('status').innerHTML =
      (s.connected ? '<b>CONNECTED</b>' : '<span style="color:#ff3b5c">DISCONNECTED</span>')
      + ' &nbsp; TABLE: <b>' + table + '</b> &nbsp; STATE: <b>' + state + '</b>';
    document.getElementById('t1').className = s.table === 1 ? 'selected' : '';
    document.getElementById('t2').className = s.table === 2 ? 'selected' : '';
    const obEl = document.getElementById('obstacle');
    obEl.textContent = s.obstacle ? '\\u26A0 OBSTACLE DETECTED' : '';
    obEl.style.color = s.obstacle ? '#ff3b5c' : '#00ffa3';

    document.getElementById('manualBtn').style.display = s.manual_mode ? 'none' : 'block';
    document.getElementById('dpad').style.display = s.manual_mode ? 'grid' : 'none';
    document.getElementById('manualHint').style.display = s.manual_mode ? 'block' : 'none';

    const bar = document.getElementById('alertbar');
    if (s.alert) {
      const reason = s.alert_reason === 'line_lost' ? 'LOST THE LINE' : 'BLOCKED BY OBSTACLE';
      const secs = s.stalled_seconds ? ' (' + Math.round(s.stalled_seconds) + 's)' : '';
      bar.textContent = '\\u26A0 ROBOT NEEDS HELP \\u2014 ' + reason + secs;
      bar.style.display = 'block';
      if (!lastAlert) triggerAlert();
    } else {
      bar.style.display = 'none';
      document.body.classList.remove('alerting');
    }
    lastAlert = s.alert;
  } catch (e) {
    document.getElementById('status').textContent = 'connection lost...';
  }
}
setInterval(poll, 1000);
poll();
</script>
</body>
</html>
"""


class PhoneRequestHandler(http.server.BaseHTTPRequestHandler):
    dashboard = None  # bound per-server via a subclass, see PhoneControlServer

    def log_message(self, format, *args):
        pass  # keep the terminal quiet; UI log already shows activity

    def _send_json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_html(self, html):
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        d = self.dashboard
        if path == "/":
            self._send_html(PHONE_PAGE_HTML)
        elif path == "/api/status":
            now = time.time()
            stalled_secs = None
            if d.line_lost_since is not None:
                stalled_secs = now - d.line_lost_since
            elif d.obstacle_pause_since is not None:
                stalled_secs = now - d.obstacle_pause_since
            self._send_json({
                "connected": bool(d.ser and d.ser.is_open),
                "table": d.selected_table,
                "running": d.is_running,
                "auto_paused": d.auto_paused,
                "obstacle": d.obstacle_active,
                "manual_mode": d.manual_mode,
                "alert": d.alert_active,
                "alert_reason": d.alert_reason,
                "stalled_seconds": None if stalled_secs is None else round(stalled_secs, 1),
            })
        else:
            self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        d = self.dashboard
        actions = {
            "/api/table1": lambda: d._select_table(1),
            "/api/table2": lambda: d._select_table(2),
            "/api/start": d._send_start,
            "/api/stop": d._send_stop,
            "/api/manual/on": d._enter_manual,
            "/api/manual/fwd": lambda: d._manual_move("MFWD"),
            "/api/manual/back": lambda: d._manual_move("MBACK"),
            "/api/manual/left": lambda: d._manual_move("MLEFT"),
            "/api/manual/right": lambda: d._manual_move("MRIGHT"),
            "/api/manual/stop": lambda: d._manual_move("MSTOP"),
        }
        action = actions.get(path)
        if action is None:
            self.send_error(404)
            return
        # Hop onto the Tk main thread - Tkinter/pyserial calls aren't safe
        # to make directly from this HTTP handler's own thread.
        d.root.after(0, action)
        self._send_json({"ok": True})


class PhoneControlServer(threading.Thread):
    """Serves the phone remote page + API on the LAN, in a background thread."""

    def __init__(self, dashboard, port):
        super().__init__(daemon=True)
        handler = type("BoundPhoneHandler", (PhoneRequestHandler,), {"dashboard": dashboard})
        self.httpd = socketserver.ThreadingTCPServer(("0.0.0.0", port), handler)
        self.httpd.daemon_threads = True

    def run(self):
        try:
            self.httpd.serve_forever()
        except Exception:
            pass

    def stop(self):
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass


# =====================================================================
# Main dashboard
# =====================================================================
class RobotDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("◈ ROBOT CONTROL SYSTEM — LIDAR HUD ◈")
        self.root.resizable(True, True)
        self.root.configure(bg=BG_MAIN)
        try:
            self.root.state("zoomed")  # start maximized (Windows/most Linux WMs)
        except tk.TclError:
            self.root.attributes("-zoomed", True)

        # ---- Robot serial state ----
        self.ser = None
        self.reader_running = False
        self.selected_table = None
        self.is_running = False
        self.manual_mode = False       # True after MANUAL takeover (phone or desktop)
        self._manual_repeat_job = None  # after() handle for desktop press-and-hold

        # ---- LiDAR state ----
        self.lidar_worker = None
        self.scan_points = {}          # {angle_deg_int: (distance_mm, quality)}
        self.scan_lock = threading.Lock()
        self.obstacle_active = False
        self.auto_paused = False       # True if WE stopped the robot for an obstacle
        self.obstacle_threshold_mm = tk.IntVar(value=400)
        self.current_direction = None   # one of DIRECTION_STYLES keys, or None

        # ---- Stall / alert tracking (line lost or blocked >10s -> alert phone) ----
        self.line_lost_since = None
        self.obstacle_pause_since = None
        self.alert_active = False
        self.alert_reason = None       # "line_lost" or "obstacle"

        self.speech = SpeechWorker(on_status=lambda msg: self.root.after(0, self._log, msg))
        if pyttsx3 is None:
            self.root.after(0, self._log,
                             "Voice alerts disabled (pip install pyttsx3 to enable).")

        # ---- Phone remote control (LAN) ----
        self.phone_server = None

        self._setup_style()
        self._build_ui()
        self._refresh_ports()
        self._draw_direction(None)
        self._refresh_map()  # start the periodic canvas/obstacle-check loop
        self._start_phone_server()

    def _start_phone_server(self):
        try:
            self.phone_server = PhoneControlServer(self, PHONE_SERVER_PORT)
            self.phone_server.start()
            url = f"http://{get_lan_ip()}:{PHONE_SERVER_PORT}"
            self.phone_url_label.config(text=f"Phone remote: {url}")
            self._log(f"Phone control server running at {url}  (same Wi-Fi as this laptop)")
        except Exception as e:
            self.phone_url_label.config(text="Phone remote: unavailable")
            self._log(f"Phone control server failed to start: {e}")

    # ---------------- HUD styling ----------------
    def _setup_style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure("TFrame", background=BG_MAIN)
        style.configure("HUD.TFrame", background=BG_MAIN)
        style.configure("Panel.TFrame", background=BG_PANEL)

        style.configure("TLabelframe", background=BG_PANEL, bordercolor=ACCENT_DIM,
                         borderwidth=1, relief="solid")
        style.configure("TLabelframe.Label", background=BG_PANEL, foreground=ACCENT,
                         font=FONT_MONO_BOLD)

        style.configure("TLabel", background=BG_PANEL, foreground=FG_TEXT, font=FONT_MONO)
        style.configure("Header.TLabel", background=BG_MAIN, foreground=ACCENT, font=FONT_HEADER)
        style.configure("SubHeader.TLabel", background=BG_MAIN, foreground=FG_DIM,
                         font=("Consolas", 9))

        style.configure("TButton", background=BG_PANEL_ALT, foreground=ACCENT,
                         font=FONT_BTN, borderwidth=1, focuscolor=BG_PANEL_ALT)
        style.map("TButton",
                  background=[("active", ACCENT_DIM), ("pressed", ACCENT_DIM)],
                  foreground=[("disabled", FG_DIM)])

        style.configure("TCombobox", fieldbackground=BG_PANEL_ALT, background=BG_PANEL_ALT,
                         foreground=FG_TEXT, arrowcolor=ACCENT, bordercolor=ACCENT_DIM,
                         insertcolor=ACCENT)
        style.map("TCombobox", fieldbackground=[("readonly", BG_PANEL_ALT)],
                   foreground=[("readonly", FG_TEXT)])

        style.configure("TSpinbox", fieldbackground=BG_PANEL_ALT, background=BG_PANEL_ALT,
                         foreground=FG_TEXT, arrowcolor=ACCENT, bordercolor=ACCENT_DIM,
                         insertcolor=ACCENT)

        # Make the dropdown listboxes match the dark theme too
        self.root.option_add("*TCombobox*Listbox.background", BG_PANEL_ALT)
        self.root.option_add("*TCombobox*Listbox.foreground", FG_TEXT)
        self.root.option_add("*TCombobox*Listbox.selectBackground", ACCENT_DIM)
        self.root.option_add("*TCombobox*Listbox.font", FONT_MONO)

    def _neon_button(self, parent, **kwargs):
        defaults = dict(
            font=FONT_BTN, bg=BG_PANEL_ALT, fg=ACCENT, activebackground=ACCENT_DIM,
            activeforeground=ACCENT, disabledforeground=FG_DIM, relief="flat",
            highlightthickness=1, highlightbackground=ACCENT_DIM, highlightcolor=ACCENT,
            bd=0, cursor="hand2",
        )
        defaults.update(kwargs)
        return tk.Button(parent, **defaults)

    # ---------------- UI ----------------
    def _build_ui(self):
        header = ttk.Label(self.root, text="◈ MIST CAFE BOT ◈",
                            style="Header.TLabel", anchor="center")
        header.pack(fill="x", pady=(12, 0))
        sub = ttk.Label(self.root, text="TEENSY LINK · RPLIDAR C1 · REAL-TIME OBSTACLE FIELD",
                         style="SubHeader.TLabel", anchor="center")
        sub.pack(fill="x", pady=(0, 8))

        body = ttk.Frame(self.root)
        body.pack(fill="both", expand=True)

        left = ttk.Frame(body)
        left.pack(side="left", fill="y", padx=10, pady=10)

        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True, padx=10, pady=10)

        # ===== LEFT: Robot control =====
        conn_frame = ttk.LabelFrame(left, text="◆ ROBOT LINK (TEENSY)")
        conn_frame.pack(fill="x", pady=(0, 8))

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=16, state="readonly")
        self.port_combo.grid(row=0, column=0, padx=6, pady=6)
        ttk.Button(conn_frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=1, padx=4)
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connect)
        self.connect_btn.grid(row=1, column=0, padx=6, pady=(0, 6))
        self.conn_status = ttk.Label(conn_frame, text="● DISCONNECTED", foreground=DANGER,
                                      font=FONT_STATUS)
        self.conn_status.grid(row=1, column=1, padx=6)

        self.phone_url_label = ttk.Label(conn_frame, text="Phone remote: starting...",
                                          font=("Consolas", 8), foreground=FG_DIM,
                                          wraplength=220)
        self.phone_url_label.grid(row=2, column=0, columnspan=2, padx=6, pady=(0, 6), sticky="w")

        table_frame = ttk.LabelFrame(left, text="◆ SELECT TABLE (BEFORE START)")
        table_frame.pack(fill="x", pady=8)
        self.table1_btn = self._neon_button(table_frame, text="TABLE 1\n(turn LEFT)", width=14, height=3,
                                             command=lambda: self._select_table(1))
        self.table1_btn.grid(row=0, column=0, padx=8, pady=8)
        self.table2_btn = self._neon_button(table_frame, text="TABLE 2\n(turn RIGHT)", width=14, height=3,
                                             command=lambda: self._select_table(2))
        self.table2_btn.grid(row=0, column=1, padx=8, pady=8)

        control_frame = ttk.LabelFrame(left, text="◆ CONTROL")
        control_frame.pack(fill="x", pady=8)
        self.start_btn = self._neon_button(control_frame, text="▶ START", width=14, height=3,
                                            fg=SUCCESS, highlightbackground=SUCCESS,
                                            activebackground="#0a3324", state="disabled",
                                            command=self._send_start)
        self.start_btn.grid(row=0, column=0, padx=8, pady=8)
        self.stop_btn = self._neon_button(control_frame, text="■ STOP", width=14, height=3,
                                           fg=DANGER, highlightbackground=DANGER,
                                           activebackground="#3a0a14", state="disabled",
                                           command=self._send_stop)
        self.stop_btn.grid(row=0, column=1, padx=8, pady=8)

        self.status_label = ttk.Label(left, text="TABLE: NONE  |  STATE: STOPPED",
                                       font=FONT_STATUS, foreground=ACCENT)
        self.status_label.pack(fill="x", pady=(4, 4))

        self.alert_label = ttk.Label(left, text="", font=FONT_STATUS, foreground=DANGER,
                                      wraplength=260)
        self.alert_label.pack(fill="x", pady=(0, 8))

        manual_frame = ttk.LabelFrame(left, text="◆ MANUAL DRIVE (TAKEOVER, e.g. lost line/obstacle)")
        manual_frame.pack(fill="x", pady=8)
        self.manual_btn = self._neon_button(
            manual_frame, text="✎ TAKE MANUAL CONTROL", width=28,
            fg=WARNING, highlightbackground=WARNING, command=self._enter_manual)
        self.manual_btn.grid(row=0, column=0, columnspan=3, padx=8, pady=(8, 4), sticky="ew")

        self.mfwd_btn = self._neon_button(manual_frame, text="▲", width=5, state="disabled")
        self.mfwd_btn.grid(row=1, column=1, padx=4, pady=2)
        self.mfwd_btn.bind("<ButtonPress-1>", lambda e: self._manual_press("MFWD"))
        self.mfwd_btn.bind("<ButtonRelease-1>", lambda e: self._manual_release())

        self.mleft_btn = self._neon_button(manual_frame, text="◀", width=5, state="disabled")
        self.mleft_btn.grid(row=2, column=0, padx=4, pady=2)
        self.mleft_btn.bind("<ButtonPress-1>", lambda e: self._manual_press("MLEFT"))
        self.mleft_btn.bind("<ButtonRelease-1>", lambda e: self._manual_release())

        self.mstop_btn = self._neon_button(manual_frame, text="■", width=5, state="disabled",
                                            fg=DANGER, highlightbackground=DANGER,
                                            command=lambda: self._manual_move("MSTOP"))
        self.mstop_btn.grid(row=2, column=1, padx=4, pady=2)

        self.mright_btn = self._neon_button(manual_frame, text="▶", width=5, state="disabled")
        self.mright_btn.grid(row=2, column=2, padx=4, pady=2)
        self.mright_btn.bind("<ButtonPress-1>", lambda e: self._manual_press("MRIGHT"))
        self.mright_btn.bind("<ButtonRelease-1>", lambda e: self._manual_release())

        self.mback_btn = self._neon_button(manual_frame, text="▼", width=5, state="disabled")
        self.mback_btn.grid(row=3, column=1, padx=4, pady=(2, 8))
        self.mback_btn.bind("<ButtonPress-1>", lambda e: self._manual_press("MBACK"))
        self.mback_btn.bind("<ButtonRelease-1>", lambda e: self._manual_release())

        log_frame = ttk.LabelFrame(left, text="◆ ROBOT LOG")
        log_frame.pack(fill="both", expand=True)

        dir_row = ttk.Frame(log_frame, style="Panel.TFrame")
        dir_row.pack(fill="x", padx=4, pady=(6, 2))
        self.dir_canvas = tk.Canvas(dir_row, width=50, height=50, bg=BG_INSET,
                                     highlightthickness=1, highlightbackground=ACCENT_DIM)
        self.dir_canvas.pack(side="left")
        self.dir_label = ttk.Label(dir_row, text="—  IDLE", font=("Consolas", 12, "bold"),
                                    foreground=FG_DIM, background=BG_PANEL)
        self.dir_label.pack(side="left", padx=10)

        self.log_text = tk.Text(log_frame, height=16, width=42, state="disabled", wrap="word",
                                 bg=BG_INSET, fg=SUCCESS, insertbackground=ACCENT,
                                 font=("Consolas", 9), relief="flat", highlightthickness=1,
                                 highlightbackground=ACCENT_DIM, highlightcolor=ACCENT,
                                 padx=6, pady=4)
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

        # ===== RIGHT: LiDAR =====
        lidar_conn_frame = ttk.LabelFrame(right, text="◆ LIDAR LINK (RPLIDAR C1)")
        lidar_conn_frame.pack(fill="x")

        self.lidar_port_var = tk.StringVar()
        self.lidar_port_combo = ttk.Combobox(lidar_conn_frame, textvariable=self.lidar_port_var,
                                              width=16, state="readonly")
        self.lidar_port_combo.grid(row=0, column=0, padx=6, pady=6)
        ttk.Button(lidar_conn_frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=1, padx=4)
        self.lidar_connect_btn = ttk.Button(lidar_conn_frame, text="Connect LiDAR",
                                             command=self._toggle_lidar_connect)
        self.lidar_connect_btn.grid(row=0, column=2, padx=6)

        ttk.Label(lidar_conn_frame, text="Obstacle distance (mm):").grid(row=0, column=3, padx=(16, 4))
        ttk.Spinbox(lidar_conn_frame, from_=100, to=4000, increment=50, width=6,
                    textvariable=self.obstacle_threshold_mm).grid(row=0, column=4, padx=4)

        self.lidar_status_label = ttk.Label(right, text="● LIDAR: DISCONNECTED", foreground=DANGER,
                                             font=FONT_STATUS, background=BG_MAIN)
        self.lidar_status_label.pack(anchor="w", pady=(6, 0))

        self.obstacle_label = ttk.Label(right, text="✓ PATH CLEAR", foreground=SUCCESS,
                                         font=("Consolas", 13, "bold"), background=BG_MAIN)
        self.obstacle_label.pack(anchor="w", pady=(0, 6))

        map_frame = ttk.LabelFrame(right, text="◆ LIDAR MAP — FRONT 180° OBSTACLE FIELD (SHADED)")
        map_frame.pack(fill="both", expand=True)
        self.canvas_size = 560  # updated live to match the map_frame's actual size
        self.canvas = tk.Canvas(map_frame, width=self.canvas_size, height=self.canvas_size,
                                 bg=BG_INSET, highlightthickness=1, highlightbackground=ACCENT_DIM)
        self.canvas.pack(fill="both", expand=True, padx=8, pady=8)
        self.canvas.bind("<Configure>", self._on_map_canvas_resize)

    def _on_map_canvas_resize(self, event):
        # Keep the radar circle square: use the smaller of width/height.
        self.canvas_size = max(100, min(event.width, event.height))

    # ---------------- Port list ----------------
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        self.lidar_port_combo["values"] = ports
        if ports:
            if not self.port_var.get():
                self.port_var.set(ports[0])
            if not self.lidar_port_var.get():
                self.lidar_port_var.set(ports[-1])

    # ---------------- Robot serial ----------------
    def _toggle_connect(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            self._log("No robot port selected.")
            return
        try:
            self.ser = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)  # allow Teensy to reset/boot
            self.reader_running = True
            threading.Thread(target=self._read_loop, daemon=True).start()

            self.conn_status.config(text=f"● CONNECTED ({port})", foreground=SUCCESS)
            self.connect_btn.config(text="Disconnect")
            self.stop_btn.config(state="normal")
            self._log(f"Connected to robot on {port}")
        except Exception as e:
            self._log(f"Connection failed: {e}")

    def _disconnect(self):
        self.reader_running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.conn_status.config(text="● DISCONNECTED", foreground=DANGER)
        self.connect_btn.config(text="Connect")
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="disabled")
        self._log("Disconnected from robot.")

    def _read_loop(self):
        while self.reader_running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line:
                    self.root.after(0, self._on_robot_line, line)
            except Exception:
                break

    def _on_robot_line(self, line):
        # Track how long the robot has been sitting lost, so we can alert
        # the phone if it stays that way (see _check_stall_alert).
        if line == "LINE_LOST":
            if self.line_lost_since is None:
                self.line_lost_since = time.time()
        elif line in ("FORWARD", "LEFT", "RIGHT") or line.startswith("INTERSECTION"):
            self.line_lost_since = None
        self._log(f"Robot: {line}")

    def _select_table(self, table_num):
        if not (self.ser and self.ser.is_open):
            self._log("Connect to the robot first.")
            return
        self.selected_table = table_num
        self._send("TABLE1" if table_num == 1 else "TABLE2")

        self.table1_btn.config(bg=ACCENT2 if table_num == 1 else BG_PANEL_ALT,
                                fg=BG_INSET if table_num == 1 else ACCENT)
        self.table2_btn.config(bg=ACCENT2 if table_num == 2 else BG_PANEL_ALT,
                                fg=BG_INSET if table_num == 2 else ACCENT)
        self.start_btn.config(state="normal")
        self._update_status()

    def _send_start(self):
        self._send("START")
        self.is_running = True
        self.auto_paused = False
        self.manual_mode = False
        self.line_lost_since = None
        self.obstacle_pause_since = None
        self._clear_alert()
        self._update_status()

    def _send_stop(self):
        self._send("STOP")
        self.is_running = False
        self.auto_paused = False
        self.manual_mode = False
        self._update_status()

    def _send(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + "\n").encode())
                self._log(f"Sent: {cmd}")
            except Exception as e:
                self._log(f"Send failed: {e}")
        else:
            self._log("Not connected.")

    # ---------------- Manual drive takeover (phone or desktop) ----------------
    def _enter_manual(self):
        if not (self.ser and self.ser.is_open):
            self._log("Connect to the robot first.")
            return
        self._send("MANUAL")
        self.manual_mode = True
        self.is_running = False
        self.auto_paused = False
        self.line_lost_since = None
        self.obstacle_pause_since = None
        self._clear_alert()
        self._update_status()
        for btn in (self.mfwd_btn, self.mback_btn, self.mleft_btn, self.mright_btn, self.mstop_btn):
            btn.config(state="normal")

    def _manual_move(self, cmd):
        if not self.manual_mode:
            self._log("Enable manual control first.")
            return
        self._send(cmd)

    def _manual_press(self, cmd):
        """Desktop press-and-hold: re-sends cmd every 150ms while held, mirroring
        the phone joystick, well inside the firmware's 400ms dead-man's-switch."""
        if not self.manual_mode:
            return
        self._send(cmd)
        self._manual_repeat_job = self.root.after(150, lambda: self._manual_press(cmd))

    def _manual_release(self):
        if self._manual_repeat_job is not None:
            self.root.after_cancel(self._manual_repeat_job)
            self._manual_repeat_job = None
        if self.manual_mode:
            self._send("MSTOP")

    def _update_status(self):
        table_txt = "NONE" if self.selected_table is None else f"TABLE {self.selected_table}"
        if self.manual_mode:
            state_txt, color = "MANUAL CONTROL", ACCENT2
        elif self.auto_paused:
            state_txt, color = "WAITING (OBSTACLE)", WARNING
        else:
            state_txt = "RUNNING" if self.is_running else "STOPPED"
            color = SUCCESS if self.is_running else ACCENT
        self.status_label.config(text=f"TABLE: {table_txt}  |  STATE: {state_txt}", foreground=color)
        if not self.manual_mode:
            for btn in (self.mfwd_btn, self.mback_btn, self.mleft_btn, self.mright_btn, self.mstop_btn):
                btn.config(state="disabled")

    # ---------------- Stall alert (line lost or obstacle-blocked >10s) ----------------
    def _check_stall_alert(self):
        now = time.time()
        reason = None
        if self.line_lost_since is not None and (now - self.line_lost_since) > STALL_ALERT_SECONDS:
            reason = "line_lost"
        elif self.obstacle_pause_since is not None and (now - self.obstacle_pause_since) > STALL_ALERT_SECONDS:
            reason = "obstacle"

        if reason and not self.alert_active:
            self.alert_active = True
            self.alert_reason = reason
            desc = "lost the line" if reason == "line_lost" else "blocked by an obstacle"
            self._log(f"⚠ ALERT: robot has been stalled ({desc}) for over "
                      f"{STALL_ALERT_SECONDS}s — take manual control from your phone.")
            self.speech.speak("Robot needs help, please take control")
            tag = "LINE LOST" if reason == "line_lost" else "BLOCKED"
            self.alert_label.config(text=f"⚠ NEEDS HELP: {tag} — USE PHONE/MANUAL", foreground=DANGER)
        elif not reason and self.alert_active:
            self._clear_alert()

    def _clear_alert(self):
        if self.alert_active:
            self._log("Alert cleared.")
        self.alert_active = False
        self.alert_reason = None
        self.alert_label.config(text="")

    def _detect_direction(self, msg):
        """Infer a heading from a log line (our own 'Sent: ...' commands, or
        anything the robot echoes back that mentions a direction word)."""
        m = msg.upper()
        if "TABLE1" in m or "LEFT" in m:
            return "left"
        if "TABLE2" in m or "RIGHT" in m:
            return "right"
        if "STOP" in m:
            return "stop"
        if "BACK" in m or "REVERSE" in m:
            return "back"
        if "START" in m or "FORWARD" in m or "FWD" in m:
            return "forward"
        return None

    def _log(self, msg):
        direction = self._detect_direction(msg)
        self.log_text.config(state="normal")
        if direction:
            glyph, color, _label = DIRECTION_STYLES[direction]
            tag = f"dir_{direction}"
            self.log_text.tag_configure(tag, foreground=color, font=FONT_MONO_BOLD)
            self.log_text.insert("end", f"{glyph} ", tag)
            self.log_text.insert("end", msg + "\n")
            self._set_direction(direction)
        else:
            self.log_text.insert("end", msg + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    # ---------------- Direction indicator ----------------
    def _set_direction(self, direction):
        if direction == self.current_direction:
            return
        self.current_direction = direction
        self._draw_direction(direction)
        glyph, color, label = DIRECTION_STYLES[direction]
        self.dir_label.config(text=f"{glyph}  {label}", foreground=color)

    def _draw_direction(self, direction):
        c = self.dir_canvas
        c.delete("all")
        cx, cy = 25, 25
        c.create_oval(cx - 23, cy - 23, cx + 23, cy + 23, outline=ACCENT_DIM)

        if direction is None:
            c.create_text(cx, cy, text="—", fill=FG_DIM, font=("Consolas", 18, "bold"))
            return

        _glyph, color, _label = DIRECTION_STYLES[direction]
        if direction == "stop":
            r = 12
            c.create_rectangle(cx - r, cy - r, cx + r, cy + r, fill=color, outline="")
            return

        angle_deg = {"forward": 0, "right": 90, "back": 180, "left": 270}[direction]
        rad = math.radians(angle_deg)
        pts = [(0, -18), (-11, 9), (0, 1), (11, 9)]  # arrow pointing "up" before rotation
        rotated = []
        for px, py in pts:
            rx = px * math.cos(rad) - py * math.sin(rad)
            ry = px * math.sin(rad) + py * math.cos(rad)
            rotated.append((cx + rx, cy + ry))
        c.create_polygon(rotated, fill=color, outline="")

    # ---------------- LiDAR ----------------
    def _toggle_lidar_connect(self):
        if self.lidar_worker:
            self.lidar_worker.stop()
            self.lidar_worker = None
            self.lidar_connect_btn.config(text="Connect LiDAR")
            self.lidar_status_label.config(text="● LIDAR: DISCONNECTED", foreground=DANGER)
            return

        port = self.lidar_port_var.get()
        if not port:
            self._log("No LiDAR port selected.")
            return
        if C1Lidar is None:
            self._log("ERROR: 'rplidarc1' package not installed (pip install rplidarc1).")
            return

        self.lidar_worker = LidarWorker(port, self._on_lidar_point, self._on_lidar_status)
        self.lidar_worker.start()
        self.lidar_connect_btn.config(text="Disconnect LiDAR")
        self.lidar_status_label.config(text=f"● LIDAR: CONNECTING ON {port}...", foreground=WARNING)

    def _on_lidar_status(self, msg):
        self.root.after(0, self._apply_lidar_status, msg)

    def _apply_lidar_status(self, msg):
        self._log(f"LiDAR: {msg}")
        if "connecting" in msg.lower():
            self.lidar_status_label.config(text=f"● {msg}", foreground=WARNING)
        elif "streaming" in msg.lower():
            self.lidar_status_label.config(text="● LIDAR: CONNECTED, STREAMING", foreground=SUCCESS)
        elif "error" in msg.lower() or "stopped" in msg.lower() or "ended" in msg.lower():
            self.lidar_status_label.config(text=f"● {msg}", foreground=DANGER)

    def _on_lidar_point(self, angle_deg, distance_mm, quality):
        # Called from the LiDAR background thread - just update the shared
        # snapshot here; all UI/serial work happens in the Tk-thread timer.
        with self.scan_lock:
            self.scan_points[int(angle_deg) % 360] = (distance_mm, quality)

    # ---------------- Map draw + obstacle check (runs on the Tk thread) ----------------
    def _refresh_map(self):
        self._draw_map_and_check_obstacle()
        self._check_stall_alert()
        self.root.after(150, self._refresh_map)  # ~6-7 Hz refresh

    def _draw_map_and_check_obstacle(self):
        c = self.canvas
        c.delete("all")
        cx, cy = c.winfo_width() // 2, c.winfo_height() // 2
        max_r = self.canvas_size // 2 - 10
        scale = max_r / MAP_MAX_RANGE_MM

        # Shade the front-180 sector (heading 0 = straight up on screen)
        c.create_arc(cx - max_r, cy - max_r, cx + max_r, cy + max_r,
                     start=0, extent=180, fill="#0a1f2e", outline="", style="pieslice")
        # Crosshair spokes
        for ang in (0, 45, 90, 135, 180, 225, 270, 315):
            theta = math.radians(ang - 90)
            c.create_line(cx, cy, cx + max_r * math.cos(theta), cy + max_r * math.sin(theta),
                          fill="#0f2536")
        # Range rings (glow: faint outer + crisp inner)
        for frac in (0.25, 0.5, 0.75, 1.0):
            r = max_r * frac
            c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=ACCENT_DIM)
            if frac < 1.0:
                c.create_text(cx + 4, cy - r + 8, text=f"{int(MAP_MAX_RANGE_MM * frac)}mm",
                              fill=FG_DIM, font=("Consolas", 7), anchor="w")
        c.create_oval(cx - max_r, cy - max_r, cx + max_r, cy + max_r, outline=ACCENT, width=2)
        # Robot marker (heading = straight up), with a soft glow halo
        c.create_oval(cx - 12, cy - 12, cx + 12, cy + 12, outline=ACCENT_DIM)
        c.create_polygon(cx, cy - 10, cx - 6, cy + 6, cx + 6, cy + 6, fill=ACCENT, outline="")

        threshold = self.obstacle_threshold_mm.get()
        obstacle_found = False
        obstacle_dist = None

        with self.scan_lock:
            points = list(self.scan_points.items())

        for angle_deg, (dist_mm, quality) in points:
            if quality < MIN_QUALITY or dist_mm <= 0:
                continue
            r = min(dist_mm, MAP_MAX_RANGE_MM) * scale
            # angle_deg: 0 = front/up, increasing clockwise
            theta = math.radians(angle_deg - 90)  # rotate so 0 deg = up
            x = cx + r * math.cos(theta)
            y = cy + r * math.sin(theta)

            in_front = angle_in_front(angle_deg)
            is_obstacle_point = in_front and dist_mm <= threshold

            if is_obstacle_point:
                obstacle_found = True
                if obstacle_dist is None or dist_mm < obstacle_dist:
                    obstacle_dist = dist_mm
                color = DANGER
            elif in_front:
                color = WARNING
            else:
                color = ACCENT

            c.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline="")

        self._handle_obstacle_state(obstacle_found, obstacle_dist)

    def _handle_obstacle_state(self, obstacle_found, obstacle_dist):
        if obstacle_found == self.obstacle_active:
            # no state change, but keep the label distance fresh
            if obstacle_found:
                self.obstacle_label.config(text=f"⚠ OBSTACLE AT {obstacle_dist} MM — WAITING", foreground=DANGER)
            return

        self.obstacle_active = obstacle_found

        if obstacle_found:
            self.obstacle_label.config(text=f"⚠ OBSTACLE AT {obstacle_dist} MM — WAITING", foreground=DANGER)
            self._log(f"Voice: \"{OBSTACLE_VOICE_MSG}\"")
            self.speech.speak(OBSTACLE_VOICE_MSG)
            if self.is_running and not self.auto_paused:
                self._log(f"Obstacle detected at {obstacle_dist} mm in front 180 -> STOP")
                self._send("STOP")
                self.auto_paused = True
                self.obstacle_pause_since = time.time()
                self._update_status()
        else:
            self.obstacle_label.config(text="✓ PATH CLEAR", foreground=SUCCESS)
            if self.auto_paused:
                self._log("Obstacle cleared -> resuming")
                self._send("START")
                self.auto_paused = False
                self.obstacle_pause_since = None
                self._clear_alert()
                self._update_status()


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotDashboard(root)
    root.mainloop()