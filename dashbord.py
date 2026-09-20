"""
Robot Dashboard - manual drive + scripted PATH playback (with RPLidar C1
obstacle detection)
----------------------------------------------------------------------------
Two independent serial links:

  1) Teensy (robot control) - plain text commands, newline terminated:
        MANUAL/MFWD/MBACK/MLEFT/MRIGHT/MSTOP -> manual takeover and drive
        PATH:<steps> / PATH_STOP / PATH_PAUSE / PATH_RESUME -> scripted
            timed-move playback (built or recorded in the PATH tab)
        STOP     -> stop immediately, cancelling manual or path mode
        FOOD     -> query the IR food-tray sensor
     The dashboard sends PATH_PAUSE/PATH_RESUME itself (in addition to the
     buttons) whenever the LiDAR sees/clears an obstacle in front of the
     robot while a path is running.

     The Teensy pushes FOOD:PRESENT / FOOD:TAKEN when the IR tray sensor
     changes. A FOOD:TAKEN after a delivery path finished is the pickup
     event: the dashboard says "Thank you sir" and, a few seconds later,
     drives build_return_path() - a 180 degree U-turn, the delivery path
     retraced with its turns mirrored, then a second U-turn to park the
     robot on its original heading.

     There is no closed-loop line-following or encoder odometry - the
     robot's wheel encoders have a hardware fault that can't be replaced,
     so all driving is either manual or open-loop scripted PATH playback.

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
    python dashbord.py
    (opens fullscreen, covering the taskbar - "Exit full screen" in the header,
     or Esc/F11, gets out; DASHBOARD_WINDOWED=1 starts in a normal window)

NOTE: the exact import path for the RPLidar class depends on how the
'rplidarc1' package you installed is laid out. If the import below fails,
run:  python -c "import rplidarc1, pkgutil; print(pkgutil.walk_packages(rplidarc1.__path__))"
and adjust the "from ... import RPLidar" line to match.
"""

import asyncio
import http.server
import json
import math
import os
import queue
import socket
import socketserver
import threading
import time
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk
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

# A Bluetooth speaker's A2DP link takes a moment to wake up from idle, and
# whatever is said in that gap is lost - the first word or two. The commas
# make espeak generate a short silence first, so the stream is already
# running by the time the words start. Set to "" for a wired speaker.
VOICE_LEAD_IN = ", , "

# Everything the robot says out loud, in one place (pyttsx3 -> espeak-ng).
OBSTACLE_VOICE_MSG = "Please give side"          # something entered the front-180 zone
STALL_VOICE_MSG = "Robot needs help, please take control"  # still blocked after STALL_ALERT_SECONDS

# ---- Food tray (IR sensor) + automatic return trip ----
FOOD_TAKEN_VOICE_MSG = "Thank you sir"           # the customer lifted the food off the tray
RETURN_DONE_VOICE_MSG = "Back at the counter"    # return trip finished
RETURN_DELAY_SECONDS = 3     # pause after the thank-you before pulling away, so the
                              # customer hears it and steps clear of the robot
RETURN_TURN_ACTION = "TURN RIGHT"  # which way the robot spins for its two U-turns
RETURN_TURN_DEGREES = 180
ALIGN_OK_RESIDUAL_DEG = 5    # a bigger residual from an ALIGN step means it timed out
                              # rather than reached the heading (keep >= the firmware's
                              # ALIGN_TOLERANCE_DEG, which is what it settles for)
PATH_MAX_STEPS = 64          # must match PATH_MAX_STEPS in lineflow.ino - the firmware
                              # silently drops steps past this, which would strand the
                              # robot mid-return, so we refuse to send an over-long path

LIDAR_BAUDRATE = 460800
MIN_QUALITY = 5            # ignore very low-confidence points
MAP_MAX_RANGE_MM = 4000    # canvas display range (outer ring)
FRONT_HALF_ANGLE = 90      # "front 180" = heading +/- 90 degrees

PHONE_SERVER_PORT = 8765   # phone remote control - browse to http://<laptop-lan-ip>:8765
STALL_ALERT_SECONDS = 10   # alert the phone if stopped this long (line lost or obstacle)
MANUAL_COMMANDS = {"MFWD", "MBACK", "MLEFT", "MRIGHT", "MSTOP"}

# PATH tab step labels -> the action tokens lineflow.ino's PATH: command expects.
# TURN LEFT/TURN RIGHT (rotate N degrees) and ALIGN TO START (rotate until back
# on the heading the delivery began at, +N degrees) are closed-loop via the
# gyro and take degrees; everything else is open-loop/timed and takes seconds.
PATH_ACTION_TOKENS = {"FORWARD": "FWD", "BACK": "BACK", "LEFT": "LEFT",
                       "RIGHT": "RIGHT", "HOLD": "HOLD",
                       "TURN LEFT": "TURNL", "TURN RIGHT": "TURNR",
                       "ALIGN TO START": "ALIGN"}
TURN_ACTIONS = {"TURN LEFT", "TURN RIGHT"}
ALIGN_ACTION = "ALIGN TO START"
# Everything whose value is degrees rather than seconds - the unit difference
# matters for the spinbox, the step display, the saved-path summary and the
# PATH: wire format (degrees go as-is, seconds are multiplied into ms).
DEGREE_ACTIONS = TURN_ACTIONS | {ALIGN_ACTION}
MANUAL_CMD_TO_PATH_ACTION = {"MFWD": "FORWARD", "MBACK": "BACK",
                             "MLEFT": "LEFT", "MRIGHT": "RIGHT"}

# Retracing a path after a 180 degree U-turn: the robot now faces back down
# the route, so each leg is driven the same way round (FORWARD stays FORWARD,
# BACK stays BACK) but every turn is mirrored. Worked through: start facing
# north, FORWARD 3s -> (0,3); TURN LEFT 90 -> facing west; FORWARD 2s ->
# (-2,3). U-turn to face east, FORWARD 2s -> (0,3), then a *right* 90 puts
# the robot on south, FORWARD 3s -> (0,0) home, and the closing U-turn
# restores the original north heading.
REVERSE_ACTION_MIRROR = {"FORWARD": "FORWARD", "BACK": "BACK", "HOLD": "HOLD",
                         "LEFT": "RIGHT", "RIGHT": "LEFT",
                         "TURN LEFT": "TURN RIGHT", "TURN RIGHT": "TURN LEFT"}

# Library of named PATH step-sequences, persisted next to this script so
# they survive a dashboard restart.
SAVED_PATHS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "saved_paths.json")
MIN_RECORDED_GAP_SECONDS = 0.3  # idle gaps shorter than this aren't recorded as a HOLD step
MIN_RECORDED_TURN_DEG = 2       # a gyro-measured turn smaller than this stays a timed nudge
MTURN_MATCH_WINDOW_S = 1.0      # an MTURN report pairs with a recorded turn at most this old
HDG_STALE_SECONDS = 1.5         # no HDG line for this long -> the dial shows "no gyro data"
HEADING_DIAL_SIZE = 104         # px, square
TIMED_TO_GYRO_TURN = {"LEFT": "TURN LEFT", "RIGHT": "TURN RIGHT"}

# =====================================================================
# Light "classic" theme - standard light desktop-app palette used
# throughout the Tkinter dashboard UI (the phone remote page below has
# its own separate, unrelated dark theme).
# =====================================================================
BG_MAIN = "#f0f0f0"      # window background (classic light gray)
BG_PANEL = "#ffffff"     # panel/frame background
BG_PANEL_ALT = "#e6e6e6" # button faces / alternate panel background
BG_INSET = "#ffffff"     # text/listbox/canvas insets
FG_TEXT = "#1e1e1e"      # primary text
FG_DIM = "#6e6e6e"       # secondary/dim text
ACCENT = "#0058a3"       # primary blue
ACCENT_DIM = "#a9c6e8"   # light blue - borders, dim highlights
ACCENT2 = "#8e24aa"      # purple highlight (manual/turn indicators)
SUCCESS = "#1e7d32"      # dark green
DANGER = "#c62828"       # classic red
WARNING = "#9a6d00"      # dark amber (readable on light backgrounds)

# Very light tints used as button hover/active backgrounds, and subtle
# fills on the LiDAR map canvas - kept separate from the palette above
# since they're deliberately much lighter than any of those colors.
SUCCESS_TINT = "#dff2e1"
DANGER_TINT = "#fbdede"
MAP_SECTOR_FILL = "#eaf2fb"
MAP_SPOKE_COLOR = "#cfd8e3"

FONT_MONO = ("Consolas", 9)
FONT_MONO_BOLD = ("Consolas", 9, "bold")
FONT_HEADER = ("Consolas", 12, "bold")
FONT_STATUS = ("Consolas", 9, "bold")
FONT_BTN = ("Consolas", 9, "bold")

# direction key -> (glyph, color, label) - used to visualize robot heading
DIRECTION_STYLES = {
    "left":    ("◀", ACCENT2, "TURN LEFT"),
    "right":   ("▶", ACCENT2, "TURN RIGHT"),
    "forward": ("▲", SUCCESS, "FORWARD"),
    "back":    ("▼", WARNING, "REVERSE"),
    "stop":    ("■", DANGER, "STOPPED"),
}


def normalize_deg180(deg):
    """Wrap to (-180, 180] - the short way round, as lineflow.ino's ALIGN does."""
    deg = math.fmod(deg, 360.0)
    if deg > 180:
        deg -= 360
    elif deg <= -180:
        deg += 360
    return deg


def build_return_path(steps):
    """The delivery path, driven home: U-turn, retrace, U-turn.

    Steps run in reverse order with every turn mirrored (see
    REVERSE_ACTION_MIRROR), bracketed by two RETURN_TURN_DEGREES turns - the
    first to face back down the route, the last to leave the robot parked on
    its original heading, ready for the next delivery.

    The last step is ALIGN TO START, which turns until the gyro says the robot
    is back on the heading the delivery began at (HEADING_ANCHOR, latched when
    the outbound path was sent). The U-turn before it does the bulk of the
    rotation; ALIGN trims off whatever every step of the round trip
    accumulated, so the robot parks facing the way it left instead of drifting
    a few degrees further off with each delivery.

    HOLD steps are kept so the retrace stays a faithful mirror of the drive
    out; delete them from the PATH tab before it runs if the waits aren't
    wanted on the way back. ALIGN steps in the delivery path are dropped -
    they're absolute headings for the outbound route and mean nothing
    reversed.
    """
    turn = (RETURN_TURN_ACTION, RETURN_TURN_DEGREES)
    mirrored = [(REVERSE_ACTION_MIRROR.get(action, action), value)
                for action, value in reversed(steps) if action != ALIGN_ACTION]
    return [turn] + mirrored + [turn, (ALIGN_ACTION, 0)]


def angle_in_front(angle_deg):
    """True if angle_deg (0-360, 0 = straight ahead) is within the front 180."""
    a = angle_deg % 360
    return a <= FRONT_HALF_ANGLE or a >= (360 - FRONT_HALF_ANGLE)


# =====================================================================
# LiDAR background worker (runs its own asyncio loop in a separate thread)
# =====================================================================
RECONNECT_DELAY_SECONDS = 3  # pause before retrying after the lidar lib crashes on a bad packet


class LidarWorker:
    def __init__(self, port, on_point, on_status):
        self.port = port
        self.on_point = on_point       # callback(angle_deg, distance_mm, quality)
        self.on_status = on_status     # callback(str) - thread-safe logging
        self.lidar = None
        self.thread = None
        self._stop_flag = threading.Event()      # signals the current scan session to stop
        self._stop_requested = threading.Event()  # user asked to disconnect - no more retries

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        # rplidarc1 occasionally raises IndexError deep inside its packet
        # parser when a serial read comes back short/partial (common over
        # USB-serial) - it doesn't recover on its own, so we tear down and
        # reconnect instead of leaving the dashboard silently disconnected.
        while not self._stop_requested.is_set():
            self._stop_flag.clear()
            try:
                asyncio.run(self._main())
            except Exception as e:
                self.on_status(f"LiDAR worker ended: {e}")

            if self._stop_requested.is_set():
                break
            self.on_status(f"LiDAR reconnecting in {RECONNECT_DELAY_SECONDS}s...")
            self._stop_requested.wait(RECONNECT_DELAY_SECONDS)

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
        self._stop_requested.set()
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
                self._engine.say(VOICE_LEAD_IN + text)
                self._engine.runAndWait()
            except Exception:
                pass

    def speak(self, text):
        if pyttsx3 is None:
            return
        self._queue.put(text)


# =====================================================================
# Phone remote control (LAN web page - stop / run path / manual d-pad)
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
  button { padding:26px 8px; font-size:1.15rem; font-weight:bold; border-radius:10px;
           border:1px solid #0a5f70; background:#101a2e; color:#00e5ff; touch-action:none; }
  button:active { background:#0a5f70; }
  button.stop { color:#ff3b5c; border-color:#ff3b5c; }
  #stopBtn { width:100%; }
  #pathBtn { width:100%; margin-top:12px; color:#ff2ea6; border-color:#ff2ea6; }
  #returnBtn { width:100%; margin-top:12px; color:#ffb800; border-color:#ffb800; }
  .tray { text-align:center; margin-top:14px; font-weight:bold; font-size:0.95rem; }
  .obstacle { text-align:center; margin-top:16px; font-weight:bold; min-height:1.2em; }
  .radar-wrap { display:flex; justify-content:center; margin-top:10px; }
  #radar { background:#020508; border:1px solid #0a5f70; border-radius:8px; max-width:100%; }
  .radar-legend { text-align:center; font-size:0.7rem; color:#5b7a94; margin-top:4px; }
  .radar-legend span { padding:0 6px; }
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
  <button id="stopBtn" class="stop" onclick="post('/api/stop')">&#9632; STOP</button>
  <button id="pathBtn" onclick="post('/api/path/run')">&#9654; RUN PATH</button>
  <button id="returnBtn" onclick="post('/api/path/return')">&#8617; RETURN HOME</button>
  <div class="tray" id="tray"></div>
  <div class="tray" id="heading"></div>
  <div class="obstacle" id="obstacle"></div>

  <div class="radar-wrap">
    <canvas id="radar" width="320" height="320"></canvas>
  </div>
  <div class="radar-legend">
    <span style="color:#ff3b5c">&#9679; blocked</span>
    <span style="color:#ffb800">&#9679; front (clear)</span>
    <span style="color:#00e5ff">&#9679; behind</span>
  </div>

  <div class="manual-wrap">
    <button id="manualBtn" onclick="post('/api/manual/on')">&#9998; TAKE MANUAL CONTROL</button>
    <div class="dpad" id="dpad">
      <div></div><button class="dbtn" data-cmd="fwd">&#9650;</button><div></div>
      <button class="dbtn" data-cmd="left">&#9664;</button>
      <button class="dbtn stopbtn" data-cmd="stop">&#9632;</button>
      <button class="dbtn" data-cmd="right">&#9654;</button>
      <div></div><button class="dbtn" data-cmd="back">&#9660;</button><div></div>
    </div>
    <div class="hint" id="manualHint">Manual control active. Steer the robot, then press
      &#9632; STOP above to release manual control, or run a saved PATH.</div>
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

// ---- LiDAR radar canvas ----
const radar = document.getElementById('radar');
const radarCtx = radar.getContext('2d');
function sizeRadar() {
  const side = Math.min(radar.parentElement.clientWidth, 420);
  radar.width = side;
  radar.height = side;
}
sizeRadar();
window.addEventListener('resize', sizeRadar);

function drawRadar(scan) {
  const w = radar.width, h = radar.height;
  const cx = w / 2, cy = h / 2;
  const maxR = Math.min(w, h) / 2 - 10;
  const maxRangeMm = scan.max_range_mm;
  const scale = maxR / maxRangeMm;
  const threshold = scan.threshold_mm;
  const frontHalf = scan.front_half_angle;

  radarCtx.clearRect(0, 0, w, h);
  radarCtx.fillStyle = '#020508';
  radarCtx.fillRect(0, 0, w, h);

  // Shade the front-180 sector (heading 0 = straight up)
  radarCtx.beginPath();
  radarCtx.moveTo(cx, cy);
  radarCtx.arc(cx, cy, maxR, Math.PI, 2 * Math.PI);
  radarCtx.closePath();
  radarCtx.fillStyle = '#0a1f2e';
  radarCtx.fill();

  // Range rings
  [0.25, 0.5, 0.75, 1.0].forEach(function(frac) {
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, maxR * frac, 0, 2 * Math.PI);
    radarCtx.strokeStyle = '#0a5f70';
    radarCtx.lineWidth = 1;
    radarCtx.stroke();
  });
  radarCtx.beginPath();
  radarCtx.arc(cx, cy, maxR, 0, 2 * Math.PI);
  radarCtx.strokeStyle = '#00e5ff';
  radarCtx.lineWidth = 2;
  radarCtx.stroke();

  // Robot marker (heading = straight up)
  radarCtx.fillStyle = '#00e5ff';
  radarCtx.beginPath();
  radarCtx.moveTo(cx, cy - 10);
  radarCtx.lineTo(cx - 6, cy + 6);
  radarCtx.lineTo(cx + 6, cy + 6);
  radarCtx.closePath();
  radarCtx.fill();

  scan.points.forEach(function(p) {
    const angleDeg = p[0], distMm = p[1];
    const r = Math.min(distMm, maxRangeMm) * scale;
    const theta = (angleDeg - 90) * Math.PI / 180;  // rotate so 0deg = up
    const x = cx + r * Math.cos(theta);
    const y = cy + r * Math.sin(theta);
    const a = ((angleDeg % 360) + 360) % 360;
    const inFront = a <= frontHalf || a >= (360 - frontHalf);

    let color;
    if (inFront && distMm <= threshold) color = '#ff3b5c';
    else if (inFront) color = '#ffb800';
    else color = '#00e5ff';

    radarCtx.fillStyle = color;
    radarCtx.beginPath();
    radarCtx.arc(x, y, 2.5, 0, 2 * Math.PI);
    radarCtx.fill();
  });
}

async function pollScan() {
  try {
    const r = await fetch('/api/scan');
    const scan = await r.json();
    drawRadar(scan);
  } catch (e) {}
}

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
    let state = 'STOPPED';
    if (s.manual_mode) state = 'MANUAL CONTROL';
    else if (s.path_active) state = s.returning ? 'RETURNING HOME' : 'PATH RUNNING';
    else if (s.auto_paused) state = 'WAITING (OBSTACLE)';
    else if (s.awaiting_pickup) state = 'WAITING FOR PICKUP';
    document.getElementById('status').innerHTML =
      (s.connected ? '<b>CONNECTED</b>' : '<span style="color:#ff3b5c">DISCONNECTED</span>')
      + ' &nbsp; STATE: <b>' + state + '</b>';
    const trayEl = document.getElementById('tray');
    if (s.food_present === null || s.food_present === undefined) {
      trayEl.textContent = 'TRAY: no reading';
      trayEl.style.color = '#5b7a94';
    } else if (s.food_present) {
      trayEl.textContent = 'TRAY: FOOD LOADED';
      trayEl.style.color = '#00ffa3';
    } else {
      trayEl.textContent = s.awaiting_pickup ? 'TRAY: EMPTY - WAITING FOR PICKUP' : 'TRAY: EMPTY';
      trayEl.style.color = s.awaiting_pickup ? '#ffb800' : '#5b7a94';
    }

    const hdEl = document.getElementById('heading');
    if (s.heading_from_start === null || s.heading_from_start === undefined) {
      hdEl.textContent = 'HEADING: no gyro data';
      hdEl.style.color = '#5b7a94';
    } else {
      const h = s.heading_from_start;
      hdEl.textContent = 'HEADING: ' + (h > 0 ? '+' : '') + h + ' deg from start'
        + (s.gyro_calibrated ? '' : ' (turn once to calibrate)');
      hdEl.style.color = Math.abs(h) > 5 ? '#ffb800' : '#00ffa3';
    }

    const obEl = document.getElementById('obstacle');
    obEl.textContent = s.obstacle
      ? ('\\u26A0 OBSTACLE' + (s.obstacle_dist_mm ? ' AT ' + s.obstacle_dist_mm + 'MM' : '') + ' \\u2014 WAITING')
      : '\\u2713 PATH CLEAR';
    obEl.style.color = s.obstacle ? '#ff3b5c' : '#00ffa3';

    document.getElementById('manualBtn').style.display = s.manual_mode ? 'none' : 'block';
    document.getElementById('dpad').style.display = s.manual_mode ? 'grid' : 'none';
    document.getElementById('manualHint').style.display = s.manual_mode ? 'block' : 'none';

    const bar = document.getElementById('alertbar');
    if (s.alert) {
      const secs = s.stalled_seconds ? ' (' + Math.round(s.stalled_seconds) + 's)' : '';
      bar.textContent = '\\u26A0 ROBOT NEEDS HELP \\u2014 BLOCKED BY OBSTACLE' + secs;
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
setInterval(pollScan, 300);
poll();
pollScan();
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
            if d.obstacle_pause_since is not None:
                stalled_secs = now - d.obstacle_pause_since
            # Read once: this runs on the HTTP thread, and the Tk thread can
            # blank the heading (stale gyro) between two separate reads.
            heading = d._heading_from_start()
            self._send_json({
                "connected": bool(d.ser and d.ser.is_open),
                "auto_paused": d.auto_paused,
                "obstacle": d.obstacle_active,
                "obstacle_dist_mm": d.obstacle_dist_mm,
                "manual_mode": d.manual_mode,
                "path_active": d.path_active,
                "alert": d.alert_active,
                "stalled_seconds": None if stalled_secs is None else round(stalled_secs, 1),
                "food_present": d.food_present,
                "awaiting_pickup": d.awaiting_pickup,
                "returning": d.return_active,
                "heading_from_start": None if heading is None else round(heading),
                "gyro_calibrated": bool(d.gyro_right_sign),
            })
        elif path == "/api/scan":
            with d.scan_lock:
                points = [[a, dm, q] for a, (dm, q) in d.scan_points.items()
                          if q >= MIN_QUALITY and dm > 0]
            self._send_json({
                "points": points,
                "threshold_mm": d.obstacle_threshold_mm.get(),
                "max_range_mm": MAP_MAX_RANGE_MM,
                "front_half_angle": FRONT_HALF_ANGLE,
            })
        else:
            self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        d = self.dashboard
        actions = {
            "/api/stop": d._send_stop,
            "/api/path/run": d._run_path,
            "/api/path/return": d._manual_return,
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

        # Fit the window to the actual screen exactly (works everywhere,
        # including small kiosk displays with no/minimal window manager,
        # where "zoomed"/"-zoomed" often silently does nothing).
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        self.compact = screen_w <= 1024 or screen_h <= 600  # e.g. 800x480 Pi panel
        self.root.geometry(f"{screen_w}x{screen_h}+0+0")
        if not self.compact:
            try:
                self.root.state("zoomed")
            except tk.TclError:
                pass

        # True fullscreen by default: covers the desktop taskbar and drops the
        # title bar, so the whole 800x480 panel is dashboard. That also removes
        # the window's close button, and a touchscreen has no F11 - the header's
        # "Exit full screen" button is the way out. Set DASHBOARD_WINDOWED=1 to
        # start in a normal window (developing on a PC).
        self.fullscreen = not os.environ.get("DASHBOARD_WINDOWED")
        self.root.bind("<F11>", lambda e: self._set_fullscreen(not self.fullscreen))
        self.root.bind("<Escape>", lambda e: self._set_fullscreen(False))

        # ---- Robot serial state ----
        self.ser = None
        self.reader_running = False
        self.manual_mode = False       # True after MANUAL takeover (phone or desktop)
        self._manual_repeat_job = None  # after() handle for desktop press-and-hold
        self.path_active = False       # True while the Teensy is playing back a PATH
        self.path_steps = []           # [(action_label, value), ...] built in the PATH tab -
                                        # value is seconds, except degrees for TURN LEFT/TURN RIGHT

        # ---- Food tray (IR sensor) + automatic return trip ----
        # The delivery cycle: run a path -> PATH:DONE arms awaiting_pickup ->
        # the IR sensor reports FOOD:TAKEN -> thank the customer, then drive
        # build_return_path(delivery_steps). return_active marks the return
        # trip itself so finishing it doesn't arm another pickup.
        self.food_present = None        # None until the Teensy reports (no sensor/not connected)
        self.delivery_steps = []        # snapshot of the last outbound path, for the retrace
        self.awaiting_pickup = False    # delivered, waiting for the customer to take the food
        self.return_active = False      # the path currently running IS the return trip
        self.auto_return_enabled = tk.BooleanVar(value=True)
        self._return_job = None         # after() handle for the post-thank-you delay

        # ---- Record-by-driving (manual drive -> PATH steps) ----
        self.recording = False
        self._record_active_cmd = None   # currently-held MFWD/MBACK/MLEFT/MRIGHT, or None
        self._record_start_time = None   # time.time() when _record_active_cmd started
        self._record_idle_since = None   # time.time() since the last release, for HOLD gaps
        # Recorded LEFT/RIGHT steps (indices into path_steps) waiting for the
        # Teensy's MTURN report of how far that turn really rotated, which
        # upgrades them to gyro-measured TURN LEFT/RIGHT - see _on_manual_turn.
        self._record_pending_turns = []

        # ---- Live gyro (HDG stream from the Teensy, drawn on the heading dial) ----
        self.heading_deg = None          # raw integrated heading; None until the first HDG
        self.gyro_right_sign = 0         # +1/-1 once the Teensy has learned it, 0 = not yet
        self.anchor_deg = 0.0            # heading latched by the last HEADING_ANCHOR
        self._last_hdg_time = None       # time.time() of the last HDG line, for staleness

        # ---- Saved path library (persisted to SAVED_PATHS_FILE) ----
        self.saved_paths = {}             # {name: [(action_label, seconds), ...]}
        self._load_saved_paths()

        # ---- LiDAR state ----
        self.lidar_worker = None
        self.scan_points = {}          # {angle_deg_int: (distance_mm, quality)}
        self.scan_lock = threading.Lock()
        self.obstacle_active = False
        self.obstacle_dist_mm = None
        self.auto_paused = False       # True if WE stopped the robot for an obstacle
        self.obstacle_threshold_mm = tk.IntVar(value=400)
        self.current_direction = None   # one of DIRECTION_STYLES keys, or None

        # ---- Stall / alert tracking (blocked by an obstacle >10s -> alert phone) ----
        self.obstacle_pause_since = None
        self.alert_active = False

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
        self._update_food_label()
        self._draw_heading_dial()
        self._set_fullscreen(self.fullscreen)
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
                         borderwidth=2, relief="groove")
        style.configure("TLabelframe.Label", background=BG_PANEL, foreground=ACCENT,
                         font=FONT_MONO_BOLD)

        style.configure("TLabel", background=BG_PANEL, foreground=FG_TEXT, font=FONT_MONO)
        style.configure("TCheckbutton", background=BG_PANEL, foreground=FG_TEXT, font=FONT_MONO)
        style.map("TCheckbutton", background=[("active", BG_PANEL)])
        style.configure("Header.TLabel", background=BG_MAIN, foreground=ACCENT, font=FONT_HEADER)
        style.configure("SubHeader.TLabel", background=BG_MAIN, foreground=FG_DIM,
                         font=("Consolas", 9))

        style.configure("TButton", background=BG_PANEL_ALT, foreground=FG_TEXT,
                         font=FONT_BTN, borderwidth=2, relief="raised", focuscolor=BG_PANEL_ALT)
        style.map("TButton",
                  background=[("active", ACCENT_DIM), ("pressed", ACCENT_DIM)],
                  foreground=[("disabled", FG_DIM)])

        style.configure("TCombobox", fieldbackground=BG_PANEL, background=BG_PANEL_ALT,
                         foreground=FG_TEXT, arrowcolor=FG_TEXT, bordercolor=ACCENT_DIM,
                         insertcolor=FG_TEXT)
        style.map("TCombobox", fieldbackground=[("readonly", BG_PANEL)],
                   foreground=[("readonly", FG_TEXT)])

        style.configure("TSpinbox", fieldbackground=BG_PANEL, background=BG_PANEL_ALT,
                         foreground=FG_TEXT, arrowcolor=FG_TEXT, bordercolor=ACCENT_DIM,
                         insertcolor=FG_TEXT)
        # The spinboxes are readonly (no on-screen keyboard) - keep them looking
        # normal rather than greyed out like a disabled widget.
        style.map("TSpinbox", fieldbackground=[("readonly", BG_PANEL)],
                   foreground=[("readonly", FG_TEXT)])

        style.configure("TNotebook", background=BG_MAIN, borderwidth=0)
        style.configure("TNotebook.Tab", background=BG_PANEL_ALT, foreground=FG_DIM,
                         font=FONT_BTN, padding=[10, 6], borderwidth=1)
        style.map("TNotebook.Tab",
                  background=[("selected", BG_PANEL)],
                  foreground=[("selected", ACCENT)])

        # Make the dropdown listboxes match the light theme too
        self.root.option_add("*TCombobox*Listbox.background", BG_PANEL)
        self.root.option_add("*TCombobox*Listbox.foreground", FG_TEXT)
        self.root.option_add("*TCombobox*Listbox.selectBackground", ACCENT_DIM)
        self.root.option_add("*TCombobox*Listbox.font", FONT_MONO)

        # Colored-text ttk.Button variants, used instead of _neon_button
        # (raw tk.Button) in the PATH/SAVED tabs' bottom button rows: a
        # tk.Button there reliably failed to ever paint on this Tk build
        # (reported mapped/viewable with correct geometry, but nothing was
        # drawn, even when forced to an unmissable debug color) - ttk.Button
        # rendered correctly in every context tested, including these same
        # tabs, so these styles sidestep the issue rather than chase it.
        style.configure("Small.TButton", font=("Consolas", 8), padding=[6, 1])
        style.configure("Success.TButton", foreground=SUCCESS)
        style.configure("Danger.TButton", foreground=DANGER)
        style.configure("Accent.TButton", foreground=ACCENT)

    def _set_fullscreen(self, on):
        self.fullscreen = on
        try:
            self.root.attributes("-fullscreen", on)
        except tk.TclError:
            pass  # a window manager that can't do it - stay a screen-sized window
        self.fullscreen_btn.config(text="Exit full screen" if on else "Full screen")

    def _neon_button(self, parent, **kwargs):
        # highlightthickness=0 is deliberately avoided here: on some Tk/Windows
        # builds a packed tk.Button with zero highlight thickness fails to
        # ever paint (reports mapped/viewable correctly, but nothing is drawn)
        # - a long-standing Tk redraw/damage-region quirk. A minimal 1px
        # thickness matching the background sidesteps it invisibly.
        defaults = dict(
            font=FONT_BTN, bg=BG_PANEL_ALT, fg=FG_TEXT, activebackground=ACCENT_DIM,
            activeforeground=FG_TEXT, disabledforeground=FG_DIM, relief="raised",
            highlightthickness=1, highlightbackground=BG_MAIN, highlightcolor=BG_MAIN,
            bd=2, cursor="hand2",
        )
        defaults.update(kwargs)
        return tk.Button(parent, **defaults)

    # ---------------- UI ----------------
    def _build_ui(self):
        header = ttk.Label(self.root, text="◈ MIST CAFE BOT ◈",
                            style="Header.TLabel", anchor="center")
        header.pack(fill="x", pady=(4, 0))
        # Placed over the header's right end rather than packed beside it, so
        # the title stays centred and the button costs the 480px screen no
        # extra height.
        self.fullscreen_btn = ttk.Button(header, style="Small.TButton",
                                         command=lambda: self._set_fullscreen(not self.fullscreen))
        self.fullscreen_btn.place(relx=1.0, rely=0.5, x=-6, anchor="e")
        if not self.compact:
            sub = ttk.Label(self.root, text="TEENSY LINK · RPLIDAR C1 · REAL-TIME OBSTACLE FIELD",
                             style="SubHeader.TLabel", anchor="center")
            sub.pack(fill="x", pady=(0, 4))

        # Small screens (e.g. the 800x480 Pi panel) can't fit the control
        # panel and the LiDAR map side by side, so each gets its own tab
        # and can use the full window instead of half of it.
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=4, pady=4)
        self.notebook = notebook

        control_tab = ttk.Frame(notebook)
        path_tab = ttk.Frame(notebook)
        saved_tab = ttk.Frame(notebook)
        lidar_tab = ttk.Frame(notebook)
        log_tab = ttk.Frame(notebook)
        notebook.add(control_tab, text="◆ CONTROL")
        notebook.add(path_tab, text="◆ PATH")
        notebook.add(saved_tab, text="◆ SAVED")
        notebook.add(lidar_tab, text="◆ LIDAR MAP")
        notebook.add(log_tab, text="◆ LOG")
        self.log_tab = log_tab  # _gyro_test jumps here to show the replies

        # ===== TAB 1: Robot control =====
        conn_frame = ttk.LabelFrame(control_tab, text="◆ ROBOT LINK (TEENSY)")
        conn_frame.pack(fill="x", padx=6, pady=(3, 2))

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=16, state="readonly")
        self.port_combo.grid(row=0, column=0, padx=6, pady=2)
        ttk.Button(conn_frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=1, padx=4)
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connect)
        self.connect_btn.grid(row=0, column=2, padx=6)
        self.conn_status = ttk.Label(conn_frame, text="● DISCONNECTED", foreground=DANGER,
                                      font=FONT_STATUS)
        self.conn_status.grid(row=0, column=3, padx=6)
        ttk.Button(conn_frame, text="Gyro test", style="Small.TButton",
                   command=self._gyro_test).grid(row=0, column=4, padx=(6, 2))
        ttk.Button(conn_frame, text="Motor test", style="Small.TButton",
                   command=self._motor_test).grid(row=0, column=5, padx=(2, 6))

        self.phone_url_label = ttk.Label(conn_frame, text="Phone remote: starting...",
                                          font=("Consolas", 8), foreground=FG_DIM,
                                          wraplength=760)
        self.phone_url_label.grid(row=1, column=0, columnspan=4, padx=6, pady=(0, 2), sticky="w")

        # STOP, the state line and the alert line share one row: stacked, they
        # (a whole bordered panel for one button, plus two text rows, the
        # alert one usually empty) pushed the FOOD TRAY panel off the bottom
        # of the 800x480 screen. STOP keeps its full size, at the right.
        control_frame = ttk.Frame(control_tab)
        control_frame.pack(fill="x", padx=6, pady=(3, 2))
        control_frame.columnconfigure(0, weight=1)
        self.stop_btn = self._neon_button(control_frame, text="■ STOP", width=16, height=2,
                                           fg=DANGER, activebackground=DANGER_TINT,
                                           state="disabled", command=self._send_stop)
        self.stop_btn.grid(row=0, column=1, rowspan=2, padx=(8, 0), sticky="e")

        self.status_label = ttk.Label(control_frame, text="STATE: STOPPED",
                                       font=FONT_STATUS, foreground=ACCENT, background=BG_MAIN)
        self.status_label.grid(row=0, column=0, sticky="w")

        self.alert_label = ttk.Label(control_frame, text="", font=FONT_STATUS, foreground=DANGER,
                                      background=BG_MAIN, wraplength=560)
        self.alert_label.grid(row=1, column=0, sticky="w")

        manual_frame = ttk.LabelFrame(control_tab, text="◆ MANUAL DRIVE (TAKEOVER)")
        manual_frame.pack(fill="x", padx=6, pady=2)
        self.manual_btn = self._neon_button(
            manual_frame, text="✎ TAKE MANUAL CONTROL", width=24,
            fg=WARNING, command=self._enter_manual)
        self.manual_btn.grid(row=0, column=0, rowspan=3, padx=(8, 16), pady=3, sticky="ns")

        self.mfwd_btn = self._neon_button(manual_frame, text="▲", width=4, state="disabled")
        self.mfwd_btn.grid(row=0, column=2, padx=4, pady=1)
        self.mfwd_btn.bind("<ButtonPress-1>", lambda e: self._manual_press("MFWD"))
        self.mfwd_btn.bind("<ButtonRelease-1>", lambda e: self._manual_release())

        self.mleft_btn = self._neon_button(manual_frame, text="◀", width=4, state="disabled")
        self.mleft_btn.grid(row=1, column=1, padx=4, pady=1)
        self.mleft_btn.bind("<ButtonPress-1>", lambda e: self._manual_press("MLEFT"))
        self.mleft_btn.bind("<ButtonRelease-1>", lambda e: self._manual_release())

        self.mstop_btn = self._neon_button(manual_frame, text="■", width=4, state="disabled",
                                            fg=DANGER, command=lambda: self._manual_move("MSTOP"))
        self.mstop_btn.grid(row=1, column=2, padx=4, pady=1)

        self.mright_btn = self._neon_button(manual_frame, text="▶", width=4, state="disabled")
        self.mright_btn.grid(row=1, column=3, padx=4, pady=1)
        self.mright_btn.bind("<ButtonPress-1>", lambda e: self._manual_press("MRIGHT"))
        self.mright_btn.bind("<ButtonRelease-1>", lambda e: self._manual_release())

        self.mback_btn = self._neon_button(manual_frame, text="▼", width=4, state="disabled")
        self.mback_btn.grid(row=2, column=2, padx=4, pady=(1, 3))
        self.mback_btn.bind("<ButtonPress-1>", lambda e: self._manual_press("MBACK"))
        self.mback_btn.bind("<ButtonRelease-1>", lambda e: self._manual_release())

        # Live gyro dial - sits beside the d-pad (which is only ~3 rows tall)
        # so it's in view while driving without costing the 480px screen any
        # height. Redrawn on every HDG line from the Teensy (10 Hz).
        self.heading_canvas = tk.Canvas(manual_frame, width=HEADING_DIAL_SIZE,
                                         height=HEADING_DIAL_SIZE, bg=BG_INSET,
                                         highlightthickness=1, highlightbackground=ACCENT_DIM)
        self.heading_canvas.grid(row=0, column=4, rowspan=3, padx=(16, 8), pady=3)

        food_frame = ttk.LabelFrame(control_tab, text="◆ FOOD TRAY (IR SENSOR)")
        food_frame.pack(fill="x", padx=6, pady=2)
        self.food_label = ttk.Label(food_frame, text="TRAY: —", font=FONT_STATUS,
                                     foreground=FG_DIM)
        self.food_label.grid(row=0, column=0, padx=(8, 16), pady=4, sticky="w")
        ttk.Checkbutton(food_frame, text="Auto-return when food is taken",
                        variable=self.auto_return_enabled).grid(row=0, column=1, padx=4)
        ttk.Button(food_frame, text="↩ Return Now", style="Accent.TButton",
                   command=self._manual_return).grid(row=0, column=2, padx=8)

        # ===== TAB 2: Path (fake-autonomous scripted playback) =====
        # Uses grid for the tab's own top-level rows, and gives the step
        # list a FIXED height (scrollbar for overflow) rather than
        # expand=True: a Tcl/Tk quirk in some builds lets an expand=True
        # Listbox visually render past its own reported geometry, painting
        # over whatever sits below it even though the layout manager's own
        # bookkeeping is correct. A fixed height sidesteps that outright.
        # Row 3 is an empty spacer that absorbs any extra space instead.
        path_tab.columnconfigure(0, weight=1)
        path_tab.rowconfigure(3, weight=1)

        builder_frame = ttk.LabelFrame(path_tab, text="◆ ADD STEP")
        builder_frame.grid(row=0, column=0, sticky="ew", padx=6, pady=(3, 1))

        self.path_action_var = tk.StringVar(value="FORWARD")
        path_action_combo = ttk.Combobox(
            builder_frame, textvariable=self.path_action_var,
            values=["FORWARD", "BACK", "LEFT", "RIGHT", "HOLD", "TURN LEFT", "TURN RIGHT",
                    ALIGN_ACTION],
            state="readonly", width=14)
        path_action_combo.grid(row=0, column=0, padx=6, pady=3)
        path_action_combo.bind("<<ComboboxSelected>>", self._on_path_action_changed)
        ttk.Label(builder_frame, text="for").grid(row=0, column=1)
        self.path_duration_var = tk.StringVar(value="3")
        # readonly: the arrows still work, but tapping the box can't put a text
        # cursor in it - which is what pops the touchscreen's on-screen keyboard
        # up over the dashboard. Naming a saved path is the only place typing is
        # actually wanted, and that dialog still gets the keyboard.
        self.path_duration_spin = ttk.Spinbox(builder_frame, from_=0.5, to=120, increment=0.5,
                                               width=6, textvariable=self.path_duration_var,
                                               state="readonly")
        self.path_duration_spin.grid(row=0, column=2, padx=6)
        self.path_unit_label = ttk.Label(builder_frame, text="sec")
        self.path_unit_label.grid(row=0, column=3)
        ttk.Button(builder_frame, text="+ Add Step", command=self._add_path_step).grid(
            row=0, column=4, padx=(16, 6))
        self.record_btn = self._neon_button(
            builder_frame, text="● Record", fg=WARNING, command=self._toggle_recording)
        self.record_btn.grid(row=0, column=5, padx=(6, 6))

        list_frame = ttk.LabelFrame(path_tab, text="◆ PATH STEPS")
        list_frame.grid(row=1, column=0, sticky="ew", padx=6, pady=1)
        list_frame.columnconfigure(0, weight=1)
        self.path_listbox = tk.Listbox(list_frame, height=6, bg=BG_INSET, fg=FG_TEXT,
                                        selectbackground=ACCENT_DIM, selectforeground=ACCENT,
                                        font=FONT_MONO, relief="flat", highlightthickness=1,
                                        highlightbackground=ACCENT_DIM, highlightcolor=ACCENT)
        self.path_listbox.grid(row=0, column=0, sticky="ew", padx=(6, 0), pady=3)
        path_scrollbar = ttk.Scrollbar(list_frame, orient="vertical",
                                        command=self.path_listbox.yview)
        path_scrollbar.grid(row=0, column=1, sticky="ns", padx=(0, 6), pady=3)
        self.path_listbox.config(yscrollcommand=path_scrollbar.set)

        # All buttons clustered together, left-aligned, in one narrow group
        # (not split-justified across the row) - all ttk.Button, not
        # _neon_button/tk.Button, see the Success/Danger/Accent.TButton
        # styles above for why.
        path_btn_row = ttk.Frame(path_tab)
        path_btn_row.grid(row=2, column=0, sticky="w", padx=6, pady=(1, 1))
        ttk.Button(path_btn_row, text="Remove", command=self._remove_path_step).grid(
            row=0, column=0, padx=(0, 4))
        ttk.Button(path_btn_row, text="Clear", command=self._clear_path_steps).grid(
            row=0, column=1, padx=4)
        ttk.Button(path_btn_row, text="▶ RUN", style="Success.TButton",
                   command=self._run_path).grid(row=0, column=2, padx=4)
        ttk.Button(path_btn_row, text="■ STOP", style="Danger.TButton",
                   command=self._send_stop).grid(row=0, column=3, padx=(4, 0))

        # ===== TAB 3: Saved path library =====
        # Same fixed-height-listbox-plus-spacer reasoning as the PATH tab.
        saved_tab.columnconfigure(0, weight=1)
        saved_tab.rowconfigure(2, weight=1)

        saved_list_frame = ttk.LabelFrame(saved_tab, text="◆ SAVED PATHS")
        saved_list_frame.grid(row=0, column=0, sticky="ew", padx=6, pady=(4, 1))
        saved_list_frame.columnconfigure(0, weight=1)
        self.saved_listbox = tk.Listbox(saved_list_frame, height=6, bg=BG_INSET, fg=FG_TEXT,
                                         selectbackground=ACCENT_DIM, selectforeground=ACCENT,
                                         font=FONT_MONO, relief="flat", highlightthickness=1,
                                         highlightbackground=ACCENT_DIM, highlightcolor=ACCENT)
        self.saved_listbox.grid(row=0, column=0, sticky="ew", padx=(6, 0), pady=3)
        saved_scrollbar = ttk.Scrollbar(saved_list_frame, orient="vertical",
                                         command=self.saved_listbox.yview)
        saved_scrollbar.grid(row=0, column=1, sticky="ns", padx=(0, 6), pady=3)
        self.saved_listbox.config(yscrollcommand=saved_scrollbar.set)
        self._refresh_saved_listbox()

        # Clustered left-aligned, same reasoning as path_btn_row above.
        saved_btn_row = ttk.Frame(saved_tab)
        saved_btn_row.grid(row=1, column=0, sticky="w", padx=6, pady=(1, 1))
        ttk.Button(saved_btn_row, text="Delete", command=self._delete_selected_saved_path).grid(
            row=0, column=0, padx=(0, 4))
        ttk.Button(saved_btn_row, text="Load",
                   command=self._load_selected_saved_path).grid(row=0, column=1, padx=4)
        ttk.Button(saved_btn_row, text="✎ Save As", style="Accent.TButton",
                   command=self._save_current_path_as).grid(row=0, column=2, padx=4)
        ttk.Button(saved_btn_row, text="▶ Run", style="Success.TButton",
                   command=self._run_selected_saved_path).grid(row=0, column=3, padx=(4, 0))

        # ===== TAB 4: LiDAR =====
        lidar_conn_frame = ttk.LabelFrame(lidar_tab, text="◆ LIDAR LINK (RPLIDAR C1)")
        lidar_conn_frame.pack(fill="x", padx=6, pady=(6, 4))

        self.lidar_port_var = tk.StringVar()
        self.lidar_port_combo = ttk.Combobox(lidar_conn_frame, textvariable=self.lidar_port_var,
                                              width=14, state="readonly")
        self.lidar_port_combo.grid(row=0, column=0, padx=6, pady=6)
        ttk.Button(lidar_conn_frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=1, padx=4)
        self.lidar_connect_btn = ttk.Button(lidar_conn_frame, text="Connect LiDAR",
                                             command=self._toggle_lidar_connect)
        self.lidar_connect_btn.grid(row=0, column=2, padx=6)

        ttk.Label(lidar_conn_frame, text="Obstacle (mm):").grid(row=0, column=3, padx=(12, 4))
        ttk.Spinbox(lidar_conn_frame, from_=100, to=4000, increment=50, width=6,
                    textvariable=self.obstacle_threshold_mm,
                    state="readonly").grid(row=0, column=4, padx=4)  # see path_duration_spin

        status_row = ttk.Frame(lidar_tab)
        status_row.pack(fill="x", padx=6)
        self.lidar_status_label = ttk.Label(status_row, text="● LIDAR: DISCONNECTED", foreground=DANGER,
                                             font=FONT_STATUS, background=BG_MAIN)
        self.lidar_status_label.pack(side="left", pady=(2, 2))

        self.obstacle_label = ttk.Label(status_row, text="✓ PATH CLEAR", foreground=SUCCESS,
                                         font=("Consolas", 11, "bold"), background=BG_MAIN)
        self.obstacle_label.pack(side="right", pady=(2, 2))

        map_frame = ttk.LabelFrame(lidar_tab, text="◆ MAP (FRONT 180° SHADED)")
        map_frame.pack(fill="both", expand=True, padx=6, pady=4)
        self.canvas_size = 400  # updated live to match the map_frame's actual size
        self.canvas = tk.Canvas(map_frame, width=self.canvas_size, height=self.canvas_size,
                                 bg=BG_INSET, highlightthickness=1, highlightbackground=ACCENT_DIM)
        self.canvas.pack(fill="both", expand=True, padx=6, pady=6)
        self.canvas.bind("<Configure>", self._on_map_canvas_resize)

        # ===== TAB 5: Log =====
        dir_row = ttk.Frame(log_tab, style="Panel.TFrame")
        dir_row.pack(fill="x", padx=6, pady=(6, 2))
        self.dir_canvas = tk.Canvas(dir_row, width=44, height=44, bg=BG_INSET,
                                     highlightthickness=1, highlightbackground=ACCENT_DIM)
        self.dir_canvas.pack(side="left")
        self.dir_label = ttk.Label(dir_row, text="—  IDLE", font=("Consolas", 11, "bold"),
                                    foreground=FG_DIM, background=BG_PANEL)
        self.dir_label.pack(side="left", padx=10)
        # Lives here rather than in the ROBOT LINK row (no space left there),
        # and this is where its output lands anyway.
        ttk.Button(dir_row, text="Pin test", style="Small.TButton",
                   command=self._pin_test).pack(side="right", padx=6)

        self.log_text = tk.Text(log_tab, height=10, state="disabled", wrap="word",
                                 bg=BG_INSET, fg=FG_TEXT, insertbackground=ACCENT,
                                 font=("Consolas", 9), relief="flat", highlightthickness=1,
                                 highlightbackground=ACCENT_DIM, highlightcolor=ACCENT,
                                 padx=6, pady=4)
        self.log_text.pack(fill="both", expand=True, padx=6, pady=(2, 6))

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
            self._send("FOOD")  # tray state now, rather than waiting for the next pickup
        except Exception as e:
            self._log(f"Connection failed: {e}")

    def _disconnect(self):
        self.reader_running = False
        self._cancel_return_job()
        self.food_present = None
        self.awaiting_pickup = False
        self.return_active = False
        self._update_food_label()
        self.heading_deg = None
        self._draw_heading_dial()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.conn_status.config(text="● DISCONNECTED", foreground=DANGER)
        self.connect_btn.config(text="Connect")
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
        # High-rate telemetry first, and kept out of the log: HDG arrives 10x a
        # second and would bury everything else.
        if line.startswith("HDG:"):
            self._on_heading(line)
            return
        if line.startswith("MTURN:"):
            self._on_manual_turn(line)
            return

        if line.startswith("OK:PATH_STARTED"):
            self.path_active = True
            self._update_status()
        elif line == "PATH:DONE":
            self.path_active = False
            self._update_status()
            self._on_path_finished()
        elif line == "OK:PATH_STOPPED":
            # Cancelled, not completed - nothing was delivered, so don't arm
            # a pickup (and abandon a return trip that was cut short).
            self.path_active = False
            self.awaiting_pickup = False
            self.return_active = False
            self._update_status()
        elif line.startswith("ALIGN:"):
            # Residual heading error once the ALIGN step finished. Near 0 means
            # the robot really is back on its start heading; a large value
            # means it timed out (gyro missing, or something blocked the spin).
            try:
                residual = float(line.split(":", 1)[1])
            except ValueError:
                pass
            else:
                if abs(residual) <= ALIGN_OK_RESIDUAL_DEG:
                    self._log(f"Aligned to the start heading ({residual:+.1f}° off).")
                else:
                    self._log(f"⚠ ALIGN gave up {residual:+.1f}° off the start heading - "
                              f"check the gyro (send GYRO) or whether the robot could turn.")
        elif line in ("FOOD:PRESENT", "FOOD:TAKEN", "FOOD:ABSENT"):
            self._on_food_event(line)
        elif line == "READY":
            self._send("FOOD")  # resync the tray state after a Teensy reset

        self._log(f"Robot: {line}")

    def _send_stop(self):
        self._stop_recording()
        self._cancel_return_job()  # a pending "thank you -> drive home" must not fire after a STOP
        self._send("STOP")
        self.auto_paused = False
        self.manual_mode = False
        self.path_active = False
        self.awaiting_pickup = False
        self.return_active = False
        self._update_status()
        self._update_food_label()

    def _send(self, cmd):
        if self.recording and cmd in MANUAL_COMMANDS:
            self._record_manual_cmd(cmd)
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
        self._cancel_return_job()
        self._send("MANUAL")
        self.manual_mode = True
        self.auto_paused = False
        self.path_active = False
        self.return_active = False
        self.awaiting_pickup = False
        self.obstacle_pause_since = None
        self._clear_alert()
        self._update_status()
        self._update_food_label()
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

    # ---------------- Record-by-driving (manual drive -> PATH steps) ----------------
    def _toggle_recording(self):
        if self.recording:
            self._stop_recording()
            return
        if not self.manual_mode:
            self._log("Take manual control first, then start recording.")
            return
        self.path_steps.clear()
        self._refresh_path_listbox()
        self.recording = True
        self._record_pending_turns = []
        self._record_active_cmd = None
        self._record_start_time = None
        self._record_idle_since = time.time()
        self.record_btn.config(text="■ Stop & Save", fg=DANGER)
        self._log("Recording started - drive the robot, then press Stop & Save.")

    def _stop_recording(self):
        if not self.recording:
            return
        self.recording = False
        if self._record_active_cmd is not None:
            self._finish_record_segment(time.time())
        self._record_idle_since = None
        self.record_btn.config(text="● Record", fg=WARNING)
        self._log(f"Recording stopped - {len(self.path_steps)} step(s) captured. "
                  f"Save it from the SAVED tab, or Run Path to try it now.")

    def _record_manual_cmd(self, cmd):
        now = time.time()
        if cmd == "MSTOP":
            if self._record_active_cmd is not None:
                self._finish_record_segment(now)
            self._record_idle_since = now
            return

        # A new direction: close out any idle gap since the last release as
        # a HOLD step, and (if a different direction was already active
        # with no MSTOP in between) close that segment out too.
        if self._record_idle_since is not None:
            gap = now - self._record_idle_since
            if gap >= MIN_RECORDED_GAP_SECONDS:
                self.path_steps.append(("HOLD", round(gap, 1)))
                self._refresh_path_listbox()
            self._record_idle_since = None
        if self._record_active_cmd is not None and self._record_active_cmd != cmd:
            self._finish_record_segment(now)
        if self._record_active_cmd != cmd:
            self._record_active_cmd = cmd
            self._record_start_time = now

    def _finish_record_segment(self, end_time):
        duration = end_time - self._record_start_time
        action = MANUAL_CMD_TO_PATH_ACTION[self._record_active_cmd]
        index = None
        if duration >= 0.05:
            self.path_steps.append((action, round(duration, 1)))
            index = len(self.path_steps) - 1
            self._refresh_path_listbox()
        if action in TIMED_TO_GYRO_TURN:
            # The Teensy reports every manual turn, even one too short to keep
            # here, so a dropped one still gets an entry (index None) - its
            # report must land on it, not on the next real turn.
            self._record_pending_turns.append((index, action, time.time()))
        self._record_active_cmd = None
        self._record_start_time = None

    def _on_manual_turn(self, line):
        """MTURN:<L|R>,<deg> - the Teensy's gyro measurement of a manual turn
        that just ended. If it's a turn we recorded, swap the timed LEFT/RIGHT
        step for a TURN LEFT/RIGHT of the measured angle, so playback turns by
        the gyro (closed-loop, same angle every time) instead of by the clock
        (whatever battery level and floor grip make of 1.3 seconds).

        Forward/back steps stay timed: the gyro measures rotation, not
        distance, so there's nothing better to replace them with.
        """
        try:
            direction, deg_txt = line[len("MTURN:"):].split(",")
            deg = float(deg_txt)
        except ValueError:
            return
        action = {"L": "LEFT", "R": "RIGHT"}.get(direction)

        # The report describes the turn that *just* ended - the newest entry.
        # Reports land within milliseconds of the step being recorded, so an
        # entry older than the window lost its report (the dead-man's switch
        # ended that turn before the dashboard did) and is dropped rather than
        # left to soak up some later turn's angle.
        now = time.time()
        self._record_pending_turns = [p for p in self._record_pending_turns
                                      if now - p[2] <= MTURN_MATCH_WINDOW_S]
        if not self._record_pending_turns:
            return  # not a turn we recorded (recording off, or already matched)
        index, pending_action, _ = self._record_pending_turns.pop()
        if index is None or pending_action != action:
            return
        if index >= len(self.path_steps) or self.path_steps[index][0] != action:
            return  # the step list was edited or cleared in the meantime
        if deg < MIN_RECORDED_TURN_DEG:
            return  # a nudge too small to be worth a gyro turn - keep it timed
        gyro_action = TIMED_TO_GYRO_TURN[action]
        self.path_steps[index] = (gyro_action, int(round(deg)))
        self._refresh_path_listbox()
        self._log(f"Recorded turn measured by gyro: {gyro_action} {deg:.0f}°")

    # ---------------- Path tab (fake-autonomous scripted playback) ----------------
    def _on_path_action_changed(self, event=None):
        action = self.path_action_var.get()
        if action == ALIGN_ACTION:
            # The value is an offset from the anchored start heading, so 0 -
            # "face exactly the way this delivery began" - is both the useful
            # default and a legal value, unlike a turn of 0 degrees.
            self.path_unit_label.config(text="deg off start")
            self.path_duration_spin.config(from_=0, to=359, increment=15)
            self.path_duration_var.set("0")
        elif action in DEGREE_ACTIONS:
            self.path_unit_label.config(text="deg")
            self.path_duration_spin.config(from_=1, to=360, increment=5)
            self.path_duration_var.set("90")
        else:
            self.path_unit_label.config(text="sec")
            self.path_duration_spin.config(from_=0.5, to=120, increment=0.5)
            self.path_duration_var.set("3")

    def _add_path_step(self):
        action = self.path_action_var.get()
        try:
            value = float(self.path_duration_var.get())
        except ValueError:
            self._log("Invalid step value.")
            return
        if value < 0 or (value == 0 and action != ALIGN_ACTION):
            # 0 is meaningless as a duration or a turn, but it's the normal
            # case for ALIGN: no offset from the start heading.
            self._log("Step value must be greater than 0.")
            return
        self.path_steps.append((action, value))
        self._refresh_path_listbox()

    def _remove_path_step(self):
        sel = self.path_listbox.curselection()
        if not sel:
            return
        del self.path_steps[sel[0]]
        self._refresh_path_listbox()

    def _clear_path_steps(self):
        self.path_steps.clear()
        self._refresh_path_listbox()

    def _refresh_path_listbox(self):
        self.path_listbox.delete(0, "end")
        for i, (action, value) in enumerate(self.path_steps, start=1):
            if action == ALIGN_ACTION:
                label = f"{action}  —  start heading{f' +{value:g}°' if value else ''}"
            else:
                label = f"{action}  —  {value:g}{'°' if action in DEGREE_ACTIONS else 's'}"
            self.path_listbox.insert("end", f"{i}. {label}")

    def _run_path(self, is_return=False):
        """Send the PATH tab's steps to the robot.

        is_return marks the retrace built by build_return_path(), so finishing
        it ends the delivery cycle instead of arming another pickup. Every
        other caller (buttons, saved paths, the phone page) is an outbound
        delivery and snapshots its steps for that retrace.
        """
        if not (self.ser and self.ser.is_open):
            self._log("Connect to the robot first.")
            return
        if not self.path_steps:
            self._log("Path is empty - add at least one step first.")
            return
        if len(self.path_steps) > PATH_MAX_STEPS:
            self._log(f"Path has {len(self.path_steps)} steps - the firmware only accepts "
                      f"{PATH_MAX_STEPS}. Shorten it before running.")
            return
        body = ";".join(
            f"{PATH_ACTION_TOKENS[action]},"
            f"{int(round(value)) if action in DEGREE_ACTIONS else int(round(value * 1000))}"
            for action, value in self.path_steps
        )
        self._cancel_return_job()
        self.return_active = is_return
        if not is_return:
            self.delivery_steps = list(self.path_steps)
            self.awaiting_pickup = False
            # Latch the heading the robot is leaving on, before it moves. The
            # return trip's closing ALIGN step steers back to exactly this,
            # across both PATH commands and the wait at the table.
            self._send("HEADING_ANCHOR")
        self.manual_mode = False
        self.auto_paused = False
        self._send(f"PATH:{body}")
        self._update_status()

    # ---------------- Food tray + automatic return trip ----------------
    def _on_food_event(self, line):
        """FOOD:PRESENT / FOOD:TAKEN / FOOD:ABSENT from the Teensy's IR sensor.

        FOOD:TAKEN is the only pickup event - FOOD:ABSENT is just the tray
        state reported on connect/boot, which must never launch a return trip.
        """
        self.food_present = (line == "FOOD:PRESENT")
        self._update_food_label()
        if line != "FOOD:TAKEN":
            return

        if not self.awaiting_pickup:
            self._log("Food taken, but no delivery is waiting for pickup - staying put.")
            return
        if not self.auto_return_enabled.get():
            self._log("Food taken - auto-return is off, use ↩ Return Now to send the robot back.")
            return
        self._start_return_trip()

    def _on_path_finished(self):
        if self.return_active:
            self.return_active = False
            self.awaiting_pickup = False
            self._log("Return trip complete - robot is back at the start, on its original heading.")
            self.speech.speak(RETURN_DONE_VOICE_MSG)
            return
        if not self.delivery_steps:
            return
        self.awaiting_pickup = True
        self._log("Delivery complete - waiting for the customer to take the food.")
        self._update_food_label()

    def _manual_return(self):
        """↩ Return Now - send the robot home without waiting for the IR sensor
        (auto-return switched off, a sensor that never fired, or a test run)."""
        if not self.delivery_steps and not self.path_steps:
            self._log("No delivery path to retrace - run or build a path first.")
            return
        self._start_return_trip()

    def _start_return_trip(self):
        self.awaiting_pickup = False
        self._log(f"Voice: \"{FOOD_TAKEN_VOICE_MSG}\"")
        self.speech.speak(FOOD_TAKEN_VOICE_MSG)
        self._log(f"Returning in {RETURN_DELAY_SECONDS}s...")
        self._cancel_return_job()
        self._return_job = self.root.after(int(RETURN_DELAY_SECONDS * 1000),
                                           self._send_return_path)

    def _send_return_path(self):
        self._return_job = None
        source = self.delivery_steps or self.path_steps
        if not source:
            self._log("No delivery path to retrace.")
            return
        steps = build_return_path(source)
        if len(steps) > PATH_MAX_STEPS:
            self._log(f"Return path needs {len(steps)} steps but the firmware only accepts "
                      f"{PATH_MAX_STEPS} - drive the robot back manually.")
            return
        # Load it into the PATH tab so the retrace is visible while it runs.
        self.path_steps = steps
        self._refresh_path_listbox()
        self._log(f"Running return path ({len(steps)} steps: U-turn, {len(source)} retraced, U-turn).")
        self._run_path(is_return=True)

    def _cancel_return_job(self):
        if self._return_job is not None:
            self.root.after_cancel(self._return_job)
            self._return_job = None

    def _update_food_label(self):
        if self.food_present is None:
            text, color = "TRAY: — (no reading yet)", FG_DIM
        elif self.food_present:
            text, color = "TRAY: ● FOOD LOADED", SUCCESS
        else:
            text, color = "TRAY: ○ EMPTY", FG_DIM
        if self.awaiting_pickup:
            text += "  —  WAITING FOR PICKUP"
            color = WARNING
        self.food_label.config(text=text, foreground=color)

    # ---------------- Saved path library ----------------
    def _load_saved_paths(self):
        try:
            with open(SAVED_PATHS_FILE, "r", encoding="utf-8") as f:
                raw = json.load(f)
            self.saved_paths = {
                name: [(action, seconds) for action, seconds in steps]
                for name, steps in raw.items()
            }
        except FileNotFoundError:
            self.saved_paths = {}
        except (json.JSONDecodeError, OSError, ValueError) as e:
            self.saved_paths = {}
            self.root.after(0, self._log, f"Could not load {SAVED_PATHS_FILE}: {e}")

    def _write_saved_paths(self):
        try:
            with open(SAVED_PATHS_FILE, "w", encoding="utf-8") as f:
                json.dump(self.saved_paths, f, indent=2)
        except OSError as e:
            self._log(f"Could not save path library: {e}")

    def _refresh_saved_listbox(self):
        self.saved_listbox.delete(0, "end")
        for name in sorted(self.saved_paths):
            steps = self.saved_paths[name]
            total_s = sum(value for action, value in steps if action not in DEGREE_ACTIONS)
            turn_count = sum(1 for action, _value in steps if action in DEGREE_ACTIONS)
            summary = f"{len(steps)} steps, {total_s:g}s"
            if turn_count:
                summary += f", {turn_count} turn(s)"
            self.saved_listbox.insert("end", f"{name}  ({summary})")

    def _selected_saved_name(self):
        sel = self.saved_listbox.curselection()
        if not sel:
            self._log("Select a saved path first.")
            return None
        return sorted(self.saved_paths)[sel[0]]

    def _save_current_path_as(self):
        if not self.path_steps:
            self._log("Nothing to save - add steps or record a drive first.")
            return
        name = self._ask(simpledialog.askstring, "Save Path", "Name this path:")
        if not name:
            return
        name = name.strip()
        if not name:
            return
        self.saved_paths[name] = list(self.path_steps)
        self._write_saved_paths()
        self._refresh_saved_listbox()
        self._log(f"Saved path '{name}' ({len(self.path_steps)} steps).")

    def _load_selected_saved_path(self):
        name = self._selected_saved_name()
        if name is None:
            return
        self.path_steps = list(self.saved_paths[name])
        self._refresh_path_listbox()
        self._log(f"Loaded '{name}' into the path editor.")

    def _run_selected_saved_path(self):
        name = self._selected_saved_name()
        if name is None:
            return
        self.path_steps = list(self.saved_paths[name])
        self._refresh_path_listbox()
        self._log(f"Running saved path '{name}'.")
        self._run_path()

    def _delete_selected_saved_path(self):
        name = self._selected_saved_name()
        if name is None:
            return
        del self.saved_paths[name]
        self._write_saved_paths()
        self._refresh_saved_listbox()
        self._log(f"Deleted saved path '{name}'.")

    def _update_status(self):
        if self.manual_mode:
            state_txt, color = "MANUAL CONTROL", ACCENT2
        elif self.auto_paused:
            state_txt, color = "WAITING (OBSTACLE)", WARNING
        elif self.path_active:
            state_txt, color = "PATH RUNNING", ACCENT2
        else:
            state_txt, color = "STOPPED", ACCENT
        self.status_label.config(text=f"STATE: {state_txt}", foreground=color)
        if not self.manual_mode:
            for btn in (self.mfwd_btn, self.mback_btn, self.mleft_btn, self.mright_btn, self.mstop_btn):
                btn.config(state="disabled")

    # ---------------- Stall alert (obstacle-blocked >10s) ----------------
    def _check_stall_alert(self):
        now = time.time()
        stalled = (self.obstacle_pause_since is not None
                   and (now - self.obstacle_pause_since) > STALL_ALERT_SECONDS)

        if stalled and not self.alert_active:
            self.alert_active = True
            self._log(f"⚠ ALERT: robot has been blocked by an obstacle for over "
                      f"{STALL_ALERT_SECONDS}s — take manual control from your phone.")
            self.speech.speak(STALL_VOICE_MSG)
            self.alert_label.config(text="⚠ NEEDS HELP: BLOCKED — USE PHONE/MANUAL", foreground=DANGER)
        elif not stalled and self.alert_active:
            self._clear_alert()

    def _clear_alert(self):
        if self.alert_active:
            self._log("Alert cleared.")
        self.alert_active = False
        self.alert_label.config(text="")

    def _detect_direction(self, msg):
        """Infer a heading from a log line (our own 'Sent: ...' commands, or
        anything the robot echoes back that mentions a direction word)."""
        m = msg.upper()
        if "LEFT" in m:
            return "left"
        if "RIGHT" in m:
            return "right"
        if "STOP" in m:
            return "stop"
        if "BACK" in m or "REVERSE" in m:
            return "back"
        if "FORWARD" in m or "FWD" in m:
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

    # ---------------- Live gyro dial ----------------
    def _on_heading(self, line):
        """HDG:<heading>,<gyroRightSign>,<anchor> - the Teensy's 10 Hz stream."""
        try:
            heading, sign, anchor = line[len("HDG:"):].split(",")
            self.heading_deg = float(heading)
            self.gyro_right_sign = int(sign)
            self.anchor_deg = float(anchor)
        except ValueError:
            return
        self._last_hdg_time = time.time()
        self._draw_heading_dial()

    def _ask(self, dialog, *args, **kwargs):
        """Show a dialog with fullscreen dropped for the duration.

        A fullscreen window can cover the touchscreen's on-screen keyboard,
        and on some window managers the dialog itself opens *behind* it - a
        modal nobody can see looks like a frozen app.
        """
        was_fullscreen = self.fullscreen
        if was_fullscreen:
            self._set_fullscreen(False)
        try:
            return dialog(*args, parent=self.root, **kwargs)
        finally:
            if was_fullscreen:
                self._set_fullscreen(True)

    def _motor_test(self):
        """Drive each motor directly on the Teensy, bypassing manual mode, the
        dead-man's switch and path playback - so "nothing moves" can be pinned
        on the wiring/driver rather than the command path."""
        if not (self.ser and self.ser.is_open):
            self._log("Connect to the robot first.")
            return
        if not self._ask(messagebox.askokcancel, "Motor test",
                         "The robot will drive each motor forwards and backwards "
                         "for about 3 seconds.\n\nLift the wheels off the ground "
                         "first - STOP cannot interrupt this test."):
            return
        self._log("--- Motor test: each side, each way, ~0.6s per burst ---")
        self._send("MTEST")
        self.notebook.select(self.log_tab)

    def _pin_test(self):
        """Hold each motor-signal pin high in turn, to check with a multimeter
        that the Teensy's outputs still work - what a lost common ground
        damages."""
        if not (self.ser and self.ser.is_open):
            self._log("Connect to the robot first.")
            return
        if not self._ask(messagebox.askokcancel, "Pin test",
                         "Unplug the four signal wires (PWM/DIR) from the motor "
                         "driver first.\n\nEach pin is then held HIGH for 3 seconds: "
                         "measure it against the Teensy's GND - a healthy pin reads "
                         "3.3V, a damaged one reads near 0V."):
            return
        self._log("--- Pin test: 3s per pin, expect 3.3V on each against Teensy GND ---")
        self._send("PINTEST")
        self.notebook.select(self.log_tab)

    def _gyro_test(self):
        """Ask the Teensy what it can see on the I2C bus, and show the answer.

        There's no keyboard on the robot, so this is the only way to run the
        GYRO/I2CSCAN diagnostics. Jumps to the LOG tab because that's where
        the replies land.
        """
        if not (self.ser and self.ser.is_open):
            self._log("Connect to the robot first.")
            return
        self._log("--- Gyro test: GYRO expects OK; I2C expects a device at 0x68 or 0x69 ---")
        if self.heading_deg is None:
            self._log("No HDG stream: the Teensy isn't reporting a heading (gyro not detected).")
        else:
            self._log(f"HDG stream is live - heading {self.heading_deg:+.1f}°, "
                      f"{'direction learned' if self.gyro_right_sign else 'turn once to calibrate'}.")
        self._send("GYRO")
        self._send("I2CSCAN")
        self.notebook.select(self.log_tab)

    def _heading_from_start(self):
        """Degrees turned from START (the anchor), right turns positive, or
        None without live gyro data. The raw heading's sign depends on how the
        MPU6050 is mounted; the Teensy's learned gyroRightSign undoes that."""
        heading = self.heading_deg  # one read - the phone API calls this off the Tk thread
        if heading is None:
            return None
        mirror = self.gyro_right_sign if self.gyro_right_sign else 1
        return normalize_deg180((heading - self.anchor_deg) * mirror)

    def _check_heading_stale(self):
        # The stream stops if the gyro drops out or the link does - say so on
        # the dial rather than freeze on the last heading as if it were live.
        if (self.heading_deg is not None and self._last_hdg_time is not None
                and time.time() - self._last_hdg_time > HDG_STALE_SECONDS):
            self.heading_deg = None
            self._draw_heading_dial()

    def _draw_heading_dial(self):
        """Top-down view of the robot relative to the heading it started on.

        "START" at the top is the anchor (the heading latched when the last
        delivery set off - or power-on, before any). The arrow is the robot
        now, the shaded wedge how far it has turned from START. A right turn
        rotates the arrow clockwise, which needs the gyro's mounting sign -
        until the Teensy has learned it from a first turn, the drawing may be
        mirrored and the dial says so.
        """
        c = self.heading_canvas
        c.delete("all")
        size = HEADING_DIAL_SIZE
        # Ring between the START label above and two text lines below.
        ring_top, text_band = 16, 26
        r = (size - ring_top - text_band) / 2
        cx, cy = size / 2, ring_top + r

        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=ACCENT_DIM, width=2)
        for tick in range(0, 360, 45):
            a = math.radians(tick - 90)
            inner = r - (6 if tick % 90 == 0 else 3)
            c.create_line(cx + inner * math.cos(a), cy + inner * math.sin(a),
                          cx + r * math.cos(a), cy + r * math.sin(a), fill=ACCENT_DIM)
        c.create_text(cx, 6, text="START", fill=FG_DIM, font=("Consolas", 7, "bold"))
        c.create_polygon(cx, ring_top + 1, cx - 4, ring_top - 5, cx + 4, ring_top - 5,
                         fill=SUCCESS, outline="")

        if self.heading_deg is None:
            c.create_text(cx, cy, text="NO GYRO\nDATA", fill=FG_DIM, justify="center",
                          font=("Consolas", 8, "bold"))
            c.create_text(cx, size - 8, text="send GYRO to check", fill=FG_DIM,
                          font=("Consolas", 6))
            return

        rel = self._heading_from_start()
        off_start = abs(rel) > 5

        if off_start:
            # Only once visibly off START: on Windows a pie slice only a
            # degree or so wide - edge points landing on the same pixel -
            # is drawn by GDI as the *whole* disc.
            # Tk arcs: 0 deg = 3 o'clock, counter-clockwise positive.
            c.create_arc(cx - r + 4, cy - r + 4, cx + r - 4, cy + r - 4, start=90, extent=-rel,
                         fill=MAP_SECTOR_FILL, outline=ACCENT_DIM, style="pieslice")

        a = math.radians(rel)
        arrow = [(0, -r + 6), (-8, 9), (0, 3), (8, 9)]  # pointing up before rotation
        pts = []
        for px, py in arrow:
            pts.extend((cx + px * math.cos(a) - py * math.sin(a),
                        cy + px * math.sin(a) + py * math.cos(a)))
        c.create_polygon(pts, fill=ACCENT, outline="")

        c.create_text(cx, size - 17, text=f"{rel:+.0f}°", font=("Consolas", 9, "bold"),
                      fill=WARNING if off_start else SUCCESS)
        c.create_text(cx, size - 6, font=("Consolas", 6),
                      text="gyro OK" if self.gyro_right_sign else "turn to calibrate",
                      fill=FG_DIM)

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
        self._check_heading_stale()
        self.root.after(150, self._refresh_map)  # ~6-7 Hz refresh

    def _draw_map_and_check_obstacle(self):
        c = self.canvas
        c.delete("all")
        cx, cy = c.winfo_width() // 2, c.winfo_height() // 2
        max_r = self.canvas_size // 2 - 10
        scale = max_r / MAP_MAX_RANGE_MM

        # Shade the front-180 sector (heading 0 = straight up on screen)
        c.create_arc(cx - max_r, cy - max_r, cx + max_r, cy + max_r,
                     start=0, extent=180, fill=MAP_SECTOR_FILL, outline="", style="pieslice")
        # Crosshair spokes
        for ang in (0, 45, 90, 135, 180, 225, 270, 315):
            theta = math.radians(ang - 90)
            c.create_line(cx, cy, cx + max_r * math.cos(theta), cy + max_r * math.sin(theta),
                          fill=MAP_SPOKE_COLOR)
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
        self.obstacle_dist_mm = obstacle_dist
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
            if self.path_active and not self.auto_paused:
                self._log(f"Obstacle detected at {obstacle_dist} mm in front 180 -> PATH_PAUSE")
                self._send("PATH_PAUSE")
                self.auto_paused = True
                self.obstacle_pause_since = time.time()
                self._update_status()
        else:
            self.obstacle_label.config(text="✓ PATH CLEAR", foreground=SUCCESS)
            if self.auto_paused:
                self._log("Obstacle cleared -> PATH_RESUME")
                self._send("PATH_RESUME")
                self.auto_paused = False
                self.obstacle_pause_since = None
                self._clear_alert()
                self._update_status()


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotDashboard(root)
    root.mainloop()