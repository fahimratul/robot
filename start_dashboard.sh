#!/usr/bin/env bash
# Launch the robot dashboard on the Pi's 7" touchscreen.
#
#   ./start_dashboard.sh      run it now (to test)
#   ./install_autostart.sh    run it at every boot (once, see that script)
#
# Everything resolves relative to this script's own folder, so it doesn't
# care what the working directory is when the desktop session launches it.
# Output goes to dashboard.log next to this script - the desktop session
# has nowhere to show a terminal, so a crash would otherwise vanish.

set -u

ROOT="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
LOG="$ROOT/dashboard.log"

STARTUP_DELAY="${DASHBOARD_STARTUP_DELAY:-8}"  # seconds to wait for the desktop + USB
RESTART_DELAY=5
FAST_FAILURE_SECONDS=15   # a run shorter than this counts as "crashed on startup"
MAX_FAST_FAILURES=5       # that many in a row -> stop retrying, something is actually broken

# Keep the previous boot's log rather than growing one file forever.
[ -f "$LOG" ] && mv -f "$LOG" "$LOG.1"
exec >>"$LOG" 2>&1

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

log "=== start_dashboard.sh — $ROOT ==="

# dashbord.py is a Tkinter window: without a desktop session there is
# nothing to draw into and it would crash on import.
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    log "ERROR: no DISPLAY/WAYLAND_DISPLAY - the dashboard needs a desktop session."
    log "       If the Pi boots to a console, run 'sudo raspi-config' ->"
    log "       System Options -> Boot / Auto Login -> Desktop Autologin."
    exit 1
fi

if [ ! -f "$ROOT/dashbord.py" ]; then
    log "ERROR: dashbord.py not found in $ROOT"
    exit 1
fi

# The desktop session, the touchscreen, and USB serial enumeration (Teensy +
# RPLidar C1) all settle a few seconds after login - starting into that race
# comes up with an empty port list in the CONTROL/LIDAR tabs.
log "Waiting ${STARTUP_DELAY}s for the desktop and USB devices to settle..."
sleep "$STARTUP_DELAY"

# setup_pi.sh's venv holds pyserial/rplidarc1/pyttsx3; fall back to the
# system python3 if the dashboard was installed some other way.
if [ -x "$ROOT/venv/bin/python3" ]; then
    PYTHON="$ROOT/venv/bin/python3"
else
    PYTHON="$(command -v python3 || true)"
    log "No venv in $ROOT - falling back to system python3: ${PYTHON:-<not found>}"
fi
if [ -z "$PYTHON" ]; then
    log "ERROR: no python3 found. Run ./setup_pi.sh first."
    exit 1
fi

log "Serial ports visible right now: $(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')"

cd "$ROOT" || exit 1

# Restart on a crash (the rplidarc1 parser can take the process down on a bad
# packet), but never on a clean exit - closing the window should stay closed.
fast_failures=0
while true; do
    log "Starting: $PYTHON dashbord.py"
    started=$(date +%s)
    "$PYTHON" dashbord.py
    status=$?
    ran=$(( $(date +%s) - started ))

    if [ "$status" -eq 0 ]; then
        log "Dashboard closed normally after ${ran}s - not restarting."
        break
    fi
    if [ "$status" -ge 128 ]; then
        log "Killed by a signal (status $status) after ${ran}s - not restarting."
        break
    fi

    if [ "$ran" -lt "$FAST_FAILURE_SECONDS" ]; then
        fast_failures=$(( fast_failures + 1 ))
    else
        fast_failures=1
    fi

    if [ "$fast_failures" -ge "$MAX_FAST_FAILURES" ]; then
        log "Exited with status $status ${fast_failures} times in a row inside"
        log "${FAST_FAILURE_SECONDS}s - giving up. The traceback is above in this log."
        break
    fi

    log "Exited with status $status after ${ran}s - restarting in ${RESTART_DELAY}s"
    log "(startup failure ${fast_failures} of ${MAX_FAST_FAILURES})."
    sleep "$RESTART_DELAY"
done

log "=== start_dashboard.sh finished ==="
