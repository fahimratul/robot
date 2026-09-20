#!/usr/bin/env bash
# Compile lineflow.ino and upload it to the Teensy plugged into this Pi.
#
#   ./flash_teensy.sh                          build + upload (Teensy 4.1)
#   TEENSY_FQBN=teensy:avr:teensy40 ./flash_teensy.sh   a different board
#
# One-time prerequisite: ./setup_teensy_flash.sh
#
# Typical update on the robot:   git pull && ./flash_teensy.sh
set -euo pipefail

ROOT="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
export PATH="$HOME/.local/bin:$PATH"

# arduino-cli only compiles a sketch whose folder matches the .ino's name
# (lineflow/lineflow.ino). The repo keeps lineflow.ino at its root next to
# the dashboard, so build from a copy instead of restructuring the repo.
BUILD="$ROOT/.build"
SKETCH_DIR="$BUILD/lineflow"
OUT_DIR="$BUILD/out"

die() { echo "ERROR: $*" >&2; exit 1; }

for tool in arduino-cli teensy_loader_cli python3; do
    command -v "$tool" >/dev/null 2>&1 || die "$tool not found - run ./setup_teensy_flash.sh first."
done
[ -f "$ROOT/lineflow.ino" ] || die "lineflow.ino not found in $ROOT"

# ---- Where is the Teensy? ----
# Its USB-serial port has a stable name under by-id, so it can't be confused
# with the RPLidar (a CP210x on ttyUSB*) whatever order they enumerated in.
PORT_LINK="$(ls /dev/serial/by-id/usb-Teensyduino* 2>/dev/null | head -n 1 || true)"
PORT=""
[ -n "$PORT_LINK" ] && PORT="$(readlink -f "$PORT_LINK")"

# ---- Which Teensy? ----
# The robot's board is a Teensy 4.1. Still ask the connected board (the
# Teensy core's discovery reports the exact model) and believe it if it
# answers - flashing a 4.1 image onto anything else fails - but fall back to
# 4.1 when it can't say (blank, crashed, or sitting in the bootloader).
ROBOT_FQBN="teensy:avr:teensy41"
FQBN="${TEENSY_FQBN:-}"
if [ -z "$FQBN" ]; then
    DETECTED="$(arduino-cli board list --format json 2>/dev/null | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
except ValueError:
    sys.exit()
# arduino-cli 1.x wraps the list in {"detected_ports": [...]}, 0.x does not.
ports = data.get("detected_ports", []) if isinstance(data, dict) else data
for port in ports:
    for board in port.get("matching_boards") or []:
        if board.get("fqbn", "").startswith("teensy:"):
            print(board["fqbn"])
            sys.exit()
' || true)"
    if [ -z "$DETECTED" ]; then
        FQBN="$ROBOT_FQBN"
        echo "Couldn't ask the board its model - assuming the robot's Teensy 4.1."
    elif [ "$DETECTED" != "$ROBOT_FQBN" ]; then
        FQBN="$DETECTED"
        echo "WARNING: the connected board reports $DETECTED, not the robot's Teensy 4.1."
        echo "         Building for what's actually plugged in."
    else
        FQBN="$DETECTED"
    fi
fi

# The same board under teensy_loader_cli's name for it.
IFS=: read -r _ _ BOARD _ <<< "$FQBN"
case "$BOARD" in
    teensy41) MCU=TEENSY41 ;;
    teensy40) MCU=TEENSY40 ;;
    teensyMM) MCU=TEENSY_MICROMOD ;;
    teensy36) MCU=TEENSY36 ;;
    teensy35) MCU=TEENSY35 ;;
    teensy31) MCU=TEENSY31 ;;
    teensy30) MCU=TEENSY30 ;;
    teensyLC) MCU=TEENSYLC ;;
    *) die "Don't know the loader's name for board '$BOARD' (from $FQBN)." ;;
esac
echo "Board: $FQBN  ($MCU)"

# ---- Compile ----
mkdir -p "$SKETCH_DIR"
cp "$ROOT/lineflow.ino" "$SKETCH_DIR/lineflow.ino"
echo
echo "==> Compiling lineflow.ino"
arduino-cli compile --fqbn "$FQBN" --build-path "$OUT_DIR" "$SKETCH_DIR"
HEX="$OUT_DIR/lineflow.ino.hex"
[ -f "$HEX" ] || die "compile finished but $HEX is missing"

# ---- Upload ----
echo
echo "==> Uploading"
DASHBOARD_HAD_PORT=0
if [ -n "$PORT" ] && fuser "$PORT" >/dev/null 2>&1; then
    DASHBOARD_HAD_PORT=1
    echo "Note: something (probably the dashboard) has the Teensy's port open."
    echo "      Flashing still works; that connection drops when the Teensy reboots"
    echo "      and the dashboard reconnects by itself a few seconds later."
fi

if [ -n "$PORT" ]; then
    # Setting a Teensy's USB serial to 134 baud is its built-in "reboot into
    # the bootloader" request - handled by the Teensy core, so it works with
    # whatever sketch is running. This is what stops a GUI being needed.
    echo "Rebooting the Teensy into its bootloader..."
    stty -F "$PORT" 134 2>/dev/null || true
else
    echo "No Teensy serial port found - it may be blank, crashed, or already in the"
    echo "bootloader. If the upload doesn't start, press the Teensy's white button."
fi

# -w waits for the bootloader to appear; the loader reboots into the new
# firmware when done. Capped so a missing board can't hang the script.
if ! timeout 60 teensy_loader_cli --mcu="$MCU" -w -v "$HEX"; then
    die "upload failed or timed out. Press the white button on the Teensy and run
       this again. If it says 'Unable to open device', the udev rules are missing
       - rerun ./setup_teensy_flash.sh."
fi

# ---- Check it came back up ----
echo
echo "==> Waiting for the new firmware to boot"
for _ in $(seq 1 20); do
    PORT_LINK="$(ls /dev/serial/by-id/usb-Teensyduino* 2>/dev/null | head -n 1 || true)"
    [ -n "$PORT_LINK" ] && break
    sleep 0.5
done
[ -n "$PORT_LINK" ] || die "upload succeeded but the Teensy's serial port didn't come back"
PORT="$(readlink -f "$PORT_LINK")"
sleep 2   # boot: MPU6050 calibration + the 1s startup delay

# The firmware streams HDG: lines at 10 Hz when the gyro is up - a free
# end-to-end check. Raw mode with echo OFF matters: the default line
# discipline would echo every line back to the Teensy as a command.
stty -F "$PORT" raw -echo 9600 2>/dev/null || true
SAMPLE="$(timeout 2 cat "$PORT" 2>/dev/null || true)"
if grep -q "^HDG:" <<< "$SAMPLE"; then
    echo "OK: new firmware is running and the gyro is streaming heading."
elif [ -n "$SAMPLE" ]; then
    echo "Firmware is running, but no heading stream - the MPU6050 wasn't detected."
    echo "Check its wiring (SDA 18, SCL 19, 3.3V) and send GYRO from the dashboard."
else
    echo "Upload finished, but the Teensy printed nothing in 2s - open the dashboard"
    echo "and send GYRO to check it."
fi

echo
echo "Keep the robot still for a few seconds after a flash - the gyro calibrates at boot."
if [ "$DASHBOARD_HAD_PORT" -eq 1 ]; then
    echo "The dashboard picks the Teensy back up on its own within a few seconds."
fi
