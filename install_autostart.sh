#!/usr/bin/env bash
# Run once on the Pi: make start_dashboard.sh launch with the desktop session,
# so the dashboard is on screen after a power-on with nothing to click.
#
#   ./install_autostart.sh           install
#   ./install_autostart.sh --remove  undo
#
# This uses an XDG autostart entry rather than a systemd service on purpose:
# dashbord.py is a Tkinter window, so it has to start *inside* the logged-in
# desktop session, which is exactly what ~/.config/autostart is for (and it
# works under both the X11 and Wayland sessions Pi OS ships).
set -e

ROOT="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
AUTOSTART_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/autostart"
ENTRY="$AUTOSTART_DIR/mist-cafe-bot.desktop"

if [ "${1:-}" = "--remove" ]; then
    rm -f "$ENTRY"
    echo "Removed $ENTRY - the dashboard will no longer start at boot."
    exit 0
fi

if [ ! -f "$ROOT/start_dashboard.sh" ]; then
    echo "ERROR: start_dashboard.sh not found in $ROOT" >&2
    exit 1
fi
chmod +x "$ROOT/start_dashboard.sh"

mkdir -p "$AUTOSTART_DIR"
# Exec goes through bash explicitly so the entry still works if the execute
# bit is lost (copying the folder off a Windows machine or a FAT USB stick).
cat > "$ENTRY" <<EOF
[Desktop Entry]
Type=Application
Name=MIST Cafe Bot Dashboard
Comment=Robot control dashboard (Teensy + RPLidar C1)
Exec=/bin/bash "$ROOT/start_dashboard.sh"
Terminal=false
X-GNOME-Autostart-enabled=true
EOF

echo "Installed $ENTRY"
echo "  -> runs: $ROOT/start_dashboard.sh"
echo
echo "The Pi must boot straight to the desktop and log in by itself, or there"
echo "is no session to start into. Turn on automatic login:"
echo "  Ubuntu:             Settings -> System -> Users -> Unlock -> Automatic Login: on"
echo "  Raspberry Pi OS:    sudo raspi-config -> System Options -> Boot / Auto Login"
echo "                      -> Desktop Autologin"
echo
echo "Reboot to test. If the dashboard doesn't appear, the reason is in:"
echo "  $ROOT/dashboard.log"
echo "Undo with: $ROOT/install_autostart.sh --remove"
