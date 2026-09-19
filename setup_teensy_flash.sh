#!/usr/bin/env bash
# Run once on the Pi: install what flash_teensy.sh needs to compile
# lineflow.ino and upload it to the Teensy over USB - no laptop, no GUI.
#
#   - arduino-cli + PJRC's Teensy core   (compiling)
#   - teensy_loader_cli                  (uploading; headless, unlike the
#                                         Teensy Loader app the IDE uses)
#   - PJRC's udev rules                  (so uploading doesn't need sudo)
#
# The Teensy core download is a few hundred MB of toolchain - expect this to
# take a while on the Pi. Safe to re-run; finished steps are skipped.
set -euo pipefail

TEENSY_INDEX_URL="https://www.pjrc.com/teensy/package_teensy_index.json"
BIN_DIR="$HOME/.local/bin"
export PATH="$BIN_DIR:$PATH"

step() { echo; echo "==> $*"; }

step "System packages (build tools + libusb for teensy_loader_cli)"
sudo apt update
sudo apt install -y curl git build-essential libusb-dev python3 psmisc

step "arduino-cli"
if command -v arduino-cli >/dev/null 2>&1; then
    echo "already installed: $(arduino-cli version)"
else
    mkdir -p "$BIN_DIR"
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
        | BINDIR="$BIN_DIR" sh
fi

step "Teensy board support (teensy:avr)"
arduino-cli config init >/dev/null 2>&1 || true   # errors if one already exists - fine
if ! arduino-cli config dump 2>/dev/null | grep -q "pjrc.com/teensy"; then
    arduino-cli config add board_manager.additional_urls "$TEENSY_INDEX_URL"
fi
arduino-cli core update-index
arduino-cli core install teensy:avr

step "teensy_loader_cli"
# Built from PJRC's source rather than taken from apt: older packaged
# versions predate Teensy 4.x and fail with "Unknown MCU type".
if command -v teensy_loader_cli >/dev/null 2>&1 \
        && teensy_loader_cli --list-mcus 2>&1 | grep -q TEENSY41; then
    echo "already installed with Teensy 4.x support"
else
    src="$(mktemp -d)"
    git clone --depth 1 https://github.com/PaulStoffregen/teensy_loader_cli "$src"
    make -C "$src"
    sudo install -m 755 "$src/teensy_loader_cli" /usr/local/bin/teensy_loader_cli
    rm -rf "$src"
fi

step "udev rules (lets a normal user talk to the Teensy bootloader)"
sudo curl -fsSL -o /etc/udev/rules.d/00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# flash_teensy.sh sends the reboot-to-bootloader request through the serial
# port, which needs the dialout group (setup_pi.sh adds it too).
sudo usermod -aG dialout "$USER"

echo
echo "Done. If this is the first time you've run it, log out and back in (or"
echo "reboot) so the dialout group and ~/.local/bin on PATH take effect."
echo "Then, from this folder:  ./flash_teensy.sh"
