#!/usr/bin/env bash
# Run this on the Raspberry Pi 5 itself (in the robot/ folder), once, to
# install everything dashbord.py needs. Then reboot (or re-login) so the
# dialout group membership takes effect.
set -e

sudo apt update
sudo apt install -y python3-tk python3-venv python3-pip espeak-ng

python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

sudo usermod -aG dialout "$USER"

echo
echo "Done. Log out/in (or reboot) so the 'dialout' group takes effect,"
echo "then run:  source venv/bin/activate && python3 dashbord.py"
