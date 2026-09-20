#!/usr/bin/env bash
# Stop the audio sink suspending when idle, so the Bluetooth speaker doesn't
# swallow the start (or end) of what the robot says.
#
#   ./setup_audio.sh
#
# A Bluetooth A2DP link takes about a second to wake up. If the sink is
# allowed to suspend between announcements, every announcement loses its
# first words - and a sink that suspends mid-sentence chops the rest too.
#
# The config format depends on the version installed, and writing the wrong
# one stops WirePlumber starting at all, so this detects the version first
# and rolls its own change back if the service doesn't come back up.
set -uo pipefail

CONF_HOME="${XDG_CONFIG_HOME:-$HOME/.config}/wireplumber"
WRITTEN=()

# Over SSH these are usually unset, and without them neither `pactl` nor
# `systemctl --user` can reach the desktop session's audio stack - which
# looks exactly like "the audio system is broken" if you don't set them.
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=$XDG_RUNTIME_DIR/bus}"

say() { echo; echo "==> $*"; }

# Can we talk to this user's systemd at all? Not the same question as
# "is WirePlumber healthy", and confusing the two makes this script roll
# back a perfectly good config.
systemd_user_ok() { systemctl --user show-environment >/dev/null 2>&1; }

wp_restart_ok() {
    systemctl --user restart wireplumber >/dev/null 2>&1 || return 1
    sleep 1
    systemctl --user is-active --quiet wireplumber
}

rollback() {
    echo "WirePlumber did not come back up after the change - undoing it."
    for f in "${WRITTEN[@]:-}"; do [ -n "$f" ] && rm -f "$f"; done
    systemctl --user restart wireplumber >/dev/null 2>&1
    sleep 1
    systemctl --user is-active --quiet wireplumber \
        && echo "Audio is back as it was." \
        || echo "WARNING: WirePlumber is still down - reboot the Pi."
    exit 1
}

# ---- Which audio stack? ----
if ! command -v pactl >/dev/null 2>&1; then
    echo "ERROR: pactl not found - no PipeWire/PulseAudio here?" >&2
    exit 1
fi
SERVER="$(pactl info 2>/dev/null | grep '^Server Name:' || true)"
if [ -n "$SERVER" ]; then
    echo "Audio server: ${SERVER#Server Name: }"
else
    echo "Audio server: (can't reach it from this session - the config below is still"
    echo "               written correctly; it applies at the next login)"
fi

if command -v wireplumber >/dev/null 2>&1; then
    WP_VER="$(wireplumber --version 2>/dev/null | grep -oE '[0-9]+\.[0-9]+' | head -n 1)"
    say "WirePlumber $WP_VER"

    case "$WP_VER" in
        0.4)
            # 0.4 is configured in Lua, and ignores (or chokes on) the 0.5
            # .conf format. ALSA and Bluetooth are separate config dirs here.
            mkdir -p "$CONF_HOME/main.lua.d" "$CONF_HOME/bluetooth.lua.d"
            f1="$CONF_HOME/main.lua.d/51-no-suspend.lua"
            f2="$CONF_HOME/bluetooth.lua.d/51-no-suspend.lua"
            cat > "$f1" <<'EOF'
-- Keep ALSA sinks awake (see setup_audio.sh)
table.insert(alsa_monitor.rules, {
  matches = {{{ "node.name", "matches", "alsa_output.*" }}},
  apply_properties = { ["session.suspend-timeout-seconds"] = 0 },
})
EOF
            cat > "$f2" <<'EOF'
-- Keep the Bluetooth speaker awake (see setup_audio.sh)
table.insert(bluez_monitor.rules, {
  matches = {{{ "node.name", "matches", "bluez_output.*" }}},
  apply_properties = { ["session.suspend-timeout-seconds"] = 0 },
})
EOF
            WRITTEN=("$f1" "$f2")
            ;;
        0.5|1.*)
            mkdir -p "$CONF_HOME/wireplumber.conf.d"
            f1="$CONF_HOME/wireplumber.conf.d/51-no-suspend.conf"
            cat > "$f1" <<'EOF'
monitor.alsa.rules = [
  { matches = [ { node.name = "~alsa_output.*" } ]
    actions = { update-props = { session.suspend-timeout-seconds = 0 } } }
]
monitor.bluez.rules = [
  { matches = [ { node.name = "~bluez_output.*" } ]
    actions = { update-props = { session.suspend-timeout-seconds = 0 } } }
]
EOF
            WRITTEN=("$f1")
            ;;
        *)
            echo "Unrecognised WirePlumber version '$WP_VER' - not guessing at a format."
            exit 1
            ;;
    esac

    printf 'Wrote:'; printf ' %s' "${WRITTEN[@]}"; echo
    if systemd_user_ok; then
        wp_restart_ok || rollback
        echo "WirePlumber restarted, sinks no longer suspend."
    else
        # Keep the config: it is very probably fine, and there's no evidence
        # either way from here. It applies at the next login regardless.
        echo
        echo "Can't reach this user's service manager from here (usually means an SSH"
        echo "session without the desktop's environment), so WirePlumber wasn't"
        echo "restarted. The config is written and takes effect on the next login:"
        echo "  sudo reboot"
        echo "Then check it worked with: LOG tab -> Speaker test in the dashboard."
    fi

elif pgrep -x pulseaudio >/dev/null 2>&1; then
    say "PulseAudio (no WirePlumber)"
    mkdir -p "$HOME/.config/pulse"
    PA="$HOME/.config/pulse/default.pa"
    if ! grep -q "suspend-on-idle" "$PA" 2>/dev/null; then
        { echo ".include /etc/pulse/default.pa"
          echo "unload-module module-suspend-on-idle"; } >> "$PA"
    fi
    pactl unload-module module-suspend-on-idle 2>/dev/null
    echo "Suspend-on-idle unloaded now and disabled for future sessions ($PA)."
else
    echo "Neither WirePlumber nor PulseAudio found - nothing to configure." >&2
    exit 1
fi

# ---- Report what the speaker is actually doing ----
say "Current audio routing"
pactl info | grep -E "Default Sink" || true

CARD="$(pactl list cards short 2>/dev/null | awk '/bluez_card/ {print $2}' | head -n 1)"
if [ -n "$CARD" ]; then
    PROFILE="$(pactl list cards 2>/dev/null | awk -v c="$CARD" '
        $0 ~ c {found=1} found && /Active Profile:/ {print $3; exit}')"
    echo "Bluetooth card:  $CARD"
    echo "Active profile:  $PROFILE"
    case "$PROFILE" in
        a2dp*) echo "  OK - A2DP, the good one for playback." ;;
        *headset*|*handsfree*|*hfp*|*hsp*)
            echo "  ⚠ This is the headset/phone-call profile: low quality and prone to"
            echo "    chopping. Switch it with:"
            echo "      pactl set-card-profile $CARD a2dp-sink" ;;
        *) echo "  (unrecognised profile - a2dp-sink is the one you want)" ;;
    esac
else
    echo "No Bluetooth audio card connected right now."
fi

echo
echo "Now test from the dashboard: LOG tab -> Speaker test."
