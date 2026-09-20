# MIST Cafe Bot — Project Notes

Manual-drive + scripted-path delivery robot. This file is the working memory
between Claude Code sessions: hardware truth and current firmware/dashboard
behavior. Update it whenever hardware or the protocol changes — don't let it
drift from the code.

## Hardware

- **MCU**: **Teensy 4.1** (`teensy:avr:teensy41`, loader MCU `TEENSY41`),
  talks to the Pi over USB serial, plain-text newline-terminated commands.
  The code says 9600 baud, but Teensy USB serial ignores the rate — it
  always runs at full USB speed. Its pins are **3.3V only, not 5V
  tolerant**, which is why every sensor here is powered from 3.3V.
- **Motor driver**: Cytron MDD10A, PWM+DIR per side.
- **Pi**: Raspberry Pi 5 running **Ubuntu** (kernel `6.8.0-*-raspi`), not
  Raspberry Pi OS — so no `raspi-config`; `vcgencmd` needs `sudo` (or the
  `video` group). USB devices at boot: the Teensy (`16c0:0483`,
  `ttyACM0`), the LiDAR's CP2102N (`10c4:ea60`, `ttyUSB0`), and the
  **Waveshare WS170120 7" touchscreen** (`0eef:0005`, USB touch — and likely
  USB power for its backlight too).
- **Power — known problem (2026-09-20)**: the supply was forced to "5A" (the
  Pi can't detect that itself from a non-PD source), which lifts the Pi's
  600mA USB limit. Measured **`EXT5V_V` ≈ 4.62–4.67V at idle** with
  `throttled=0x50005` (undervolting *and* throttling right now) and
  `Undervoltage detected!` in `dmesg` 9s after boot — *before* the LiDAR is
  started. The Pi powers off when the LiDAR's motor spins up. Needs ~5.1V
  under load. Check with
  `sudo vcgencmd pmic_read_adc EXT5V_V; sudo vcgencmd get_throttled`.
- **LiDAR**: RPLidar C1, plugged into the **Pi** directly (its own serial
  port, not through the Teensy), read by `dashbord.py` via the `rplidarc1`
  package.
- **MPU6050** (added 2026-09-18; detection widened 2026-09-20): probed at
  **0x68 and 0x69** (breakouts that tie AD0 high use 0x69), and **any**
  WHO_AM_I that isn't 0x00/0xFF is accepted — "MPU6050" modules are often
  MPU6500/9250 clones reporting 0x70/0x72/0x73, whose gyro registers are
  identical. Refusing on WHO_AM_I != 0x68 (the pre-2026-09-20 check) is a
  likely cause of a "gyro not detected" that is actually wired fine.
  **This robot's module is such a clone**: confirmed on hardware
  2026-09-20 reporting `ADDR=0x68,WHOAMI=0x74`, which the old check
  rejected — that was the real cause of its "gyro not detected".
  I2C gyro/accel breakout (GY-521 style),
  read on the Teensy via a minimal raw-I2C driver in `lineflow.ino` (no
  extra Arduino library). Only the Z-axis gyro is used, for heading — see
  "Heading (MPU6050 gyro)" below. **Not** a wheel-encoder replacement: it
  can't measure distance traveled, only rotation.
- **IR food-tray sensor** (added 2026-09-19): a cheap IR obstacle/proximity
  module (FC-51 style) aimed across the food tray, read by the Teensy on a
  plain digital pin. Tray occupied → module pulls OUT LOW; empty → HIGH.
  Power it from **3.3V** (Teensy 4.x pins are not 5V tolerant). Drives the
  "food taken → thank the customer → drive home" cycle, see below.
- **No line sensor, no wheel encoders**: the QTR-8 line-following array and
  the wheel encoders were both removed 2026-09-18. The encoders had a
  hardware fault that can't be replaced, which also ends any encoder-based
  closed-loop or autonomous-nav plan (see below) — driving is manual or
  open-loop scripted PATH playback, now with gyro-assisted turns/straight
  driving (below), but still no distance/position tracking.

### Teensy pin map

| Function          | Pins |
|--------------------|------|
| Left motor PWM/DIR | 7 / 16 |
| Right motor PWM/DIR| 8 / 17 |
| MPU6050 SDA / SCL  | 18 / 19 (Teensy's default `Wire` bus) |
| IR food-tray OUT   | 6 (`INPUT_PULLUP`) |

Pins 9, 15, 20, 21, 22 are free (9, 15 and 22 can do PWM).

**Pins 2, 3, 4, 5 are dead — do not reuse them.** They were the motor
PWM/DIR pins until 2026-09-20, when the common ground between the Teensy and
the MDD10A came loose *while driving*: motor return current went back through
the logic side and killed all four. The symptom was motors that stopped mid-run
and never came back, while the dashboard still looked completely healthy —
`MTEST` ran, `OK:MFWD` came back, and nothing moved. `PINTEST` (hold each
motor pin HIGH for 3s and measure against Teensy GND: 3.3V healthy, ~0V dead)
is what identified it. **The common ground is the wire to keep bolted down** —
screw terminal and thick wire, never a dupont jumper, ideally two of them.

## Files

- `lineflow.ino` — Teensy firmware. Manual-drive takeover with a 400ms
  dead-man's-switch (`MANUAL_TIMEOUT_MS`), non-blocking scripted PATH
  playback (`pathRunning`/`updatePath()`), gyro heading, debounced IR
  food-tray sensor (`updateFoodSensor()`), plain-text serial command
  protocol. No line-following, no encoder odometry.
- `dashbord.py` — Robot-side dashboard (Tkinter, light "classic" theme; runs
  on the Pi 5 + 7" touchscreen, tabbed CONTROL/PATH/SAVED/LIDAR MAP/LOG
  layout sized to fit small screens). Opens **fullscreen** (`-fullscreen`,
  covering the desktop taskbar and title bar); the header's "Exit full
  screen" button is the touch way out since there's no close button (Esc /
  F11 too), and `DASHBOARD_WINDOWED=1` starts windowed for PC development.
  **No text typing needed anywhere**: both Spinboxes are `state="readonly"`
  (arrows still work), because a focused text field pops the touchscreen's
  on-screen keyboard up over the dashboard; and Save As pre-fills the next
  free "Path N" so it can be saved with OK alone. The robot's on-screen
  keyboard is turned off outright
  (`gsettings set org.gnome.desktop.a11y.applications screen-keyboard-enabled false`),
  so nothing in the UI may *require* typing. A **Gyro test** button in the
  ROBOT LINK row sends `GYRO` + `I2CSCAN` and jumps to the LOG tab — the
  robot has no keyboard, so that button is the only way to run those.
  Save As drops out of fullscreen while its name dialog is open — it's the
  only typed input, and a fullscreen window can cover the on-screen keyboard
  or hide the dialog behind itself. The CONTROL tab is tight at 800x480:
  STOP shares a row with the state/alert text so the FOOD TRAY panel fits
  (~60px spare) — check new CONTROL-tab rows against that height. Owns two serial links (Teensy +
  RPLidar C1 directly), draws the live LiDAR radar/map, auto-pauses a
  running PATH when something enters the front-180° obstacle zone and
  auto-resumes when clear, alerts (voice + phone vibration) if blocked
  >10s, lets the user build/record/save/run scripted timed-move sequences
  (PATH + SAVED tabs, see `PATH:` below), runs the food-taken → thank-you →
  return-trip cycle off the IR sensor's events, shows a live gyro heading
  dial beside the manual d-pad, and runs a small HTTP server
  (port 8765) serving a phone remote-control page (stop / run path / return
  home / manual d-pad / live radar / heading) on the LAN.

- `setup_pi.sh` — one-time install on the Pi (apt deps, `venv/`, `dialout`
  group).
- `start_dashboard.sh` — launcher used at boot: waits for the desktop/USB to
  settle, picks `venv/bin/python3`, runs `dashbord.py`, logs to
  `dashboard.log` (previous boot kept as `dashboard.log.1`, both gitignored),
  and restarts it if it crashes — but not on a clean exit, so closing the
  window stays closed, and not after 5 instant failures in a row.
- `install_autostart.sh` — one-time: writes
  `~/.config/autostart/mist-cafe-bot.desktop` so the desktop session launches
  `start_dashboard.sh`. XDG autostart rather than a systemd service because
  the dashboard is a Tkinter window and must start inside the logged-in
  session; needs automatic desktop login (the Pi runs **Ubuntu**: Settings
  → System → Users → Automatic Login — there is no `raspi-config`).
  `--remove` undoes it.
- `setup_audio.sh` — one-time: stops the audio sink suspending when idle, so
  the Bluetooth speaker stops clipping announcements. Version-aware and
  self-rolling-back; see "Voice" below.
- `setup_teensy_flash.sh` — one-time: installs `arduino-cli` (to
  `~/.local/bin`) + PJRC's `teensy:avr` core, builds `teensy_loader_cli`
  from PJRC's source (older apt builds predate Teensy 4.x), installs PJRC's
  udev rules, adds `dialout`.
- `flash_teensy.sh` — compile `lineflow.ino` and upload it to the Teensy
  from the Pi, headless (`git pull && ./flash_teensy.sh`). Compiles from a
  copy in `.build/lineflow/` (gitignored) because `arduino-cli` requires the
  folder name to match the `.ino`. Builds for the robot's Teensy 4.1 unless
  `arduino-cli board list` reports a different board is plugged in (then it
  warns and builds for that; `TEENSY_FQBN=...` overrides both), finds the port
  via `/dev/serial/by-id/usb-Teensyduino*` so it can't pick the LiDAR,
  reboots the Teensy into its bootloader with the 134-baud trick (handled
  by the Teensy core, so no GUI Teensy Loader is needed), uploads with
  `teensy_loader_cli -w`, then checks for the `HDG:` stream as an
  end-to-end "firmware running + gyro up" test. A dashboard connected at the
  time loses its link when the Teensy reboots — Disconnect/Connect after.

### Voice (pyttsx3 → espeak-ng, Bluetooth speaker)

Four spoken messages, all constants at the top of `dashbord.py`:
`OBSTACLE_VOICE_MSG` (something enters the front-180 zone),
`STALL_VOICE_MSG` (still blocked after `STALL_ALERT_SECONDS`),
`FOOD_TAKEN_VOICE_MSG`, `RETURN_DONE_VOICE_MSG`. Every one is also written to
the log as `Voice: "..."`, so the log still shows what it tried to say when
nothing is audible.

The LOG tab's **Speaker test** button says a test phrase, then reports the
default sink (warning when it isn't the `bluez_output...` one), lists the
sinks, and names the connected Bluetooth device — silence otherwise looks
the same whether pyttsx3 is missing, espeak-ng won't start, the speaker has
dropped its connection, or audio is going to HDMI. It then asks "did you
hear it?" and prints what to check on a no. `pactl`/`bluetoothctl` run off
the Tk thread and are treated as optional (absent on a dev PC).

The speaker is **Bluetooth**, so its A2DP link sleeps when idle and swallows
whatever is said in the ~1s it takes to wake — and a sink that suspends
mid-sentence chops the rest. Two halves to the fix:

- `VOICE_LEAD_IN` / `VOICE_LEAD_OUT` (commas, wrapped around the text in
  `SpeechWorker._run`) make espeak emit silence at both ends, so the stream
  is live before the words start and still up when they finish.
- `setup_audio.sh` stops the sink suspending at all. **The config format
  depends on the WirePlumber version** — 0.4 is Lua in `main.lua.d/` +
  `bluetooth.lua.d/`, 0.5+ is SPA-JSON in `wireplumber.conf.d/` — and
  writing the wrong one stops WirePlumber starting at all (this happened
  2026-09-20). The script detects the version, and rolls its own files back
  if the service doesn't come back up. It also reports the Bluetooth card's
  active profile: `headset`/`hfp` is the call profile (low quality, chops),
  `a2dp-sink` is the one for playback.

Note the obstacle line is spoken on *every* transition into the zone, even
when no path is running.

## Serial protocol (Teensy ⇄ Pi) — current

Plain text, newline-terminated, replies are `OK:...` / `ERR:...`:

- `STOP` — stop immediately, cancelling manual mode or an in-progress path
- `MANUAL` — takeover; then `MFWD` / `MBACK` / `MLEFT` / `MRIGHT` / `MSTOP`
  drive motors directly (each resets the dead-man's-switch timer)
- `PATH:<steps>` — scripted playback: run a fixed, user-authored sequence of
  motion steps (e.g. forward 3s, then turn left 90°, then hold 5s...).
  `<steps>` is `ACTION,VALUE` pairs separated by `;`. ACTION is one of
  `FWD`/`BACK`/`LEFT`/`RIGHT`/`HOLD` (VALUE = milliseconds, open-loop/timed)
  or `TURNL`/`TURNR` (VALUE = degrees, closed-loop via the gyro — see
  "Heading" below) or `ALIGN` (VALUE = degrees offset from the anchored
  start heading, closed-loop and *absolute* — see "Absolute heading anchor"
  below; also emits `ALIGN:<residual°>` when the step ends), built by the
  dashboard's PATH tab from a step list the
  user adds to or records by driving (recording produces timed
  FWD/BACK/HOLD and gyro-measured TURNL/TURNR — see "Recording" below —
  never ALIGN). Mutually exclusive with manual takeover
  — `MANUAL`/`STOP` cancel it. Runs non-blocking out of `loop()`, so `STOP`
  still takes effect immediately mid-path. Replies `OK:PATH_STARTED:<n>`,
  then `PATH_STEP:<i>/<n>` per step, `PATH:DONE` at the end,
  `ERR:PATH_EMPTY` if `<steps>` parsed to nothing.
- `PATH_STOP` — abort path playback early → `OK:PATH_STOPPED`
- `PATH_PAUSE` / `PATH_RESUME` — freeze/continue path playback in place
  (current step + elapsed time within it preserved via `pathPausedElapsedMs`
  in `lineflow.ino`), unlike `PATH_STOP` which cancels and resets position.
  Used by `dashbord.py`'s LiDAR obstacle auto-pause: when a path is running
  and an obstacle enters the front-180° zone, it sends `PATH_PAUSE`; resumes
  with `PATH_RESUME` when clear. Replies `OK:PATH_PAUSED`/`OK:PATH_RESUMED`,
  or `ERR:PATH_NOT_RUNNING` / `ERR:PATH_ALREADY_PAUSED` / `ERR:PATH_NOT_PAUSED`.
- `FOOD` — query the IR tray sensor → `FOOD:PRESENT` / `FOOD:ABSENT`. The
  dashboard sends this on connect and on `READY` so its tray display starts
  in sync instead of waiting for the next pickup.
- `HEADING_ANCHOR` — latch `currentHeadingDeg` as `anchorHeadingDeg`, the
  absolute reference `ALIGN` steps steer back to →
  `OK:HEADING_ANCHOR:<deg>`. The dashboard sends it immediately before every
  *outbound* path (never before a return trip, which must keep the
  delivery's anchor).
- `HEADING_RESET` — zero `currentHeadingDeg` **and** `anchorHeadingDeg` →
  `OK:HEADING_RESET` (zeroing one without the other would silently offset
  every later `ALIGN`)
- `GYRO` — diagnostic →
  `GYRO:<OK|NOT_FOUND>,<currentHeadingDeg>,ADDR=0x<addr>,WHOAMI=0x<id>,SIGN=<-1|0|1>`
  (first two fields unchanged from before 2026-09-20); use to verify the
  MPU6050 is wired/detected and that the value changes sensibly when the
  chassis is rotated by hand
- `I2CSCAN` — diagnostic → one `I2C:0x<addr>` line per device answering on
  the bus, then `I2C:DONE:<count>`. Nothing found = wiring/power; something
  at 0x68 or 0x69 = wired fine. Also printed automatically after
  `ERR:MPU6050_NOT_FOUND` at boot.
- Robot → Pi: `READY` on boot (followed by the initial `FOOD:PRESENT`/
  `FOOD:ABSENT`), the unsolicited tray events `FOOD:PRESENT` / `FOOD:TAKEN`
  (see below), plus the `OK:...`/`ERR:...` replies above, and two
  unsolicited gyro lines (added 2026-09-19):
  - `HDG:<heading>,<gyroRightSign>,<anchor>` at 10 Hz whenever the MPU6050
    is up, idle or not — feeds the dashboard's live gyro dial. Teensy USB
    serial ignores the 9600 baud setting, so the stream costs nothing. The
    dashboard keeps these (and `MTURN`) out of its log.
  - `MTURN:<L|R>,<deg>` when a manual `MLEFT`/`MRIGHT` segment ends — the
    gyro-measured rotation, used by the recorder (see "Recording").

  **Manual commands are segmented on the Teensy**: the dashboard re-sends a
  held button every 150ms, so only a *change* of `MFWD`/`MBACK`/`MLEFT`/
  `MRIGHT`/`MSTOP` starts a new segment (`manualMotion`, `setManualMotion()`
  in `lineflow.ino`). The dead-man's switch, `STOP`, and a `PATH:` starting
  all close an open segment too (`leaveManualMode()`), so a turn is
  reported however it ended.

`PATH_MAX_STEPS` is **64** (raised from 30 on 2026-09-19): a return trip is
the delivery path mirrored plus two U-turns, so it needs roughly double a
recorded delivery's step count. `dashbord.py` mirrors the constant and
refuses to send a longer path rather than let the firmware silently drop the
steps past the cap and strand the robot mid-route.

### Heading (MPU6050 gyro) — added 2026-09-18

Z-axis gyro only, integrated into a free-running `currentHeadingDeg` in
`lineflow.ino` (`updateHeading()`, called every `loop()` iteration
unconditionally). Three features use it:

- **`TURNL`/`TURNR` PATH steps**: closed-loop turns that stop once the
  robot has rotated the target number of degrees (measured via the gyro),
  instead of the old approach of guessing a fixed motor-on duration. Reuses
  the existing tuned `left()`/`right()` motor functions — same physical
  turn, just a different, more accurate stopping condition — so the
  turn *direction* doesn't need re-verifying, only degrees does (and that's
  self-correcting: it stops on rotation *magnitude*, so a wrong yaw-sign
  assumption can't affect correctness here, only whether the printed heading
  itself matches your intuition when using `GYRO` to look at it).
- **`HEADING_HOLD`** (`applyHeadingHold()`): while driving straight — PATH
  `FWD`/`BACK` steps **and** manual `MFWD`/`MBACK` (since 2026-09-19) — a
  simple P controller (`HEADING_KP`, `HEADING_MAX_CORRECTION`) nudges the
  two wheels' PWM apart to hold the heading the segment started on.
  `HEADING_KP` is a **magnitude only**: the steering direction comes from
  the learned `gyroRightSign` (see "Absolute heading anchor"), derived from
  the motor functions — a positive correction runs the right wheel faster,
  which going forward rotates the robot the way `right()` does and in
  reverse the opposite way. Until a first turn has taught the sign,
  heading-hold stays **off** (no correction beats a wrong one), so the very
  first straight after boot is uncorrected. The pre-2026-09-19 version used
  one sign for `FWD` and `BACK`, which made `BACK` steps amplify drift;
  simulated with a 10%-weak left motor, 3s of driving now ends ~4° off
  instead of ~36°, forward and reverse, both mounting orientations.
  `#define HEADING_HOLD_ENABLED` still toggles it off entirely.
- **`ALIGN` PATH steps** — absolute, see the next section.

Gyro integration drifts slowly over time (fine for single steps a few
seconds long) and the bias is calibrated once at boot while assumed
stationary — power-cycle the Teensy to recalibrate if it seems off.
This does **not** revive the SLAM/autonomous-nav roadmap below — it gives
heading only, not distance/position, and both are needed for occupancy-grid
mapping or dead-reckoning.

### Absolute heading anchor (`ALIGN`) — added 2026-09-19

Everything else about the heading is *relative*: `TURNL`/`TURNR` and
`HEADING_HOLD` both measure against `pathStepStartHeadingDeg`, re-snapshotted
at every step. That keeps drift from corrupting a step, but it also means
each step's small error (a turn stopping a few degrees early, a timed
`LEFT`/`RIGHT` overshooting, slip) is permanent — nothing ever corrects it,
so the robot ends each round trip a bit further off than the last.

`anchorHeadingDeg` is the fix: an absolute reference that persists **across
separate PATH commands**, so the delivery and the return trip share one.

- `HEADING_ANCHOR` latches it. `dashbord.py` sends it right before every
  outbound path (in `_run_path`, guarded by `if not is_return`).
- An `ALIGN,<offset>` step turns until `currentHeadingDeg` is within
  `ALIGN_TOLERANCE_DEG` (3°) of `anchorHeadingDeg + offset`, taking the short
  way round (`normalizeDeg180`), then reports `ALIGN:<residual>`. It gives up
  after `ALIGN_TIMEOUT_MS` (8s) rather than spinning forever if the gyro is
  dead — the dashboard flags a residual over `ALIGN_OK_RESIDUAL_DEG` in the
  log instead of letting it pass silently.
- `build_return_path()` ends every return trip with `ALIGN,0`, so the robot
  parks facing exactly the way it left, however sloppy the round trip was.

**How it copes with the unverified gyro sign**: unlike `TURNL`/`TURNR`
(magnitude only), `ALIGN` has to know *which way* to spin to reduce the
error, which depends on the board's mounting. So it isn't assumed — it's
learned. `gyroRightSign` records whether `right()` makes the heading rise or
fall, captured for free whenever the robot turns deliberately — a
`TURNL`/`TURNR` or timed `LEFT`/`RIGHT` step completing, or a manual
`MLEFT`/`MRIGHT` segment ending (`learnGyroSign()`). It lives in RAM, so it
is relearned after every boot from the first turn; heading-hold and the
dashboard dial both depend on it too. If an `ALIGN` runs before anything has taught it,
`maintainAlign()` spins `right()` and watches until the rotation exceeds
`GYRO_SIGN_LEARN_DEG` (5°), which settles the question; a wrong initial guess
costs at most those 5°, which the alignment then corrects. `ALIGN` is also
the one action re-evaluated every `loop()` iteration (via `maintainAlign()`,
like `maintainHeadingHold()`) rather than set once in `applyPathAction()`,
because the direction can only be chosen while the error is being watched.

Verified in simulation against both mounting orientations (cold start with
the sign unknown, both turn directions, a 185° error taking the short way,
an offset target, a full round trip, and the no-gyro timeout).

The honest limit: the anchor is a gyro heading, so integration drift (roughly
a degree or two a minute) rides on top of it. A long wait at the table eats
into the accuracy. It is still far better than letting per-turn error pile up
uncorrected, but it is not a compass.

### PATH tab: recording and a saved-path library

- The step-builder dropdown includes `TURN LEFT`/`TURN RIGHT`/`ALIGN TO
  START` alongside the timed actions; picking one switches the value
  spinbox's unit label ("sec" ↔ "deg" ↔ "deg off start") and range via
  `_on_path_action_changed()` in `dashbord.py`. `DEGREE_ACTIONS` (a
  module-level set — the two turns plus `ALIGN TO START`) is checked
  everywhere the unit difference matters — step-list display, the saved-path
  total summary, and the `PATH:` wire-format builder (degrees sent as-is,
  everything else × 1000 for ms). `ALIGN TO START` is the one action whose
  value may be 0, since "no offset from the start heading" is its normal
  case, so `_add_path_step`'s "> 0" validation excludes it.
- **Record-by-driving**: a "● Record" toggle on the PATH tab (only usable
  while in Manual Control). While recording, every manual drive command
  (desktop dpad or phone d-pad — both funnel through `dashbord.py`'s
  `_send()`) is captured as a step: each held direction becomes an
  `ACTION,duration` step, and idle gaps between moves ≥
  `MIN_RECORDED_GAP_SECONDS` (0.3s) become `HOLD` steps — so "drive
  forward, let go, wait, drive left" naturally records as
  `FORWARD 3s, HOLD 4s, LEFT 2s`, matching how the user actually drove it.
  Stopping recording (or hitting global STOP, which also cancels manual
  mode) finalizes the captured sequence into the PATH tab's step list, same
  as if it had been hand-built there.
- **Recorded turns are gyro-measured** (since 2026-09-19). The recorder
  still appends a timed `LEFT`/`RIGHT` step when a turn ends (at send time,
  so step order is always right), then the Teensy's `MTURN:<L|R>,<deg>`
  report — measured at motor-off, the same moment a `TURNL`/`TURNR` step
  stops, so playback reaches the same angle — upgrades it in place to
  `TURN LEFT`/`TURN RIGHT <deg>` (`_on_manual_turn()`). This is the main
  accuracy win for driven paths: a timed turn lands wherever battery level
  and floor grip put it, a gyro turn lands on the angle. Turns under
  `MIN_RECORDED_TURN_DEG` (2°) stay timed; with no gyro, no report arrives
  and every turn stays timed. `FORWARD`/`BACK` stay timed — the gyro can't
  measure distance.

  Pairing reports to steps (`_record_pending_turns`) is newest-first within
  `MTURN_MATCH_WINDOW_S` (1s), with direction checked, because two things
  can desync a plain FIFO: a turn too short to keep (<0.05s) still gets a
  report, so it gets a placeholder entry; and the dead-man's switch can end
  a turn on the Teensy *before* the dashboard records it, so that report
  finds nothing and the later entry never gets one — the window makes such
  an entry expire instead of soaking up a later turn's angle.
- **Saved path library**: the SAVED tab lists named path step-sequences,
  persisted to `saved_paths.json` (next to `dashbord.py`, local state, not
  checked in — see `.gitignore`) via `_load_saved_paths`/`_write_saved_paths`.
  "Save Current As..." names and stores whatever's currently in the PATH
  tab's step list (hand-built or recorded); "Load" copies a saved path back
  into the editable step list; "Run" loads it and immediately sends the
  `PATH:` command; "Delete" removes it. This is the "keep track of many
  named paths, then pick one to run" workflow.
- **Live gyro dial** (CONTROL tab, beside the manual d-pad so it costs the
  480px screen no height; `_draw_heading_dial()`): a top-down view of the
  robot relative to START — the anchor, i.e. the heading the last delivery
  set off on (or power-on before any). Arrow = robot now, shaded wedge = how
  far it has turned from START, drawn only once >5° off (on Windows a pie
  slice ~1° wide renders as the *whole* disc, a GDI quirk), readout amber
  past 5°. Right turns rotate it clockwise via the learned
  `gyroRightSign`; until a first turn it says "turn to calibrate" since it
  may be mirrored. Shows "NO GYRO DATA" if `HDG` stops for
  `HDG_STALE_SECONDS`. The phone page shows the same number as a text line
  (`heading_from_start` in `/api/status`, via `_heading_from_start()`).
- Phone remote page also has a "▶ RUN PATH" button (`/api/path/run`) that
  runs whatever's currently in `path_steps` — it does not expose the
  recording/saved-library UI, just a trigger, matching the phone page's
  existing simple-trigger design (Stop/manual d-pad, not full editors).

### Food tray + automatic return trip — added 2026-09-19

The delivery cycle, driven by the IR tray sensor:

1. A PATH runs out to the table and reports `PATH:DONE`. `dashbord.py` arms
   `awaiting_pickup` and snapshots the steps it just sent (`delivery_steps`)
   — only a path that *completed* arms it; `OK:PATH_STOPPED` does not.
2. The customer lifts the food → Teensy debounces the IR pin
   (`FOOD_DEBOUNCE_MS`, 250ms, so a hand passing over the tray doesn't
   count) and pushes `FOOD:TAKEN`.
3. The dashboard speaks **"Thank you sir"** (same `SpeechWorker` as the
   obstacle alert), waits `RETURN_DELAY_SECONDS` (3s, so the customer hears
   it and steps clear), then sends the return path.
4. The return path is `build_return_path()`: a 180° turn, the delivery steps
   in reverse order with every turn mirrored (`REVERSE_ACTION_MIRROR` —
   `FORWARD`/`BACK`/`HOLD` unchanged, `LEFT`↔`RIGHT`, `TURN LEFT`↔
   `TURN RIGHT`), a second 180° turn, and finally `ALIGN,0`, which trims off
   whatever error the round trip accumulated and puts the robot back on the
   heading anchored in step 1 (see "Absolute heading anchor"). `ALIGN` steps
   inside the delivery path are dropped rather than mirrored — they're
   absolute headings for the outbound route and mean nothing reversed. The mirroring is because the U-turn leaves the robot facing back
   down the route; the docstring works the geometry through with an example.
   It's loaded into the PATH tab's step list so the retrace is visible while
   it runs, and LiDAR obstacle auto-pause applies to it like any other path.
5. `PATH:DONE` on the return trip ends the cycle (`return_active` keeps it
   from arming another pickup). Taking food off the tray at any other time
   just logs and is ignored.

Only `FOOD:TAKEN` (a real present→empty transition) triggers this — the
`FOOD:ABSENT` reply to a `FOOD` query and the boot-time announce never do,
so connecting the dashboard to a robot with an empty tray can't launch it.

Controls: the CONTROL tab's **◆ FOOD TRAY (IR SENSOR)** frame shows the tray
state, an "Auto-return when food is taken" checkbox, and a **↩ Return Now**
button that runs the retrace without waiting for the sensor (also on the
phone page as **↩ RETURN HOME**, plus a tray-status line). `STOP`, manual
takeover and starting another path all cancel a pending return.

The two U-turns are `TURNR,180` gyro turns, so they need the MPU6050. With
no gyro they fall back to spinning until `PATH_TURN_TIMEOUT_MS` (raised to
8000ms, since a real 180° turn at `nudgeSpeed` can exceed the old 5s cap and
get cut short). `RETURN_TURN_ACTION`/`RETURN_TURN_DEGREES` in `dashbord.py`
change the spin direction/angle.

## Line-following and encoder odometry — removed (2026-09-18)

The robot used to line-follow off a QTR-8 array with intersection turns
toward a selected "table" (`TABLE1`/`TABLE2`/`START` commands), and a
Phase 1 wheel-encoder odometry implementation existed as the first step of
a planned LiDAR-SLAM autonomous-navigation feature (occupancy-grid mapping,
click-to-goal path planning, return-to-start). Both are gone: the wheel
encoders have a hardware fault that can't be replaced, which makes any
encoder-based closed-loop motion or odometry permanently unavailable, so
the whole autonomous-nav roadmap is abandoned along with it. The QTR
line-following was removed in the same pass since it's no longer the
robot's operating mode either way. Driving is now manual (d-pad) or
scripted open-loop PATH playback only. If encoder hardware is ever
replaced, treat that as a fresh design rather than resurrecting the old
Phase 1-5 plan.

## Working conventions

- Keep the serial protocol plain-text and backward compatible where
  possible — the phone remote and dashboard both parse it directly.
- This file should be updated at the end of any session that changes
  hardware or the serial protocol.
