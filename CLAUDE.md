# MIST Cafe Bot — Project Notes

Manual-drive + scripted-path delivery robot. This file is the working memory
between Claude Code sessions: hardware truth and current firmware/dashboard
behavior. Update it whenever hardware or the protocol changes — don't let it
drift from the code.

## Hardware

- **MCU**: Teensy (see `lineflow.ino`), talks to the Pi over USB serial at
  9600 baud, plain-text newline-terminated commands.
- **Motor driver**: Cytron MDD10A, PWM+DIR per side.
- **LiDAR**: RPLidar C1, plugged into the **Pi** directly (its own serial
  port, not through the Teensy), read by `dashbord.py` via the `rplidarc1`
  package.
- **MPU6050** (added 2026-09-18): I2C gyro/accel breakout (GY-521 style),
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
| Left motor PWM/DIR | 2 / 3 |
| Right motor PWM/DIR| 4 / 5 |
| MPU6050 SDA / SCL  | 18 / 19 (Teensy's default `Wire` bus) |
| IR food-tray OUT   | 6 (`INPUT_PULLUP`) |

Pins 7-9 and 15-17, 20-22 (formerly encoders and the rest of the QTR array)
are free.

## Files

- `lineflow.ino` — Teensy firmware. Manual-drive takeover with a 400ms
  dead-man's-switch (`MANUAL_TIMEOUT_MS`), non-blocking scripted PATH
  playback (`pathRunning`/`updatePath()`), gyro heading, debounced IR
  food-tray sensor (`updateFoodSensor()`), plain-text serial command
  protocol. No line-following, no encoder odometry.
- `dashbord.py` — Robot-side dashboard (Tkinter, light "classic" theme; runs
  on the Pi 5 + 7" touchscreen, tabbed CONTROL/PATH/SAVED/LIDAR MAP/LOG
  layout sized to fit small screens). Owns two serial links (Teensy +
  RPLidar C1 directly), draws the live LiDAR radar/map, auto-pauses a
  running PATH when something enters the front-180° obstacle zone and
  auto-resumes when clear, alerts (voice + phone vibration) if blocked
  >10s, lets the user build/record/save/run scripted timed-move sequences
  (PATH + SAVED tabs, see `PATH:` below), runs the food-taken → thank-you →
  return-trip cycle off the IR sensor's events, and runs a small HTTP server
  (port 8765) serving a phone remote-control page (stop / run path / return
  home / manual d-pad / live radar) on the LAN.

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
  session; needs "Desktop Autologin" in `raspi-config`. `--remove` undoes it.

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
  "Heading" below), built by the dashboard's PATH tab from a step list the
  user adds to or records by driving (recording only ever produces the
  timed actions, not TURNL/TURNR). Mutually exclusive with manual takeover
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
- `HEADING_RESET` — zero `currentHeadingDeg` → `OK:HEADING_RESET`
- `GYRO` — diagnostic → `GYRO:<OK|NOT_FOUND>,<currentHeadingDeg>`; use to
  verify the MPU6050 is wired/detected and that the value changes sensibly
  when the chassis is rotated by hand
- Robot → Pi: `READY` on boot (followed by the initial `FOOD:PRESENT`/
  `FOOD:ABSENT`), the unsolicited tray events `FOOD:PRESENT` / `FOOD:TAKEN`
  (see below), plus the `OK:...`/`ERR:...` replies above.

`PATH_MAX_STEPS` is **64** (raised from 30 on 2026-09-19): a return trip is
the delivery path mirrored plus two U-turns, so it needs roughly double a
recorded delivery's step count. `dashbord.py` mirrors the constant and
refuses to send a longer path rather than let the firmware silently drop the
steps past the cap and strand the robot mid-route.

### Heading (MPU6050 gyro) — added 2026-09-18

Z-axis gyro only, integrated into a free-running `currentHeadingDeg` in
`lineflow.ino` (`updateHeading()`, called every `loop()` iteration
unconditionally). Two features use it:

- **`TURNL`/`TURNR` PATH steps**: closed-loop turns that stop once the
  robot has rotated the target number of degrees (measured via the gyro),
  instead of the old approach of guessing a fixed motor-on duration. Reuses
  the existing tuned `left()`/`right()` motor functions — same physical
  turn, just a different, more accurate stopping condition — so the
  turn *direction* doesn't need re-verifying, only degrees does (and that's
  self-correcting: it stops on rotation *magnitude*, so a wrong yaw-sign
  assumption can't affect correctness here, only whether the printed heading
  itself matches your intuition when using `GYRO` to look at it).
- **`HEADING_HOLD`**: while a PATH `FWD`/`BACK` step drives, a simple P
  controller (`HEADING_KP`, `HEADING_MAX_CORRECTION` in `lineflow.ino`)
  nudges the two wheels' PWM apart to counter heading drift and drive
  straighter. **Unverified sign** — flipping the board's mounting orientation
  flips which way is "positive" rotation, so if this makes drift *worse*
  instead of better on real hardware, flip the sign of `HEADING_KP`.
  `#define HEADING_HOLD_ENABLED` toggles it off entirely without removing
  code, in case it misbehaves before that sign is confirmed.

Gyro integration drifts slowly over time (fine for single steps a few
seconds long) and the bias is calibrated once at boot while assumed
stationary — power-cycle the Teensy to recalibrate if it seems off.
This does **not** revive the SLAM/autonomous-nav roadmap below — it gives
heading only, not distance/position, and both are needed for occupancy-grid
mapping or dead-reckoning.

### PATH tab: recording and a saved-path library

- The step-builder dropdown includes `TURN LEFT`/`TURN RIGHT` alongside the
  timed actions; picking one switches the value spinbox's unit label
  ("sec" ↔ "deg") and range via `_on_path_action_changed()` in
  `dashbord.py`. `TURN_ACTIONS` (a module-level set) is checked everywhere
  the unit difference matters — step-list display, the saved-path total
  summary, and the `PATH:` wire-format builder (degrees sent as-is,
  everything else × 1000 for ms).
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
- **Saved path library**: the SAVED tab lists named path step-sequences,
  persisted to `saved_paths.json` (next to `dashbord.py`, local state, not
  checked in — see `.gitignore`) via `_load_saved_paths`/`_write_saved_paths`.
  "Save Current As..." names and stores whatever's currently in the PATH
  tab's step list (hand-built or recorded); "Load" copies a saved path back
  into the editable step list; "Run" loads it and immediately sends the
  `PATH:` command; "Delete" removes it. This is the "keep track of many
  named paths, then pick one to run" workflow.
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
   `TURN RIGHT`), then a second 180° turn so the robot parks on its original
   heading. The mirroring is because the U-turn leaves the robot facing back
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
