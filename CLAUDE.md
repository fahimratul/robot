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
- **No line sensor, no wheel encoders**: the QTR-8 line-following array and
  the wheel encoders were both removed 2026-09-18. The encoders had a
  hardware fault that can't be replaced, which also ends any encoder-based
  closed-loop or autonomous-nav plan (see below) — driving is manual or
  open-loop scripted PATH playback only.

### Teensy pin map

| Function          | Pins |
|--------------------|------|
| Left motor PWM/DIR | 2 / 3 |
| Right motor PWM/DIR| 4 / 5 |

Pins 6-9 and 15-22 (formerly encoders and the QTR array) are free.

## Files

- `lineflow.ino` — Teensy firmware. Manual-drive takeover with a 400ms
  dead-man's-switch (`MANUAL_TIMEOUT_MS`), non-blocking scripted PATH
  playback (`pathRunning`/`updatePath()`), plain-text serial command
  protocol. No line-following, no encoder odometry.
- `dashbord.py` — Robot-side dashboard (Tkinter, light "classic" theme; runs
  on the Pi 5 + 7" touchscreen, tabbed CONTROL/PATH/SAVED/LIDAR MAP/LOG
  layout sized to fit small screens). Owns two serial links (Teensy +
  RPLidar C1 directly), draws the live LiDAR radar/map, auto-pauses a
  running PATH when something enters the front-180° obstacle zone and
  auto-resumes when clear, alerts (voice + phone vibration) if blocked
  >10s, lets the user build/record/save/run scripted timed-move sequences
  (PATH + SAVED tabs, see `PATH:` below), and runs a small HTTP server
  (port 8765) serving a phone remote-control page (stop / run path / manual
  d-pad / live radar) on the LAN.

## Serial protocol (Teensy ⇄ Pi) — current

Plain text, newline-terminated, replies are `OK:...` / `ERR:...`:

- `STOP` — stop immediately, cancelling manual mode or an in-progress path
- `MANUAL` — takeover; then `MFWD` / `MBACK` / `MLEFT` / `MRIGHT` / `MSTOP`
  drive motors directly (each resets the dead-man's-switch timer)
- `PATH:<steps>` — scripted playback: run a fixed, user-authored sequence of
  timed motion steps (e.g. forward 3s, then left 2s, then hold 5s...).
  `<steps>` is `ACTION,DURATION_MS` pairs separated by `;` (ACTION one of
  `FWD`/`BACK`/`LEFT`/`RIGHT`/`HOLD`), built by the dashboard's PATH tab
  from a step list the user adds to or records by driving. Open-loop/timed
  (no encoders); mutually exclusive with manual takeover — `MANUAL`/`STOP`
  cancel it. Runs non-blocking out of `loop()`, so `STOP` still takes effect
  immediately mid-path. Replies `OK:PATH_STARTED:<n>`, then
  `PATH_STEP:<i>/<n>` per step, `PATH:DONE` at the end, `ERR:PATH_EMPTY` if
  `<steps>` parsed to nothing.
- `PATH_STOP` — abort path playback early → `OK:PATH_STOPPED`
- `PATH_PAUSE` / `PATH_RESUME` — freeze/continue path playback in place
  (current step + elapsed time within it preserved via `pathPausedElapsedMs`
  in `lineflow.ino`), unlike `PATH_STOP` which cancels and resets position.
  Used by `dashbord.py`'s LiDAR obstacle auto-pause: when a path is running
  and an obstacle enters the front-180° zone, it sends `PATH_PAUSE`; resumes
  with `PATH_RESUME` when clear. Replies `OK:PATH_PAUSED`/`OK:PATH_RESUMED`,
  or `ERR:PATH_NOT_RUNNING` / `ERR:PATH_ALREADY_PAUSED` / `ERR:PATH_NOT_PAUSED`.
- Robot → Pi: `READY` on boot, plus the `OK:...`/`ERR:...` replies above.

### PATH tab: recording and a saved-path library

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
