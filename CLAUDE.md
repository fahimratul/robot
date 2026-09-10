# MIST Cafe Bot — Project Notes

Line-following delivery robot, evolving toward point-to-point autonomous
navigation. This file is the working memory between Claude Code sessions:
hardware truth, current firmware/dashboard behavior, and the roadmap for the
autonomous-nav feature. Update it whenever hardware, protocol, or the plan
changes — don't let it drift from the code.

## Hardware

- **MCU**: Teensy (see `lineflow.ino`), talks to the laptop over USB serial
  at 9600 baud, plain-text newline-terminated commands.
- **Motor driver**: Cytron MDD10A, PWM+DIR per side.
- **Line sensor**: QTR-8 reflectance array (`QTRSensors` lib).
- **LiDAR**: RPLidar C1, plugged into the **laptop** directly (its own
  serial port, not through the Teensy), read by `dashbord.py` via the
  `rplidarc1` package.
- **Wheel encoders** (added 2026-09-09, not yet wired into firmware):
  - Left motor: Channel A = pin 6, Channel B = pin 7
  - Right motor: Channel A = pin 8, Channel B = pin 9
  - Pins 6/7/8/9 are free (no conflict with existing pin map below).

### Teensy pin map

| Function          | Pins |
|--------------------|------|
| QTR sensors (8ch)  | 22, 21, 20, 19, 18, 17, 16, 15 |
| Left motor PWM/DIR | 2 / 3 |
| Right motor PWM/DIR| 4 / 5 |
| Left encoder A/B   | 6 / 7 |
| Right encoder A/B  | 8 / 9 |

## Files

- `lineflow.ino` — Teensy firmware. Line-following state machine off the QTR
  array, intersection turn logic (`doIntersectionTurn`, direction chosen by
  `selectedTable`), manual-drive takeover with a 400ms dead-man's-switch
  (`MANUAL_TIMEOUT_MS`), plain-text serial command protocol.
- `dashbord.py` — Robot-side dashboard (Tkinter, "HUD" theme; runs on the
  Pi 5 + 7" touchscreen, tabbed CONTROL/PATH/LIDAR MAP/LOG layout sized to
  fit small screens). Owns two serial links (Teensy + RPLidar C1 directly),
  draws the live LiDAR radar/map, auto-stops the robot when something
  enters the front-180° obstacle zone and auto-resumes when clear, alerts
  (voice + phone vibration) if the robot is stalled >10s, lets the user
  build/run a scripted timed-move sequence (PATH tab, see `PATH:` below),
  and runs a small HTTP server (port 8765) serving a phone remote-control
  page (table select / start / stop / manual d-pad / live radar) on the
  LAN.

## Serial protocol (Teensy ⇄ laptop) — current

Plain text, newline-terminated, replies are `OK:...` / `ERR:...`:

- `TABLE1` / `TABLE2` — select turn direction at the next intersection
- `START` / `STOP` — begin/end autonomous line-following
- `MANUAL` — takeover; then `MFWD` / `MBACK` / `MLEFT` / `MRIGHT` / `MSTOP`
  drive motors directly (each resets the dead-man's-switch timer)
- Robot → laptop status lines: `FORWARD`, `LEFT`, `RIGHT`, `LINE_LOST`,
  `INTERSECTION:TURN_LEFT(Table1)` / `INTERSECTION:TURN_RIGHT(Table2)`,
  `READY` on boot
- `ODOM_RESET` — zero the encoder-based pose estimate (new, Phase 1)
- Robot → laptop: `ODOM:x_mm,y_mm,heading_deg` every ~100ms, unconditional
  (new, Phase 1; not yet consumed by `dashbord.py`)
- `PINS` — instantaneous digitalRead of all 4 encoder pins → `PINS:la,lb,ra,rb`
  (wiring sanity check; Phase 1)
- `TICKS` — raw encoder tick counts → `TICKS:left,right` (used to calibrate
  `TICKS_PER_REV`; Phase 1)
- `SENSORS` — raw QTR-8 readings → `SENSORS:v0,v1,...,v7`, independent of
  `running` state (used to check the `threshold` constant against the real
  track; added 2026-09-10, not tied to a phase)
- `PATH:<steps>` — "fake-autonomous" scripted playback: run a fixed,
  user-authored sequence of timed motion steps (e.g. forward 3s, then left
  2s, then hold 5s...). `<steps>` is `ACTION,DURATION_MS` pairs separated by
  `;` (ACTION one of `FWD`/`BACK`/`LEFT`/`RIGHT`/`HOLD`), built by the
  dashboard's PATH tab from a step list the user adds to. Open-loop/timed,
  not encoder closed-loop; mutually exclusive with line-following
  (`running`) and manual takeover — `START`/`MANUAL`/`STOP` all cancel it.
  Runs non-blocking out of `loop()` (`pathRunning`/`updatePath()`), so
  `STOP` still takes effect immediately mid-path. Replies
  `OK:PATH_STARTED:<n>`, then `PATH_STEP:<i>/<n>` per step, `PATH:DONE` at
  the end, `ERR:PATH_EMPTY` if `<steps>` parsed to nothing. Added
  2026-09-10, not tied to the SLAM roadmap below (this is a stopgap, not a
  step toward it).
- `PATH_STOP` — abort path playback early → `OK:PATH_STOPPED`

## Autonomous navigation feature — design decisions (2026-09-09)

Goal: LiDAR-based mapping, click a goal on the map, robot drives there
autonomously, then returns to its starting position. Decisions locked in
with the user:

1. **Split**: laptop (`dashbord.py`) owns mapping, localization, and path
   planning; Teensy (`lineflow.ino`) only executes closed-loop motion
   primitives using the new wheel encoders. Matches the current split
   (LiDAR is already laptop-side).
2. **Map type**: persistent occupancy-grid SLAM — the robot builds and
   saves an actual map of the space across runs, not just a live/ephemeral
   obstacle radar. Bigger lift than dead-reckoning-only; needs scan
   matching / loop-closure handling and real tuning on hardware.
3. **Goal input**: user clicks a point directly on the dashboard's existing
   LiDAR radar/map canvas; the click is converted into a target pose in map
   coordinates.

### Planned phases

- [x] **Phase 1 — Encoder odometry on Teensy** (implemented 2026-09-09,
      **not yet verified on real hardware**): interrupt on each side's
      channel A (`RISING`), channel B read at that instant for direction,
      differential-drive dead-reckoning integrated every `loop()` iteration
      in `updateOdometry()`. Streams `ODOM:x_mm,y_mm,heading_deg` over
      serial every 100ms (`ODOM_INTERVAL_MS`), independent of
      running/manual mode. New command `ODOM_RESET` zeroes the pose (call
      it at the start of an autonomous run to mark the "return to here"
      point for Phase 5).
      - TUNE constants added at the top of `lineflow.ino`:
        `TICKS_PER_REV`, `WHEEL_DIAMETER_MM`, `WHEEL_TRACK_MM` — all
        placeholder values, **must be set from real specs/measurements**.
      - `INPUT_PULLUP` is used on all 4 encoder pins by default; switch to
        plain `INPUT` if the encoder boards already drive push-pull.
      - Direction sign (which way is "positive") is **unverified** — if
        driving forward reports shrinking/negative distance, flip the
        `++`/`--` in `leftEncoderISR`/`rightEncoderISR` for that side, or
        swap that side's A/B wiring.
      - Still open: confirm Teensy model (interrupt-on-any-pin was assumed
        available); dashboard (`dashbord.py`) does not parse `ODOM:` lines
        yet — that's part of Phase 3.
- [ ] **Phase 2 — Closed-loop motion primitives on Teensy**: replace
      open-loop timed turns (`doIntersectionTurn`'s delay-based approach)
      with encoder-feedback primitives the laptop can call, e.g.
      `DRIVE:<mm>` and `TURN:<deg>`, each reporting `DONE` when complete.
      Needs PID (or simpler P) tuning per side using encoder velocity.
- [ ] **Phase 3 — Laptop-side occupancy-grid SLAM**: fuse Teensy odometry
      with RPLidar scans (`dashbord.py` already has raw scan points in
      `scan_points`) into a persistent occupancy grid; save/load the map to
      disk. Pick a concrete approach (e.g. simple grid + odometry with
      periodic scan-matching correction, or an existing lightweight SLAM
      lib) — open decision, revisit before starting this phase.
- [ ] **Phase 4 — Path planning + goal UI**: click-to-goal on the radar
      canvas → map coordinates; A* (or similar) over the occupancy grid;
      convert the path into a sequence of `DRIVE`/`TURN` commands sent to
      the Teensy; replan if LiDAR sees a new obstacle mid-path.
- [ ] **Phase 5 — Return-to-start**: record the robot's pose at the moment
      an autonomous run begins; after reaching the goal (or on command),
      replan a path back to that stored pose using the same Phase 4 planner.

### Open questions to resolve before/while implementing

- Encoder specs (CPR, wheel diameter, track width) — placeholders sit in
  `lineflow.ino` (`TICKS_PER_REV`, `WHEEL_DIAMETER_MM`, `WHEEL_TRACK_MM`),
  need real values from the user/datasheet + measurement.
- Encoder direction sign per side — unverified until tested on hardware
  (see Phase 1 notes above).
- Teensy model in use (affects available interrupt pins / timers) — assumed
  Teensy 4.x based on existing pin usage; confirm if not.
- SLAM approach for Phase 3 (custom vs. library) — not yet decided.
- How autonomous-nav mode interacts with existing line-following
  (`START`/`TABLE1`/`TABLE2`) and manual-takeover modes — likely a new
  top-level mode alongside `running`/`manualMode`, needs a name and clear
  precedence rules.

## Working conventions

- Firmware constants marked `TUNE` in `lineflow.ino` are placeholders meant
  to be re-tuned on the real chassis, not trusted as final.
- Keep the serial protocol plain-text and backward compatible where
  possible — the phone remote and dashboard both parse it directly.
- This file should be updated at the end of any session that changes
  hardware, the serial protocol, or the roadmap/phase status above.
