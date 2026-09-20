#include <Wire.h>

// Declared up here, before anything else, because the Arduino IDE
// auto-generates function prototypes and inserts them right after the
// last #include/comment block - if this enum were declared further down
// (where it's used), those auto-generated prototypes would reference it
// before its definition and fail to compile ("'PathAction' was not
// declared in this scope"). See the PATH PLAYBACK section below for its use.
enum PathAction { PATH_FWD, PATH_BACK, PATH_LEFT, PATH_RIGHT, PATH_HOLD,
                   PATH_TURN_LEFT, PATH_TURN_RIGHT, PATH_ALIGN };
// Same reason - see the manual drive section for its use.
enum ManualMotion { MM_STOP, MM_FWD, MM_BACK, MM_LEFT, MM_RIGHT };

// =====================================================================
// All global state lives up here in one block, before any function
// bodies. Arduino auto-generates FUNCTION prototypes so functions can
// freely call each other regardless of definition order below - but it
// does NOT do that for plain variables, so those must textually appear
// before any function that reads/writes them. Keeping every variable
// declaration up front sidesteps that ordering trap entirely.
// =====================================================================

// ---- Cytron MDD10A ----
// Moved off pins 2/3/4/5 on 2026-09-20: the common ground to the driver came
// loose while driving, motor current found its way back through the logic
// side, and PINTEST then showed all four no longer reaching 3.3V. **Pins 2,
// 3, 4 and 5 are dead - don't reuse them for anything.**
//
// If a pin dies again, just move it: nothing else reads these numbers.
// Remaining spares are 9, 15, 20, 21, 22 (of those 9, 15 and 22 can do PWM;
// a DIR pin only needs plain digital output, so any spare will do).
// Re-flash after changing, and move the wire to match.
#define PWM_LEFT  7
#define DIR_LEFT  16
#define PWM_RIGHT 8
#define DIR_RIGHT 17

// ---- IR food-tray sensor ----
// A cheap IR obstacle/proximity module (FC-51 style) aimed across the food
// tray: something on the tray reflects IR back and the module pulls its
// OUT pin LOW; an empty tray reads HIGH (INPUT_PULLUP keeps it HIGH when
// the sensor is unplugged, so "no sensor" reads as "tray empty" rather
// than a stuck "food present").
//
// Wiring: OUT to Teensy pin 6, GND to GND, VCC to *3.3V* - Teensy 4.x
// digital pins are NOT 5V tolerant, so powering the module from 5V (its
// OUT then swings to 5V) needs a divider or level shifter first.
//
// Some modules are wired the other way round (HIGH = object detected).
// If the dashboard shows "EMPTY" with food on the tray and "FOOD LOADED"
// without, flip IR_FOOD_PRESENT_STATE to HIGH - that's the only change
// needed, everything downstream keys off foodPresent.
#define IR_FOOD_PIN 6
#define IR_FOOD_PRESENT_STATE LOW
const unsigned long FOOD_DEBOUNCE_MS = 250;  // ignore flicker as a hand passes over the tray

bool          foodPresent = false;       // debounced tray state
bool          foodRawLast = false;       // last raw read, for debounce timing
unsigned long foodRawChangeTime = 0;

int speed        = 90;    // normal forward drive speed
int nudgeSpeed   = 85;    // slow-side speed while turning
int reverseSpeed = 110;   // reverseBot()/left()/right() run harder than forward -
                          // heading-hold has to steer around the same number or it
                          // would quietly slow reversing down to `speed`

// ---- Manual drive (phone/dashboard takeover) ----
// manualMode -> true after a "MANUAL" command; MFWD/MBACK/MLEFT/MRIGHT drive
//   the motors directly. A "STOP" command always clears it.
// lastManualCmdTime + MANUAL_TIMEOUT_MS is a dead-man's switch: if the phone
//   loses the connection mid-drive, the motors auto-stop shortly after.
bool manualMode = false;
unsigned long lastManualCmdTime = 0;
const unsigned long MANUAL_TIMEOUT_MS = 400;

// What the motors are actually doing under manual control, and the heading
// when that started. The dashboard re-sends a held button every 150ms, so a
// repeat of the same command is the *same* segment - only a change counts.
// Tracking segments here (not on the Pi) means they reflect what the motors
// really did, dead-man's-switch stops included, with the heading measured at
// the exact moment the motion changed rather than whenever a report arrived.
//  - MFWD/MBACK hold the segment's starting heading (heading-hold, as in PATH
//    FWD/BACK steps), so manual driving goes straight too.
//  - When a MLEFT/MRIGHT segment ends, the rotation it produced is reported as
//    MTURN:<L|R>,<deg>, which the dashboard's recorder uses to store the turn
//    as a gyro-measured TURN LEFT/RIGHT instead of a timed LEFT/RIGHT.
ManualMotion manualMotion = MM_STOP;
double manualSegStartHeadingDeg = 0.0;

// ---- Live heading stream (the dashboard's gyro dial) ----
// HDG:<heading>,<gyroRightSign>,<anchor> at 10 Hz whenever the gyro is up.
// Teensy USB serial ignores the baud rate, so this costs nothing on the link.
const unsigned long HEADING_REPORT_MS = 100;
unsigned long lastHeadingReportMs = 0;

String inputBuffer = "";

// ---- MPU6050 gyro (heading via yaw-rate integration) ----
// Minimal raw-I2C driver (just Wire.h - no extra library to install).
// Only the Z-axis gyro is used, to track heading (yaw). The accelerometer
// is not read at all - this is heading-only, not a full IMU/AHRS.
//
// Wiring: MPU6050/GY-521 breakout - VCC to Teensy 3.3V (safest; most
// GY-521 boards also tolerate 5V via an onboard regulator, check yours),
// GND to GND, SDA to Teensy pin 18, SCL to Teensy pin 19 (Teensy's default
// Wire bus - these pins were freed up when the QTR line sensor was removed
// and were not otherwise in use). AD0 left unconnected -> I2C address 0x68.
//
// NOT VERIFIED on real hardware yet:
//  - Which physical direction is "increasing currentHeadingDeg" depends on
//    which way the board is mounted (flipping it flips the sign). This does
//    NOT affect the TURNL/TURNR feature (it stops on the *magnitude* of
//    rotation, so direction is already correct via the existing/tuned
//    left()/right() functions). Heading-hold and ALIGN do need it, and
//    learn it from the robot's own turns (gyroRightSign, below) rather than
//    assuming - nothing to flip by hand.
#define MPU_ADDR 0x68
const double GYRO_SENS_LSB_PER_DPS = 131.0;  // datasheet value for +-250 deg/s (GYRO_CONFIG=0x00)

bool    mpuReady = false;
uint8_t mpuAddr = MPU_ADDR;   // 0x68, or 0x69 when the breakout ties AD0 high
uint8_t mpuWhoAmI = 0x00;     // what the chip reported - clones aren't 0x68
double currentHeadingDeg = 0.0;      // free-running integrated yaw; drifts slowly over minutes,
                                      // fine for single turns/steps that only last a few seconds
double gyroZBiasDegPerSec = 0.0;     // measured once at boot while stationary
unsigned long lastHeadingUpdateUs = 0;

// ---- Heading-hold (steering correction while driving straight) ----
// Simple P controller, no integral/derivative - good enough for short
// straight runs. Applies to PATH FWD/BACK steps and to manual MFWD/MBACK.
// HEADING_KP is a magnitude only: which way to steer comes from the learned
// gyroRightSign, and heading-hold stays off until a turn has taught it (the
// first turn after boot, manual or scripted) - no correction at all beats a
// correction in the wrong direction. Set to false to disable entirely.
#define HEADING_HOLD_ENABLED true
double HEADING_KP = 1.2;               // PWM counts of correction per degree of drift - TUNE (keep > 0)
const int HEADING_MAX_CORRECTION = 25; // clamp so correction can't overpower the base speed - TUNE

double pathStepStartHeadingDeg = 0.0;  // currentHeadingDeg snapshot at the start of the active step

// ---- Absolute heading anchor (ALIGN steps) ----
// TURNL/TURNR only measure rotation *within* one step, so each turn's small
// error (stopping a few degrees early/late, a timed LEFT/RIGHT overshooting,
// drift while driving) accumulates over a route with nothing to correct it.
// HEADING_ANCHOR latches "this is the heading I started the delivery on" and
// survives across separate PATH commands; an ALIGN step then turns until the
// robot is back on that heading (plus an optional offset), wiping out
// everything that accumulated in between. The dashboard anchors when it
// starts a delivery and ends the return trip with ALIGN,0, so the robot
// parks facing exactly the way it left rather than a few degrees off each
// round trip.
//
// The anchor is only as good as the gyro underneath it: integration drift
// (roughly a degree or two per minute) rides on top of the stored value, so
// a long stand at the table eats into the accuracy. Still far better than
// letting per-turn error pile up uncorrected.
double anchorHeadingDeg = 0.0;
double alignTargetDeg = 0.0;           // anchorHeadingDeg + this step's offset

// Which way an ALIGN step has to spin depends on whether right() makes
// currentHeadingDeg rise or fall - i.e. on the gyro's sign convention, which
// depends on how the MPU6050 happens to be mounted and is NOT verified on
// this robot. Rather than assume, this is *learned* from the robot's own
// motion: any turn big enough to be unambiguous records the answer here, and
// an ALIGN step with no answer yet spins right() until it has one (see
// maintainAlign). +1 = right() increases the heading, -1 = decreases, 0 =
// not known yet. TURNL/TURNR never need it - they stop on magnitude.
int gyroRightSign = 0;
const double GYRO_SIGN_LEARN_DEG = 5.0;   // rotation needed before the sign is trustworthy

const double ALIGN_TOLERANCE_DEG = 3.0;      // close enough - tighter than this chases gyro noise
// (ALIGN's time limit is turnTimeoutFor() on the turn it's about to make,
// and with no gyro at all it ends at once rather than spinning blind.)

// ---- Path playback (scripted timed/turn moves) ----
// A user-authored sequence of steps sent from the dashboard, e.g. "forward
// 3s, then turn left 90 degrees, then hold 5s..." - most steps are
// open-loop/time-based, but TURNL/TURNR are closed-loop via the gyro above.
// Runs non-blocking out of loop() so handleSerial() keeps being polled and
// STOP/PATH_STOP take effect immediately instead of only after the whole
// path finishes.
// Headroom matters here: the dashboard's "return trip" sends the whole
// delivery path back mirrored plus two 180 degree turns, so a long
// recorded delivery needs roughly double its own step count.
#define PATH_MAX_STEPS 64

// A turn's timeout is a safety net for a dead gyro, NOT a limit on how long
// a real turn may take - so it scales with the angle asked for. A flat cap
// (8s for everything, up to 2026-09-20) quietly cut 180 degree U-turns short
// on this robot: they take longer than a 90, so the return trip set off
// still half-facing the table. 100ms per degree = a floor of 10 deg/s, well
// under anything the motors actually manage.
const unsigned long TURN_TIMEOUT_MS_PER_DEG = 100;
const unsigned long TURN_TIMEOUT_MIN_MS = 3000;
const unsigned long TURN_TIMEOUT_MAX_MS = 20000;
unsigned long stepTimeoutMs = 0;   // for the turn/align step running now

PathAction    pathActions[PATH_MAX_STEPS];
unsigned long pathDurations[PATH_MAX_STEPS];  // ms for timed actions, DEGREES for TURNL/TURNR
int           pathStepCount = 0;
int           pathStepIndex = -1;
unsigned long pathStepStartTime = 0;
bool          pathRunning = false;

// Set by PATH_PAUSE (e.g. the dashboard pausing for a LiDAR obstacle) and
// cleared by PATH_RESUME. Unlike PATH_STOP, this freezes progress in place
// (current step + elapsed time within it) instead of cancelling the path,
// so resuming continues the same step rather than restarting the path.
bool          pathPaused = false;
unsigned long pathPausedElapsedMs = 0;

// =====================================================================
// Functions. Order doesn't matter for calling each other (Arduino
// auto-generates prototypes for these), only relative to the variables
// above, which are already all declared by this point.
// =====================================================================

// ---- Motor functions ----
void forward() {
  digitalWrite(DIR_LEFT, HIGH);
  digitalWrite(DIR_RIGHT, HIGH);
  analogWrite(PWM_LEFT, speed);
  analogWrite(PWM_RIGHT, speed);
}

void right() {
  digitalWrite(DIR_LEFT, LOW);
  digitalWrite(DIR_RIGHT, HIGH);
  analogWrite(PWM_LEFT, 110);
  analogWrite(PWM_RIGHT, 110);
}

void left() {
  digitalWrite(DIR_LEFT, HIGH);
  digitalWrite(DIR_RIGHT, LOW);
  analogWrite(PWM_LEFT, 110);
  analogWrite(PWM_RIGHT, 110);
}

void stopBot() {
  analogWrite(PWM_LEFT, 0);
  analogWrite(PWM_RIGHT, 0);
}

// Mirrors forward()'s HIGH/HIGH = forward convention: LOW/LOW = reverse for
// this driver.
void reverseBot() {
  digitalWrite(DIR_LEFT, LOW);
  digitalWrite(DIR_RIGHT, LOW);
  analogWrite(PWM_LEFT, reverseSpeed);
  analogWrite(PWM_RIGHT, reverseSpeed);
}

// ---- MPU6050 gyro ----
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t mpuReadGyroZRaw() {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x47);  // GYRO_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(mpuAddr, 2);
  if (Wire.available() < 2) return 0;
  int16_t raw = (Wire.read() << 8);
  raw |= Wire.read();
  return raw;
}

// WHO_AM_I at the given address, or 0x00/0xFF when nothing answers.
uint8_t mpuReadWhoAmI(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x75);  // WHO_AM_I
  if (Wire.endTransmission(false) != 0) return 0x00;  // nobody acknowledged
  Wire.requestFrom(addr, 1);
  return Wire.available() ? Wire.read() : 0x00;
}

// Print every device answering on the bus - the quickest way to tell "wired
// wrong / not powered" (nothing found) from "wired fine, unexpected chip"
// (something found at 0x68 or 0x69).
void i2cScan() {
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C:0x");
      Serial.println(addr, HEX);
      found++;
    }
  }
  Serial.print("I2C:DONE:");
  Serial.println(found);
}

void setupMPU() {
  Wire.begin();
  mpuReady = false;  // re-probing must be able to conclude "gone", not keep a stale yes

  // Try both addresses: AD0 low is 0x68, but plenty of breakouts tie AD0
  // high (0x69), and a genuine MPU6050 answers 0x68 to WHO_AM_I while
  // MPU6500/9250-based clones sold as "MPU6050" answer 0x70/0x72/0x73.
  // Their gyro registers are identical, so accept any chip that answers
  // rather than refusing to run on the WHO_AM_I value alone.
  const uint8_t addresses[] = {0x68, 0x69};
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t who = mpuReadWhoAmI(addresses[i]);
    if (who != 0x00 && who != 0xFF) {
      mpuAddr = addresses[i];
      mpuWhoAmI = who;
      mpuReady = true;
      break;
    }
  }
  if (!mpuReady) {
    Serial.println("ERR:MPU6050_NOT_FOUND");
    i2cScan();  // so the log says whether anything is on the bus at all
    return;
  }
  Serial.print("OK:MPU:ADDR=0x");
  Serial.print(mpuAddr, HEX);
  Serial.print(",WHOAMI=0x");
  Serial.println(mpuWhoAmI, HEX);

  mpuWrite(0x6B, 0x00);  // PWR_MGMT_1: wake up (default power-on state is asleep)
  mpuWrite(0x1B, 0x00);  // GYRO_CONFIG: +-250 deg/s range, 131 LSB/(deg/s)

  // Calibrate: average the raw Z rate for ~400ms while presumed stationary.
  // Re-running HEADING_RESET later does NOT redo this - power-cycle (or add
  // a dedicated recalibrate command) if the bias drifts with temperature.
  delay(100);
  long sum = 0;
  const int N = 200;
  for (int i = 0; i < N; i++) {
    sum += mpuReadGyroZRaw();
    delay(2);
  }
  gyroZBiasDegPerSec = (sum / (double)N) / GYRO_SENS_LSB_PER_DPS;
  lastHeadingUpdateUs = micros();
}

// Call every loop() iteration, unconditionally, so heading tracking never
// misses rotation - needed for closed-loop TURNL/TURNR and for HEADING_HOLD.
void updateHeading() {
  if (!mpuReady) return;
  unsigned long nowUs = micros();
  double dt = (nowUs - lastHeadingUpdateUs) / 1000000.0;
  lastHeadingUpdateUs = nowUs;

  double dps = (mpuReadGyroZRaw() / GYRO_SENS_LSB_PER_DPS) - gyroZBiasDegPerSec;
  currentHeadingDeg += dps * dt;
}

// ---- IR food-tray sensor ----
// Call every loop() iteration, unconditionally (idle, manual or mid-path):
// the tray is emptied while the robot sits still at the table, which is
// exactly when loop() is doing nothing else.
//
// Only *transitions* are reported, so the Pi gets one event per pickup
// rather than a stream:
//   FOOD:TAKEN    present -> empty (the customer took the food)
//   FOOD:PRESENT  empty -> present (the tray was loaded)
// The initial state is latched in setup() without a transition event, and
// announced once after READY so the dashboard starts in sync.
void updateFoodSensor() {
  bool raw = (digitalRead(IR_FOOD_PIN) == IR_FOOD_PRESENT_STATE);

  if (raw != foodRawLast) {
    foodRawLast = raw;
    foodRawChangeTime = millis();
    return;
  }
  if (raw == foodPresent) return;                        // already settled here
  if (millis() - foodRawChangeTime < FOOD_DEBOUNCE_MS) return;

  foodPresent = raw;
  Serial.println(foodPresent ? "FOOD:PRESENT" : "FOOD:TAKEN");
}

// ---- Heading-hold ----
// While driving straight, nudges the two wheels' PWM apart proportionally to
// how far currentHeadingDeg has drifted from holdHeadingDeg, to counter
// motor/friction asymmetry and keep the line.
//
// Which way to nudge is derived from the motor functions, not assumed:
// a positive correction runs the right wheel faster than the left. Going
// forward that rotates the robot the same way right() does (right() drives
// the right wheel forward and the left one back), so it moves the heading by
// gyroRightSign. Reversing, both wheels run backwards and the same PWM split
// rotates the robot the *other* way - so the sign flips for BACK. (The old
// version used one sign for both, which made BACK steps amplify drift.)
void applyHeadingHold(double holdHeadingDeg, bool reversing) {
  if (!HEADING_HOLD_ENABLED || !mpuReady || gyroRightSign == 0) return;

  double error = currentHeadingDeg - holdHeadingDeg;
  int headingSignOfPositiveCorrection = reversing ? -gyroRightSign : gyroRightSign;
  // Steer to move the heading opposite to the error.
  int correction = (int)constrain(-error * fabs(HEADING_KP) * headingSignOfPositiveCorrection,
                                  -HEADING_MAX_CORRECTION, HEADING_MAX_CORRECTION);

  int base = reversing ? reverseSpeed : speed;  // keep the tuned speed for this direction
  analogWrite(PWM_LEFT, constrain(base - correction, 0, 255));
  analogWrite(PWM_RIGHT, constrain(base + correction, 0, 255));
}

// PATH FWD/BACK steps hold the heading they started on.
void maintainHeadingHold() {
  PathAction action = pathActions[pathStepIndex];
  if (action != PATH_FWD && action != PATH_BACK) return;
  applyHeadingHold(pathStepStartHeadingDeg, action == PATH_BACK);
}

// ---- Manual drive segments ----
// Called for every manual drive command (and by the dead-man's switch). A
// repeat of the held button is ignored; a real change closes the previous
// segment and opens the next one at the current heading.
void setManualMotion(ManualMotion next) {
  if (next == manualMotion) return;
  endManualSegment();
  manualMotion = next;
  manualSegStartHeadingDeg = currentHeadingDeg;
}

// A manual turn just ended: learn the gyro sign from it (the same way PATH
// turns do), and tell the dashboard how far it really rotated. Measured at
// motor-off, matching how a TURNL/TURNR step decides when to stop, so a
// recorded turn plays back to the same angle.
void endManualSegment() {
  if (manualMotion != MM_LEFT && manualMotion != MM_RIGHT) return;
  double deltaDeg = currentHeadingDeg - manualSegStartHeadingDeg;
  learnGyroSign(deltaDeg, manualMotion == MM_RIGHT);
  if (!mpuReady) return;  // no gyro: the recorder keeps the timed step
  Serial.print(manualMotion == MM_RIGHT ? "MTURN:R," : "MTURN:L,");
  Serial.println(fabs(deltaDeg), 1);
}

// Leaving manual mode for any reason (STOP, a PATH starting) must still close
// a turn that was in progress, or its MTURN report would be lost.
void leaveManualMode() {
  if (manualMode) setManualMotion(MM_STOP);
  manualMode = false;
}

// ---- Live heading stream ----
void reportHeading() {
  if (!mpuReady) return;
  if (millis() - lastHeadingReportMs < HEADING_REPORT_MS) return;
  lastHeadingReportMs = millis();
  Serial.print("HDG:");
  Serial.print(currentHeadingDeg, 1);
  Serial.print(",");
  Serial.print(gyroRightSign);
  Serial.print(",");
  Serial.println(anchorHeadingDeg, 1);
}

// ---- Absolute heading alignment ----
// Wrap to (-180, 180] so "turn to face X" always takes the short way round:
// an error of 350 degrees is really 10 degrees the other way.
double normalizeDeg180(double deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg <= -180.0) deg += 360.0;
  return deg;
}

double alignErrorDeg() {
  return normalizeDeg180(alignTargetDeg - currentHeadingDeg);
}

// Record which way right() moves the heading, from a turn the robot just
// made. Called with the rotation observed across a completed TURNL/TURNR
// step, or mid-ALIGN once it has spun far enough to be sure.
void learnGyroSign(double deltaDeg, bool wasTurningRight) {
  if (fabs(deltaDeg) < GYRO_SIGN_LEARN_DEG) return;
  int signOfDelta = (deltaDeg > 0) ? 1 : -1;
  gyroRightSign = wasTurningRight ? signOfDelta : -signOfDelta;
}

// Drives an active ALIGN step, re-evaluated every loop() iteration (unlike
// the other actions, which are set once in applyPathAction and left alone) -
// the required direction can only be known while the error is being watched.
void maintainAlign() {
  if (!pathRunning || pathPaused) return;
  if (pathActions[pathStepIndex] != PATH_ALIGN) return;
  if (!mpuReady) return;  // no gyro: updatePath ends the step on its timeout

  if (gyroRightSign == 0) {
    // Sign unknown: spin right() and watch. Worst case this is the wrong
    // way by GYRO_SIGN_LEARN_DEG, which the aligning below then corrects.
    learnGyroSign(currentHeadingDeg - pathStepStartHeadingDeg, true);
    right();
    return;
  }

  double error = alignErrorDeg();
  if (fabs(error) <= ALIGN_TOLERANCE_DEG) {
    stopBot();  // updatePath sees the same condition and advances the step
    return;
  }
  // error > 0 means the heading must increase to reach the target.
  bool spinRight = (error > 0) == (gyroRightSign > 0);
  if (spinRight) right();
  else left();
}

// ---- Path playback ----
// How long to allow a turn of this many degrees before giving up on it.
unsigned long turnTimeoutFor(double degrees) {
  unsigned long ms = (unsigned long)(fabs(degrees) * TURN_TIMEOUT_MS_PER_DEG);
  return constrain(ms, TURN_TIMEOUT_MIN_MS, TURN_TIMEOUT_MAX_MS);
}

void applyPathAction(PathAction action) {
  if (action == PATH_TURN_LEFT || action == PATH_TURN_RIGHT) {
    stepTimeoutMs = turnTimeoutFor((double)pathDurations[pathStepIndex]);
  }
  if (action == PATH_ALIGN) {
    // Target is relative to the anchor, not to wherever this step begins -
    // that's the whole point: it's an absolute heading, so accumulated error
    // from every step before it gets corrected here rather than carried on.
    alignTargetDeg = anchorHeadingDeg + (double)pathDurations[pathStepIndex];
    // Budget for the turn it is actually about to make, not a flat cap.
    stepTimeoutMs = turnTimeoutFor(alignErrorDeg());
    stopBot();  // maintainAlign() takes over from the next iteration
    return;
  }
  switch (action) {
    case PATH_FWD:         forward();    break;
    case PATH_BACK:         reverseBot(); break;
    case PATH_LEFT:         left();       break;
    case PATH_RIGHT:        right();      break;
    case PATH_TURN_LEFT:    left();       break;  // same motion as PATH_LEFT, stops by angle not time
    case PATH_TURN_RIGHT:   right();      break;
    default:                stopBot();    break;  // PATH_HOLD - pause in place
  }
}

void stopPath() {
  pathRunning = false;
  pathPaused = false;
  pathStepIndex = -1;
  stopBot();
}

void pausePath() {
  if (!pathRunning || pathPaused) return;
  pathPausedElapsedMs = millis() - pathStepStartTime;
  pathPaused = true;
  stopBot();
}

void resumePath() {
  if (!pathRunning || !pathPaused) return;
  pathStepStartTime = millis() - pathPausedElapsedMs;
  applyPathAction(pathActions[pathStepIndex]);
  pathPaused = false;
}

// body is everything after "PATH:", steps separated by ';', each step
// "ACTION,VALUE" e.g. "FWD,3000;TURNL,90;RIGHT,2000;HOLD,5000". VALUE is
// milliseconds for FWD/BACK/LEFT/RIGHT/HOLD, degrees for TURNL/TURNR.
void startPath(String body) {
  pathStepCount = 0;
  int start = 0;
  while (start < (int)body.length() && pathStepCount < PATH_MAX_STEPS) {
    int sep = body.indexOf(';', start);
    String token = (sep == -1) ? body.substring(start) : body.substring(start, sep);
    token.trim();
    int comma = token.indexOf(',');
    if (comma != -1) {
      String actionStr = token.substring(0, comma);
      long value = token.substring(comma + 1).toInt();
      PathAction action;
      if (actionStr == "FWD") action = PATH_FWD;
      else if (actionStr == "BACK") action = PATH_BACK;
      else if (actionStr == "LEFT") action = PATH_LEFT;
      else if (actionStr == "RIGHT") action = PATH_RIGHT;
      else if (actionStr == "TURNL") action = PATH_TURN_LEFT;
      else if (actionStr == "TURNR") action = PATH_TURN_RIGHT;
      else if (actionStr == "ALIGN") action = PATH_ALIGN;
      else action = PATH_HOLD;  // "HOLD" or anything unrecognized
      pathActions[pathStepCount] = action;
      pathDurations[pathStepCount] = (unsigned long)max(0L, value);
      pathStepCount++;
    }
    if (sep == -1) break;
    start = sep + 1;
  }

  if (pathStepCount == 0) {
    Serial.println("ERR:PATH_EMPTY");
    return;
  }

  // Path mode is mutually exclusive with manual takeover.
  leaveManualMode();
  pathRunning = true;
  pathStepIndex = 0;
  pathStepStartTime = millis();
  pathStepStartHeadingDeg = currentHeadingDeg;
  applyPathAction(pathActions[0]);
  Serial.print("OK:PATH_STARTED:");
  Serial.println(pathStepCount);
}

// Called every loop() iteration while pathRunning - advances to the next
// step once the current one's duration (or, for TURNL/TURNR, its target
// angle) has been reached.
void updatePath() {
  PathAction action = pathActions[pathStepIndex];
  bool isTurn = (action == PATH_TURN_LEFT || action == PATH_TURN_RIGHT);
  bool isAlign = (action == PATH_ALIGN);
  unsigned long elapsed = millis() - pathStepStartTime;

  bool stepDone;
  double turnedDeg = fabs(currentHeadingDeg - pathStepStartHeadingDeg);
  if (isTurn) {
    stepDone = (turnedDeg >= (double)pathDurations[pathStepIndex]) || (elapsed >= stepTimeoutMs);
  } else if (isAlign) {
    // maintainAlign() is doing the steering; this only decides when to move
    // on. With no gyro it can't work at all, so end it at once rather than
    // spin blind until a timeout.
    stepDone = !mpuReady || (fabs(alignErrorDeg()) <= ALIGN_TOLERANCE_DEG)
               || (elapsed >= stepTimeoutMs);
  } else {
    stepDone = (elapsed >= pathDurations[pathStepIndex]);
  }
  if (!stepDone) return;

  if (isTurn) {
    // Say how far it actually got, and whether it ran out of time getting
    // there - a turn that stops short otherwise looks like a gyro fault.
    Serial.print("TURN:");
    Serial.print(turnedDeg, 1);
    Serial.print("/");
    Serial.print(pathDurations[pathStepIndex]);
    Serial.print(",");
    Serial.print(elapsed);
    Serial.println(turnedDeg < (double)pathDurations[pathStepIndex] ? ",TIMEOUT" : "");
  }
  if (isTurn || action == PATH_LEFT || action == PATH_RIGHT) {
    // A deliberate turn of known direction is the one moment the gyro's sign
    // convention is observable for free - remember it for ALIGN steps and
    // heading-hold. Timed LEFT/RIGHT count too (older recorded paths).
    learnGyroSign(currentHeadingDeg - pathStepStartHeadingDeg,
                  action == PATH_TURN_RIGHT || action == PATH_RIGHT);
  }
  if (isTurn || isAlign) stopBot();  // brief brake to arrest rotation before continuing
  if (isAlign) {
    Serial.print("ALIGN:");
    Serial.println(alignErrorDeg(), 1);  // residual error - 0 +/- tolerance when it worked
  }

  pathStepIndex++;
  if (pathStepIndex >= pathStepCount) {
    stopPath();
    Serial.println("PATH:DONE");
    return;
  }
  pathStepStartTime = millis();
  pathStepStartHeadingDeg = currentHeadingDeg;
  applyPathAction(pathActions[pathStepIndex]);
  Serial.print("PATH_STEP:");
  Serial.print(pathStepIndex + 1);
  Serial.print("/");
  Serial.println(pathStepCount);
}

// ---- Serial command handling ----
// Dashboard sends plain newline-terminated text commands:
//   STOP
//   MANUAL, MFWD, MBACK, MLEFT, MRIGHT, MSTOP  (phone/manual takeover)
//   PATH:<steps>, PATH_STOP, PATH_PAUSE, PATH_RESUME  (scripted path playback)
//   HEADING_RESET, HEADING_ANCHOR, GYRO  (MPU6050 heading)
// and pushes, unprompted: HDG:<deg>,<sign>,<anchor> at 10 Hz (gyro dial),
// MTURN:<L|R>,<deg> when a manual turn ends (gyro-measured recording).
//   FOOD  (IR food-tray sensor state)
//   I2CSCAN, MTEST, PINTEST  (diagnostics: what's on the I2C bus / do the
//                             motors run / are the motor pins still alive)
void processCommand(String cmd) {
  cmd.trim();

  if (cmd == "STOP") {
    leaveManualMode();
    stopPath();  // also stops the motors
    Serial.println("OK:STOPPED");
  } else if (cmd == "MANUAL") {
    setManualMotion(MM_STOP);  // a re-sent MANUAL mid-turn still reports that turn
    manualMode = true;
    stopPath();  // also stops the motors
    lastManualCmdTime = millis();
    Serial.println("OK:MANUAL_MODE");
  } else if (cmd.startsWith("PATH:")) {
    startPath(cmd.substring(5));
  } else if (cmd == "PATH_STOP") {
    stopPath();
    Serial.println("OK:PATH_STOPPED");
  } else if (cmd == "PATH_PAUSE") {
    if (!pathRunning) {
      Serial.println("ERR:PATH_NOT_RUNNING");
    } else if (pathPaused) {
      Serial.println("ERR:PATH_ALREADY_PAUSED");
    } else {
      pausePath();
      Serial.println("OK:PATH_PAUSED");
    }
  } else if (cmd == "PATH_RESUME") {
    if (!pathRunning) {
      Serial.println("ERR:PATH_NOT_RUNNING");
    } else if (!pathPaused) {
      Serial.println("ERR:PATH_NOT_PAUSED");
    } else {
      resumePath();
      Serial.println("OK:PATH_RESUMED");
    }
  } else if (cmd == "MFWD" || cmd == "MBACK" || cmd == "MLEFT" || cmd == "MRIGHT" || cmd == "MSTOP") {
    if (!manualMode) {
      Serial.println("ERR:NOT_MANUAL");
    } else {
      lastManualCmdTime = millis();
      // Segment first, motors second: the turn that's ending is measured
      // before its motors are switched off/over.
      if (cmd == "MFWD")        { setManualMotion(MM_FWD);   forward(); }
      else if (cmd == "MBACK")  { setManualMotion(MM_BACK);  reverseBot(); }
      else if (cmd == "MLEFT")  { setManualMotion(MM_LEFT);  left(); }
      else if (cmd == "MRIGHT") { setManualMotion(MM_RIGHT); right(); }
      else                      { setManualMotion(MM_STOP);  stopBot(); }  // MSTOP
      Serial.print("OK:");
      Serial.println(cmd);
    }
  } else if (cmd == "FOOD") {
    // Diagnostic / resync: the dashboard asks once on connect so it knows
    // the tray state without waiting for the next transition.
    Serial.println(foodPresent ? "FOOD:PRESENT" : "FOOD:ABSENT");
  } else if (cmd == "HEADING_ANCHOR") {
    // Latch the current heading as the route's absolute reference. Sent by
    // the dashboard when a delivery starts; the return trip's ALIGN,0 step
    // then brings the robot back to exactly this heading.
    anchorHeadingDeg = currentHeadingDeg;
    Serial.print("OK:HEADING_ANCHOR:");
    Serial.println(anchorHeadingDeg, 1);
  } else if (cmd == "HEADING_RESET") {
    currentHeadingDeg = 0.0;
    anchorHeadingDeg = 0.0;  // the anchor is a heading too - zeroing one without
                              // the other would silently offset every ALIGN step
    Serial.println("OK:HEADING_RESET");
  } else if (cmd == "GYRO") {
    // Diagnostic: check MPU6050 wiring/sign on real hardware. Rotate the
    // chassis by hand and confirm this value changes sensibly (and that it
    // holds roughly steady, not drifting fast, while stationary).
    Serial.print("GYRO:");
    Serial.print(mpuReady ? "OK" : "NOT_FOUND");
    Serial.print(",");
    Serial.print(currentHeadingDeg, 1);
    Serial.print(",ADDR=0x");
    Serial.print(mpuAddr, HEX);
    Serial.print(",WHOAMI=0x");
    Serial.print(mpuWhoAmI, HEX);
    Serial.print(",SIGN=");
    Serial.println(gyroRightSign);
  } else if (cmd == "MTEST") {
    // Diagnostic: drive each side, each way, for a fixed burst - bypassing
    // manual mode, the dead-man's switch and path playback entirely. If the
    // wheels don't move for this, the problem is past the Teensy (driver
    // power, a missing common ground, or the motor wiring), not in the
    // command path. Deliberately blocking: nothing else should run during it
    // (so STOP can't interrupt either - keep the wheels off the ground).
    leaveManualMode();
    stopPath();
    Serial.println("OK:MTEST_START");
    const int testPwm = 150;  // well above the usual speeds, to beat friction
    for (int side = 0; side < 2; side++) {
      int pwmPin = side ? PWM_RIGHT : PWM_LEFT;
      int dirPin = side ? DIR_RIGHT : DIR_LEFT;
      for (int dir = 0; dir < 2; dir++) {
        Serial.print("MTEST:");
        Serial.print(side ? "RIGHT_" : "LEFT_");
        Serial.println(dir ? "BACK" : "FWD");
        digitalWrite(dirPin, dir ? LOW : HIGH);
        analogWrite(pwmPin, testPwm);
        delay(600);
        analogWrite(pwmPin, 0);
        delay(300);
      }
    }
    stopBot();
    Serial.println("MTEST:DONE");
  } else if (cmd == "PINTEST") {
    // Diagnostic: hold each motor-signal pin HIGH for long enough to read it
    // with a multimeter. A healthy Teensy pin sits at 3.3V; a pin damaged by
    // motor current finding its way back through the logic side (what a lost
    // common ground causes) reads near 0V, or sags well under 3.3V.
    // Unplug the four signal wires from the driver first, so what's measured
    // is the pin itself and nothing else.
    leaveManualMode();
    stopPath();
    Serial.println("OK:PINTEST_START");
    const int pins[] = {PWM_LEFT, DIR_LEFT, PWM_RIGHT, DIR_RIGHT};
    const char* names[] = {"PWM_LEFT(pin2)", "DIR_LEFT(pin3)",
                            "PWM_RIGHT(pin4)", "DIR_RIGHT(pin5)"};
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) digitalWrite(pins[j], LOW);
      digitalWrite(pins[i], HIGH);
      Serial.print("PINTEST:");
      Serial.print(names[i]);
      Serial.println("=HIGH,expect 3.3V for 3s");
      delay(3000);
    }
    for (int j = 0; j < 4; j++) digitalWrite(pins[j], LOW);
    stopBot();
    Serial.println("PINTEST:DONE");
  } else if (cmd == "I2CSCAN") {
    // Diagnostic: what's actually on the I2C bus right now.
    i2cScan();
  } else if (cmd.length() > 0) {
    Serial.print("ERR:UNKNOWN_CMD:");
    Serial.println(cmd);
  }
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

// ---- Setup ----
void setup() {
  Serial.begin(9600);

  pinMode(PWM_LEFT, OUTPUT);
  pinMode(DIR_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);

  // Latch the tray's starting state without emitting a transition event -
  // booting with an empty tray is not a pickup.
  pinMode(IR_FOOD_PIN, INPUT_PULLUP);
  foodPresent = (digitalRead(IR_FOOD_PIN) == IR_FOOD_PRESENT_STATE);
  foodRawLast = foodPresent;
  foodRawChangeTime = millis();

  setupMPU();  // safe to fail (ERR:MPU6050_NOT_FOUND) - manual drive and
               // time-based PATH steps still work fine without it; only
               // TURNL/TURNR and HEADING_HOLD need it.

  delay(1000);
  Serial.println("READY");
  Serial.println(foodPresent ? "FOOD:PRESENT" : "FOOD:ABSENT");
}

// ---- Loop ----
void loop() {
  // Always listen for dashboard commands, even while idle.
  handleSerial();

  // All run unconditionally - heading tracking must never miss rotation,
  // the tray is emptied while the robot is parked at the table, and the
  // dashboard's gyro dial should move even when the robot is turned by hand.
  updateHeading();
  updateFoodSensor();
  reportHeading();

  if (manualMode) {
    // Motors are driven directly by MFWD/MBACK/MLEFT/MRIGHT above. Here:
    // the dead-man's switch in case the phone/dashboard goes quiet, and
    // keeping a held forward/back straight.
    if (manualMotion != MM_STOP && millis() - lastManualCmdTime > MANUAL_TIMEOUT_MS) {
      setManualMotion(MM_STOP);  // closes (and reports) a turn cut off by the timeout
      stopBot();
    } else if (manualMotion == MM_FWD || manualMotion == MM_BACK) {
      applyHeadingHold(manualSegStartHeadingDeg, manualMotion == MM_BACK);
    }
    return;
  }

  if (pathRunning) {
    if (!pathPaused) {
      maintainHeadingHold();  // no-op unless the active step is FWD/BACK
      maintainAlign();        // no-op unless the active step is ALIGN
      updatePath();           // paused: motors already off, hold position in the step sequence
    }
    return;
  }

  stopBot();  // idle - not in manual mode or running a path
}
