#include <Wire.h>

// Declared up here, before anything else, because the Arduino IDE
// auto-generates function prototypes and inserts them right after the
// last #include/comment block - if this enum were declared further down
// (where it's used), those auto-generated prototypes would reference it
// before its definition and fail to compile ("'PathAction' was not
// declared in this scope"). See the PATH PLAYBACK section below for its use.
enum PathAction { PATH_FWD, PATH_BACK, PATH_LEFT, PATH_RIGHT, PATH_HOLD,
                   PATH_TURN_LEFT, PATH_TURN_RIGHT };

// =====================================================================
// All global state lives up here in one block, before any function
// bodies. Arduino auto-generates FUNCTION prototypes so functions can
// freely call each other regardless of definition order below - but it
// does NOT do that for plain variables, so those must textually appear
// before any function that reads/writes them. Keeping every variable
// declaration up front sidesteps that ordering trap entirely.
// =====================================================================

// ---- Cytron MDD10A ----
#define PWM_LEFT  2
#define DIR_LEFT  3
#define PWM_RIGHT 4
#define DIR_RIGHT 5

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

int speed      = 90;    // normal drive speed
int nudgeSpeed = 85;    // slow-side speed while turning

// ---- Manual drive (phone/dashboard takeover) ----
// manualMode -> true after a "MANUAL" command; MFWD/MBACK/MLEFT/MRIGHT drive
//   the motors directly. A "STOP" command always clears it.
// lastManualCmdTime + MANUAL_TIMEOUT_MS is a dead-man's switch: if the phone
//   loses the connection mid-drive, the motors auto-stop shortly after.
bool manualMode = false;
unsigned long lastManualCmdTime = 0;
const unsigned long MANUAL_TIMEOUT_MS = 400;

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
//    left()/right() functions) - it DOES matter for HEADING_HOLD below: if
//    the robot's drift correction makes veering worse instead of better,
//    flip the sign of HEADING_KP.
#define MPU_ADDR 0x68
const double GYRO_SENS_LSB_PER_DPS = 131.0;  // datasheet value for +-250 deg/s (GYRO_CONFIG=0x00)

bool   mpuReady = false;
double currentHeadingDeg = 0.0;      // free-running integrated yaw; drifts slowly over minutes,
                                      // fine for single turns/steps that only last a few seconds
double gyroZBiasDegPerSec = 0.0;     // measured once at boot while stationary
unsigned long lastHeadingUpdateUs = 0;

// ---- Heading-hold (steering correction during FWD/BACK) ----
// Simple P controller, no integral/derivative - good enough for short
// steps, TUNE HEADING_KP on real hardware. Set to false to disable
// entirely without removing the code, in case it misbehaves before
// HEADING_KP's sign is confirmed on hardware (see the MPU6050 note above).
#define HEADING_HOLD_ENABLED true
double HEADING_KP = 1.2;               // PWM counts of correction per degree of drift - TUNE
const int HEADING_MAX_CORRECTION = 25; // clamp so correction can't overpower the base speed - TUNE

double pathStepStartHeadingDeg = 0.0;  // currentHeadingDeg snapshot at the start of the active step

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
#define PATH_TURN_TIMEOUT_MS 8000  // safety cap per turn in case the gyro/mount is bad - TUNE.
                                    // Must comfortably exceed a real 180 degree
                                    // turn at nudgeSpeed, or the return trip's
                                    // U-turns get cut short by the timeout.

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
  analogWrite(PWM_LEFT, nudgeSpeed);
  analogWrite(PWM_RIGHT, speed);
}

void left() {
  digitalWrite(DIR_LEFT, HIGH);
  digitalWrite(DIR_RIGHT, LOW);
  analogWrite(PWM_LEFT, speed);
  analogWrite(PWM_RIGHT, nudgeSpeed);
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
  analogWrite(PWM_LEFT, speed);
  analogWrite(PWM_RIGHT, speed);
}

// ---- MPU6050 gyro ----
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t mpuReadGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);  // GYRO_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2);
  if (Wire.available() < 2) return 0;
  int16_t raw = (Wire.read() << 8);
  raw |= Wire.read();
  return raw;
}

void setupMPU() {
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);  // WHO_AM_I
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  uint8_t whoAmI = Wire.available() ? Wire.read() : 0x00;
  mpuReady = (whoAmI == 0x68);
  if (!mpuReady) {
    Serial.println("ERR:MPU6050_NOT_FOUND");
    return;
  }

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
// While a PATH FWD/BACK step is driving, nudges the two wheels' PWM apart
// proportionally to how far currentHeadingDeg has drifted from where it was
// when the step started, to counter motor/friction asymmetry and drive
// straighter over a long step.
void maintainHeadingHold() {
  if (!HEADING_HOLD_ENABLED || !mpuReady) return;
  PathAction action = pathActions[pathStepIndex];
  if (action != PATH_FWD && action != PATH_BACK) return;

  double error = currentHeadingDeg - pathStepStartHeadingDeg;
  int correction = (int)constrain(error * HEADING_KP, -HEADING_MAX_CORRECTION, HEADING_MAX_CORRECTION);

  int leftPWM  = constrain(speed - correction, 0, 255);
  int rightPWM = constrain(speed + correction, 0, 255);
  analogWrite(PWM_LEFT, leftPWM);
  analogWrite(PWM_RIGHT, rightPWM);
}

// ---- Path playback ----
void applyPathAction(PathAction action) {
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
  manualMode = false;
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
  unsigned long elapsed = millis() - pathStepStartTime;

  bool stepDone;
  if (isTurn) {
    double turnedDeg = fabs(currentHeadingDeg - pathStepStartHeadingDeg);
    stepDone = (turnedDeg >= (double)pathDurations[pathStepIndex]) || (elapsed >= PATH_TURN_TIMEOUT_MS);
  } else {
    stepDone = (elapsed >= pathDurations[pathStepIndex]);
  }
  if (!stepDone) return;

  if (isTurn) stopBot();  // brief brake to arrest rotation before continuing

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
//   HEADING_RESET, GYRO  (MPU6050 diagnostics)
//   FOOD  (IR food-tray sensor state)
void processCommand(String cmd) {
  cmd.trim();

  if (cmd == "STOP") {
    manualMode = false;
    stopPath();  // also stops the motors
    Serial.println("OK:STOPPED");
  } else if (cmd == "MANUAL") {
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
      if (cmd == "MFWD") forward();
      else if (cmd == "MBACK") reverseBot();
      else if (cmd == "MLEFT") left();
      else if (cmd == "MRIGHT") right();
      else stopBot();  // MSTOP
      Serial.print("OK:");
      Serial.println(cmd);
    }
  } else if (cmd == "FOOD") {
    // Diagnostic / resync: the dashboard asks once on connect so it knows
    // the tray state without waiting for the next transition.
    Serial.println(foodPresent ? "FOOD:PRESENT" : "FOOD:ABSENT");
  } else if (cmd == "HEADING_RESET") {
    currentHeadingDeg = 0.0;
    Serial.println("OK:HEADING_RESET");
  } else if (cmd == "GYRO") {
    // Diagnostic: check MPU6050 wiring/sign on real hardware. Rotate the
    // chassis by hand and confirm this value changes sensibly (and that it
    // holds roughly steady, not drifting fast, while stationary).
    Serial.print("GYRO:");
    Serial.print(mpuReady ? "OK" : "NOT_FOUND");
    Serial.print(",");
    Serial.println(currentHeadingDeg, 1);
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

  // Both run unconditionally - heading tracking must never miss rotation,
  // and the tray is emptied while the robot is parked at the table.
  updateHeading();
  updateFoodSensor();

  if (manualMode) {
    // Motors are driven directly by MFWD/MBACK/MLEFT/MRIGHT above; this is
    // just the dead-man's switch in case the phone/dashboard goes quiet.
    if (millis() - lastManualCmdTime > MANUAL_TIMEOUT_MS) {
      stopBot();
    }
    return;
  }

  if (pathRunning) {
    if (!pathPaused) {
      maintainHeadingHold();  // no-op unless the active step is FWD/BACK
      updatePath();           // paused: motors already off, hold position in the step sequence
    }
    return;
  }

  stopBot();  // idle - not in manual mode or running a path
}
