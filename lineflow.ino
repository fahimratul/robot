#include <QTRSensors.h>

// ================= QTR SENSOR =================
QTRSensors qtr;
uint16_t sensorValues[8];
uint8_t sensorPins[8] = {22, 21, 20, 19, 18, 17, 16, 15};

int threshold = 2000;   // black threshold

// ================= CYTRON MDD10A =================
#define PWM_LEFT  2
#define DIR_LEFT  3
#define PWM_RIGHT 4
#define DIR_RIGHT 5

int speed      = 60;    // normal forward speed
int nudgeSpeed = 50;    // slow-side speed while correcting on the line

// ================= WHEEL ENCODERS / ODOMETRY =================
#define ENC_LEFT_A  6
#define ENC_LEFT_B  7
#define ENC_RIGHT_A 8
#define ENC_RIGHT_B 9

// TUNE: set these from your encoder/gearbox datasheet and a physical
// measurement of the chassis before trusting the odometry output.
//   TICKS_PER_REV   = encoder counts per one WHEEL revolution (this code
//                     counts one tick per rising edge of channel A, so this
//                     is encoder-PPR * gearbox-ratio, NOT the raw encoder
//                     datasheet PPR if there's a gearbox in between).
//   WHEEL_DIAMETER_MM = outer diameter of the wheel touching the ground.
//   WHEEL_TRACK_MM    = center-to-center distance between the two wheels.
double TICKS_PER_REV     = 360.0;   // TUNE - placeholder
double WHEEL_DIAMETER_MM = 120.0;    // TUNE - placeholder
double WHEEL_TRACK_MM    = 310.0;   // TUNE - placeholder

volatile long leftTicks  = 0;
volatile long rightTicks = 0;

// Robot pose estimate, in the frame where (0,0,0) = position/heading at
// boot or at the last ODOM_RESET.
double robotX_mm     = 0.0;
double robotY_mm     = 0.0;
double robotHeading  = 0.0;   // radians

long lastLeftTicks  = 0;
long lastRightTicks = 0;
unsigned long lastOdomPrint = 0;
const unsigned long ODOM_INTERVAL_MS = 100;

// Reads channel B at the moment channel A rises to get direction.
// NOT VERIFIED on real hardware yet - if forward driving reports negative
// distance, swap the ISR's ++/-- (or swap the A/B wiring for that side).
void leftEncoderISR() {
  if (digitalRead(ENC_LEFT_B)) leftTicks++;
  else leftTicks--;
}
void rightEncoderISR() {
  if (digitalRead(ENC_RIGHT_B)) rightTicks--;
  else rightTicks++;
}

void resetOdometry() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
  lastLeftTicks = 0;
  lastRightTicks = 0;
  robotX_mm = 0.0;
  robotY_mm = 0.0;
  robotHeading = 0.0;
}

// Differential-drive dead-reckoning: called every loop() iteration
// (independent of running/manualMode, so odometry never misses motion).
// Prints ODOM:x_mm,y_mm,heading_deg on a fixed interval, not every call.
void updateOdometry() {
  long lt, rt;
  noInterrupts();
  lt = leftTicks;
  rt = rightTicks;
  interrupts();

  long dLeftTicks  = lt - lastLeftTicks;
  long dRightTicks = rt - lastRightTicks;
  lastLeftTicks  = lt;
  lastRightTicks = rt;

  double mmPerTick = (PI * WHEEL_DIAMETER_MM) / TICKS_PER_REV;
  double dLeft_mm  = dLeftTicks  * mmPerTick;
  double dRight_mm = dRightTicks * mmPerTick;

  double dCenter_mm = (dLeft_mm + dRight_mm) / 2.0;
  double dTheta      = (dRight_mm - dLeft_mm) / WHEEL_TRACK_MM;

  // Integrate using the heading at the midpoint of this step for better
  // accuracy than a naive start-of-step heading.
  double midHeading = robotHeading + dTheta / 2.0;
  robotX_mm    += dCenter_mm * cos(midHeading);
  robotY_mm    += dCenter_mm * sin(midHeading);
  robotHeading += dTheta;

  unsigned long now = millis();
  if (now - lastOdomPrint >= ODOM_INTERVAL_MS) {
    lastOdomPrint = now;
    Serial.print("ODOM:");
    Serial.print(robotX_mm, 1);
    Serial.print(",");
    Serial.print(robotY_mm, 1);
    Serial.print(",");
    Serial.println(robotHeading * 180.0 / PI, 1);
  }
}

// ================= DASHBOARD-CONTROLLED STATE =================
// running       -> becomes true only after a valid START command
// selectedTable -> 0 = none, 1 = Table 1 (turn LEFT at intersection),
//                  2 = Table 2 (turn RIGHT at intersection)
bool running = false;
int  selectedTable = 0;

// ================= MANUAL DRIVE (phone/dashboard takeover) =================
// manualMode -> true after a "MANUAL" command; the auto line-following loop
//   is skipped entirely while this is set, and MFWD/MBACK/MLEFT/MRIGHT drive
//   the motors directly. A "START" or "STOP" command always clears it.
// lastManualCmdTime + MANUAL_TIMEOUT_MS is a dead-man's switch: if the phone
//   loses the connection mid-drive, the motors auto-stop shortly after.
bool manualMode = false;
unsigned long lastManualCmdTime = 0;
const unsigned long MANUAL_TIMEOUT_MS = 400;

String inputBuffer = "";

// ================= MOTOR FUNCTIONS (unchanged from original) =================
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
void rightslow() {
  digitalWrite(DIR_LEFT, LOW);
  digitalWrite(DIR_RIGHT, HIGH);
  analogWrite(PWM_LEFT, 30);
  analogWrite(PWM_RIGHT, speed);
}


void left() {
  digitalWrite(DIR_LEFT, HIGH);
  digitalWrite(DIR_RIGHT, LOW);
  analogWrite(PWM_LEFT, speed);
  analogWrite(PWM_RIGHT, nudgeSpeed);
}


void leftslow() {
  digitalWrite(DIR_LEFT, HIGH);
  digitalWrite(DIR_RIGHT, LOW);
  analogWrite(PWM_LEFT, speed);
  analogWrite(PWM_RIGHT, 30);
}


void stopBot() {
  analogWrite(PWM_LEFT, 0);
  analogWrite(PWM_RIGHT, 0);
}

// Mirrors forward()'s HIGH/HIGH = forward convention: LOW/LOW = reverse for
// this driver. Untested on the real chassis yet - TUNE/verify direction.
void reverseBot() {
  digitalWrite(DIR_LEFT, LOW);
  digitalWrite(DIR_RIGHT, LOW);
  analogWrite(PWM_LEFT, speed);
  analogWrite(PWM_RIGHT, speed);
}

// ================= FAKE-AUTONOMOUS PATH PLAYBACK =================
// A user-authored sequence of timed motion steps sent from the dashboard,
// e.g. "forward 3s, then left 2s, then right 2s, then stop 5s, then back
// onto the original heading" - open-loop/time-based (not encoder
// closed-loop), for scripting a fixed route without full autonomous nav.
// Runs non-blocking out of loop() so handleSerial() keeps being polled and
// STOP/PATH_STOP take effect immediately instead of only after the whole
// path finishes.
#define PATH_MAX_STEPS 30
enum PathAction { PATH_FWD, PATH_BACK, PATH_LEFT, PATH_RIGHT, PATH_HOLD };

PathAction    pathActions[PATH_MAX_STEPS];
unsigned long pathDurations[PATH_MAX_STEPS];  // ms
int           pathStepCount = 0;
int           pathStepIndex = -1;
unsigned long pathStepStartTime = 0;
bool          pathRunning = false;

void applyPathAction(PathAction action) {
  switch (action) {
    case PATH_FWD:   forward();    break;
    case PATH_BACK:  reverseBot(); break;
    case PATH_LEFT:  left();       break;
    case PATH_RIGHT: right();      break;
    default:         stopBot();    break;  // PATH_HOLD - pause in place
  }
}

void stopPath() {
  pathRunning = false;
  pathStepIndex = -1;
  stopBot();
}

// body is everything after "PATH:", steps separated by ';', each step
// "ACTION,DURATION_MS" e.g. "FWD,3000;LEFT,2000;RIGHT,2000;HOLD,5000".
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
      long durationMs = token.substring(comma + 1).toInt();
      PathAction action;
      if (actionStr == "FWD") action = PATH_FWD;
      else if (actionStr == "BACK") action = PATH_BACK;
      else if (actionStr == "LEFT") action = PATH_LEFT;
      else if (actionStr == "RIGHT") action = PATH_RIGHT;
      else action = PATH_HOLD;  // "HOLD" or anything unrecognized
      pathActions[pathStepCount] = action;
      pathDurations[pathStepCount] = (unsigned long)max(0L, durationMs);
      pathStepCount++;
    }
    if (sep == -1) break;
    start = sep + 1;
  }

  if (pathStepCount == 0) {
    Serial.println("ERR:PATH_EMPTY");
    return;
  }

  // Path mode is mutually exclusive with line-following and manual takeover.
  running = false;
  manualMode = false;
  pathRunning = true;
  pathStepIndex = 0;
  pathStepStartTime = millis();
  applyPathAction(pathActions[0]);
  Serial.print("OK:PATH_STARTED:");
  Serial.println(pathStepCount);
}

// Called every loop() iteration while pathRunning - advances to the next
// step once the current one's duration has elapsed.
void updatePath() {
  if (millis() - pathStepStartTime < pathDurations[pathStepIndex]) return;

  pathStepIndex++;
  if (pathStepIndex >= pathStepCount) {
    stopPath();
    Serial.println("PATH:DONE");
    return;
  }
  pathStepStartTime = millis();
  applyPathAction(pathActions[pathStepIndex]);
  Serial.print("PATH_STEP:");
  Serial.print(pathStepIndex + 1);
  Serial.print("/");
  Serial.println(pathStepCount);
}

// ================= INTERSECTION TURN =================
// Reuses the same tested left()/right() motor patterns above, just held for
// longer so the robot actually rotates onto the new corridor instead of only
// nudging back onto the line. TUNE the three delay/timeout values below on
// your real robot/track.
void doIntersectionTurn(int direction) {
  // direction: 1 = turn LEFT (Table 1), 2 = turn RIGHT (Table 2)

  // 1) Roll forward briefly so the sensor array/wheel axle is centered over
  //    the intersection before we start turning. TUNE this.
  forward();
  delay(150);

  // 2) Commit to the turn for a fixed time. TUNE this so it rotates roughly
  //    90 degrees onto the branch line for your chassis/speed.
  unsigned long turnStart = millis();
  while (millis() - turnStart < 1500) {
    if (direction == 1) left();
    else right();
  }

  // 3) Keep turning (slowly, same functions) until the center sensors find
  //    the new line again, with a timeout so we never spin forever. TUNE.
  unsigned long searchStart = millis();
  while (millis() - searchStart < 6000) {
    qtr.read(sensorValues);
    bool centerOnLine = (sensorValues[3] > threshold) || (sensorValues[4] > threshold);
    if (centerOnLine) break;
    if (direction == 1) left();
    else right();
  }
}

// ================= SERIAL COMMAND HANDLING =================
// Dashboard sends plain newline-terminated text commands:
//   START, STOP, TABLE1, TABLE2
//   MANUAL, MFWD, MBACK, MLEFT, MRIGHT, MSTOP  (phone/manual takeover)
//   PATH:<steps>, PATH_STOP  (fake-autonomous scripted path playback)
//   ODOM_RESET, PINS, TICKS, SENSORS  (encoder/sensor diagnostics)
void processCommand(String cmd) {
  cmd.trim();

  if (cmd == "START") {
    if (selectedTable == 0) {
      Serial.println("ERR:NO_TABLE_SELECTED");
    } else {
      manualMode = false;
      stopPath();
      running = true;
      Serial.println("OK:RUNNING");
    }
  } else if (cmd == "STOP") {
    running = false;
    manualMode = false;
    stopPath();  // also stops the motors
    Serial.println("OK:STOPPED");
  } else if (cmd == "TABLE1") {
    selectedTable = 1;
    Serial.println("OK:TABLE1_SELECTED");
  } else if (cmd == "TABLE2") {
    selectedTable = 2;
    Serial.println("OK:TABLE2_SELECTED");
  } else if (cmd == "MANUAL") {
    manualMode = true;
    running = false;
    stopPath();  // also stops the motors
    lastManualCmdTime = millis();
    Serial.println("OK:MANUAL_MODE");
  } else if (cmd.startsWith("PATH:")) {
    startPath(cmd.substring(5));
  } else if (cmd == "PATH_STOP") {
    stopPath();
    Serial.println("OK:PATH_STOPPED");
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
  } else if (cmd == "ODOM_RESET") {
    resetOdometry();
    Serial.println("OK:ODOM_RESET");
  } else if (cmd == "PINS") {
    // Raw instantaneous digitalRead of all 4 encoder pins - bypasses the
    // interrupt/tick logic entirely. Use this to check the wiring itself:
    // send it repeatedly while slowly turning a wheel by hand and watch
    // whether the values actually toggle between 0 and 1.
    Serial.print("PINS:");
    Serial.print(digitalRead(ENC_LEFT_A));
    Serial.print(",");
    Serial.print(digitalRead(ENC_LEFT_B));
    Serial.print(",");
    Serial.print(digitalRead(ENC_RIGHT_A));
    Serial.print(",");
    Serial.println(digitalRead(ENC_RIGHT_B));
  } else if (cmd == "SENSORS") {
    // Raw QTR readings, independent of running state - use this to check the
    // `threshold` constant against your actual track's lighting/surface:
    // hold the robot over plain track and over the black line and compare
    // the printed values against `threshold` (2000). If white-background
    // readings sit above threshold, or black-line readings sit below it,
    // adjust `threshold` (or re-check sensor height/angle) before trusting
    // line-following behavior.
    qtr.read(sensorValues);
    Serial.print("SENSORS:");
    for (int i = 0; i < 8; i++) {
      Serial.print(sensorValues[i]);
      if (i < 7) Serial.print(",");
    }
    Serial.println();
  } else if (cmd == "TICKS") {
    // Raw tick counts, independent of TICKS_PER_REV/WHEEL_DIAMETER_MM - use
    // this to CALIBRATE those constants: reset, rotate a wheel N full turns
    // by hand, read this, divide by N to get that side's TICKS_PER_REV.
    long lt, rt;
    noInterrupts();
    lt = leftTicks;
    rt = rightTicks;
    interrupts();
    Serial.print("TICKS:");
    Serial.print(lt);
    Serial.print(",");
    Serial.println(rt);
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

// ================= SETUP =================
void setup() {
  Serial.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, 8);

  pinMode(PWM_LEFT, OUTPUT);
  pinMode(DIR_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);

  // INPUT_PULLUP is the safe default for open-collector encoder outputs;
  // switch to plain INPUT if your encoder modules already drive push-pull
  // (most Hall-effect quadrature boards with onboard pull-ups do).
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, RISING);

  delay(1000);
  Serial.println("READY");
}

// ================= LOOP =================
void loop() {
  // Always listen for dashboard commands, even while stopped.
  handleSerial();

  // Odometry runs unconditionally so pose tracking never misses motion,
  // whether it's autonomous line-following, manual takeover, or an
  // eventual autonomous-nav drive command.
  updateOdometry();

  if (manualMode) {
    // Motors are driven directly by MFWD/MBACK/MLEFT/MRIGHT above; this is
    // just the dead-man's switch in case the phone/dashboard goes quiet.
    if (millis() - lastManualCmdTime > MANUAL_TIMEOUT_MS) {
      stopBot();
    }
    return;
  }

  if (pathRunning) {
    updatePath();
    return;
  }

  if (!running) {
    stopBot();
    return;
  }

  qtr.read(sensorValues);

  int s[8];
  for (int i = 0; i < 8; i++) {
    s[i] = (sensorValues[i] > threshold) ? 1 : 0;
  }

  // -------- INTERSECTION: all 8 sensors black --------
  if (s[0] && s[1] && s[2] && s[3] && s[4] && s[5] && s[6] && s[7]) {
    if (selectedTable == 1) {
      Serial.println("INTERSECTION:TURN_LEFT(Table1)");
      doIntersectionTurn(1);
    } else if (selectedTable == 2) {
      Serial.println("INTERSECTION:TURN_RIGHT(Table2)");
      doIntersectionTurn(2);
    } else {
      // Safety fallback: shouldn't happen since START requires a table.
      stopBot();
      running = false;
      Serial.println("INTERSECTION:NO_TABLE-STOPPED");
    }
  }

  // -------- Straight line --------
  else if (s[2] && s[3] && s[4] && s[5]) {
    forward();
    Serial.println("FORWARD");
  }

  // -------- Line drifting left --------
  else if ((s[0] && s[1]) || (s[1] && s[2])) {
    leftslow();
    Serial.println("LEFT");
    delay(20);
  }

  // -------- Line drifting right --------
  else if ((s[7] && s[6]) || (s[5] && s[6])) {
    rightslow();
    delay(20);
    Serial.println("RIGHT");
  }
  else if(s[7]){
    right();
    delay(5);
    Serial.println("RIGHT");
  }
  else if(s[0]){
    left();
    delay(5);
    Serial.println("LEFT");
  }
  // -------- Line lost --------
  else {
    stopBot();
    Serial.println("LINE_LOST");
    delay(50);  // avoid flooding serial while sitting lost
  }
}      
