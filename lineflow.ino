// Declared up here, before anything else, because the Arduino IDE
// auto-generates function prototypes and inserts them right after the
// last #include/comment block - if this enum were declared further down
// (where it's used), those auto-generated prototypes would reference it
// before its definition and fail to compile ("'PathAction' was not
// declared in this scope"). See the PATH PLAYBACK section below for its use.
enum PathAction { PATH_FWD, PATH_BACK, PATH_LEFT, PATH_RIGHT, PATH_HOLD };

// ================= CYTRON MDD10A =================
#define PWM_LEFT  2
#define DIR_LEFT  3
#define PWM_RIGHT 4
#define DIR_RIGHT 5

int speed      = 90;    // normal drive speed
int nudgeSpeed = 85;    // slow-side speed while turning

// ================= MANUAL DRIVE (phone/dashboard takeover) =================
// manualMode -> true after a "MANUAL" command; MFWD/MBACK/MLEFT/MRIGHT drive
//   the motors directly. A "STOP" command always clears it.
// lastManualCmdTime + MANUAL_TIMEOUT_MS is a dead-man's switch: if the phone
//   loses the connection mid-drive, the motors auto-stop shortly after.
bool manualMode = false;
unsigned long lastManualCmdTime = 0;
const unsigned long MANUAL_TIMEOUT_MS = 400;

String inputBuffer = "";

// ================= MOTOR FUNCTIONS =================
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

// ================= PATH PLAYBACK (scripted timed moves) =================
// A user-authored sequence of timed motion steps sent from the dashboard,
// e.g. "forward 3s, then left 2s, then right 2s, then hold 5s..." -
// open-loop/time-based. Runs non-blocking out of loop() so handleSerial()
// keeps being polled and STOP/PATH_STOP take effect immediately instead of
// only after the whole path finishes.
#define PATH_MAX_STEPS 30
// PathAction enum itself is declared at the very top of the file - see the
// comment there for why.

PathAction    pathActions[PATH_MAX_STEPS];
unsigned long pathDurations[PATH_MAX_STEPS];  // ms
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

  // Path mode is mutually exclusive with manual takeover.
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

// ================= SERIAL COMMAND HANDLING =================
// Dashboard sends plain newline-terminated text commands:
//   STOP
//   MANUAL, MFWD, MBACK, MLEFT, MRIGHT, MSTOP  (phone/manual takeover)
//   PATH:<steps>, PATH_STOP, PATH_PAUSE, PATH_RESUME  (scripted path playback)
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

  pinMode(PWM_LEFT, OUTPUT);
  pinMode(DIR_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);

  delay(1000);
  Serial.println("READY");
}

// ================= LOOP =================
void loop() {
  // Always listen for dashboard commands, even while idle.
  handleSerial();

  if (manualMode) {
    // Motors are driven directly by MFWD/MBACK/MLEFT/MRIGHT above; this is
    // just the dead-man's switch in case the phone/dashboard goes quiet.
    if (millis() - lastManualCmdTime > MANUAL_TIMEOUT_MS) {
      stopBot();
    }
    return;
  }

  if (pathRunning) {
    if (!pathPaused) updatePath();  // paused: motors already off, hold position in the step sequence
    return;
  }

  stopBot();  // idle - not in manual mode or running a path
}
