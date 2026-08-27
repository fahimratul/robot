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

// ================= DASHBOARD-CONTROLLED STATE =================
// running       -> becomes true only after a valid START command
// selectedTable -> 0 = none, 1 = Table 1 (turn LEFT at intersection),
//                  2 = Table 2 (turn RIGHT at intersection)
bool running = false;
int  selectedTable = 0;

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
void processCommand(String cmd) {
  cmd.trim();

  if (cmd == "START") {
    if (selectedTable == 0) {
      Serial.println("ERR:NO_TABLE_SELECTED");
    } else {
      running = true;
      Serial.println("OK:RUNNING");
    }
  } else if (cmd == "STOP") {
    running = false;
    stopBot();
    Serial.println("OK:STOPPED");
  } else if (cmd == "TABLE1") {
    selectedTable = 1;
    Serial.println("OK:TABLE1_SELECTED");
  } else if (cmd == "TABLE2") {
    selectedTable = 2;
    Serial.println("OK:TABLE2_SELECTED");
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

  delay(1000);
  Serial.println("READY");
}

// ================= LOOP =================
void loop() {
  // Always listen for dashboard commands, even while stopped.
  handleSerial();

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
  }
}
