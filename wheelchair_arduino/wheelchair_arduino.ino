// Muse 2 Wheelchair - Arduino Motor Controller  v2
// Motor driver : L298N
// Ultrasonic   : HC-SR04
//
// Wiring (L298N):
//   ENA -> pin 6   Left  speed PWM
//   IN1 -> pin 7   Left  direction A
//   IN2 -> pin 8   Left  direction B
//   IN3 -> pin 11  Right direction A
//   IN4 -> pin 9   Right direction B
//   ENB -> pin 5   Right speed PWM
//
// Wiring (HC-SR04):
//   TRIG -> A5   ECHO -> A4   VCC -> 5V   GND -> GND
//
// Motion commands (9600 baud, single ASCII byte):
//   F = Forward      B = Backward     S = Stop
//   L = Spin Left    R = Spin Right
//   Q = Fwd+Left     E = Fwd+Right
//   G = Bck+Left     H = Bck+Right
//
// Speed commands:
//   1 = SLOW (130 PWM)    2 = MED (179 PWM)    3 = FAST (255 PWM)
//   Arduino flashes LED N times to confirm (1/2/3 flashes).
//   Speed change takes effect on the next motion command AND
//   immediately re-applies to any currently running motion.
//
// Safety:
//   Forward motion blocked when obstacle < STOP_DIST_CM (debounced x3).
//   Automatically unblocks when path is clear.

#define ENA  6
#define IN1  7
#define IN2  8
#define IN3  11
#define IN4  9
#define ENB  5

#define TRIG_PIN A5
#define ECHO_PIN A4

// ── Speed table ───────────────────────────────────────────────────────────────
//                         SLOW   MED   FAST
const int SPD_MAIN[]  = { 130,   179,   255 };  // straight drive
const int SPD_INNER[] = {  90,   150,   220 };  // inner wheel on pivot turn (reversed)
const int SPD_TURN[]  = { 220,   255,   255 };  // spin-in-place — fast, tight 180s

const int STOP_DIST_CM = 10;
const int BLOCK_HITS   = 3;

int  speedIdx   = 1;   // default: MED
char lastCmd    = '\0';
bool fwdBlocked = false;
int  blockCount = 0;
int  clearCount = 0;
const int CLEAR_HITS = 3;  // consecutive clear readings needed to unblock

// ── Motor helpers ─────────────────────────────────────────────────────────────
void leftFwd(int s)  { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  analogWrite(ENA,s); }
void leftBwd(int s)  { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); analogWrite(ENA,s); }
void leftStop()      { digitalWrite(IN1,LOW);  digitalWrite(IN2,LOW);  analogWrite(ENA,0); }
void rightFwd(int s) { digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  analogWrite(ENB,s); }
void rightBwd(int s) { digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); analogWrite(ENB,s); }
void rightStop()     { digitalWrite(IN3,LOW);  digitalWrite(IN4,LOW);  analogWrite(ENB,0); }

void cmdForward()    { leftFwd(SPD_MAIN[speedIdx]);  rightFwd(SPD_MAIN[speedIdx]);  }
void cmdBackward()   { leftBwd(SPD_MAIN[speedIdx]);  rightBwd(SPD_MAIN[speedIdx]);  }
void cmdLeft()       { leftFwd(SPD_TURN[speedIdx]);  rightBwd(SPD_TURN[speedIdx]);  }
void cmdRight()      { leftBwd(SPD_TURN[speedIdx]);  rightFwd(SPD_TURN[speedIdx]);  }
// Pivot turns: outer wheel drives, inner wheel REVERSES → much tighter than a
// curve (rotates about a point between the wheels, needs little space).
void cmdCurveLeft()  { leftBwd(SPD_INNER[speedIdx]); rightFwd(SPD_MAIN[speedIdx]);  }
void cmdCurveRight() { leftFwd(SPD_MAIN[speedIdx]);  rightBwd(SPD_INNER[speedIdx]); }
void cmdBackLeft()   { leftFwd(SPD_INNER[speedIdx]); rightBwd(SPD_MAIN[speedIdx]);  }
void cmdBackRight()  { leftBwd(SPD_MAIN[speedIdx]);  rightFwd(SPD_INNER[speedIdx]); }
void cmdStop()       { leftStop(); rightStop(); }

// ── Helpers ───────────────────────────────────────────────────────────────────
void flashLED(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(80);
    digitalWrite(LED_BUILTIN, LOW);  delay(120);
  }
}

void dispatchCmd(char cmd) {
  switch (cmd) {
    case 'F': cmdForward();    break;
    case 'B': cmdBackward();   break;
    case 'L': cmdLeft();       break;
    case 'R': cmdRight();      break;
    case 'Q': cmdCurveLeft();  break;
    case 'E': cmdCurveRight(); break;
    case 'G': cmdBackLeft();   break;
    case 'H': cmdBackRight();  break;
    case 'S': cmdStop();       break;
  }
}

long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  return (dur == 0) ? 999 : dur / 58L;
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  cmdStop();
  delay(300);

  // Boot motor test: forward 0.4 s then stop
  cmdForward(); delay(400); cmdStop();

  // 2 LED flashes = booted at MED speed
  flashLED(2);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {

  // ── 1. Obstacle check (debounced both ways) ───────────────────────────────
  long dist = readDistanceCM();

  if (dist == 999) {
    // Sensor timeout — treat as ambiguous: preserve current blocked state.
    // A timeout is common when a target is too close or absorbs the echo.
    // Do NOT reset clearCount or blockCount.
  } else if (dist <= STOP_DIST_CM) {
    blockCount++;
    clearCount = 0;
    if (blockCount >= BLOCK_HITS) {
      fwdBlocked = true;
      if (lastCmd == 'F' || lastCmd == 'Q' || lastCmd == 'E') {
        cmdStop();
        lastCmd = 'S';
      }
    }
  } else {
    clearCount++;
    blockCount = 0;
    if (clearCount >= CLEAR_HITS) fwdBlocked = false;
  }

  // ── 2. Incoming serial byte ────────────────────────────────────────────────
  if (!Serial.available()) return;
  char cmd = (char)Serial.read();

  // ── Speed command: '1' / '2' / '3' ───────────────────────────────────────
  if (cmd >= '1' && cmd <= '3') {
    speedIdx = cmd - '1';
    flashLED(speedIdx + 1);   // 1 flash=slow, 2=med, 3=fast

    // Immediately re-apply new speed to currently running motion
    if (lastCmd != 'S' && lastCmd != '\0') {
      bool blocked = (lastCmd=='F'||lastCmd=='Q'||lastCmd=='E') && fwdBlocked;
      if (!blocked) dispatchCmd(lastCmd);
    }
    return;
  }

  // ── Motion command ────────────────────────────────────────────────────────
  if (cmd=='F'||cmd=='B'||cmd=='L'||cmd=='R'||cmd=='S'
      ||cmd=='Q'||cmd=='E'||cmd=='G'||cmd=='H') {

    digitalWrite(LED_BUILTIN, HIGH); delay(25); digitalWrite(LED_BUILTIN, LOW);

    // Block forward-direction commands when obstacle is detected
    if ((cmd=='F'||cmd=='Q'||cmd=='E') && fwdBlocked) return;

    if (cmd != lastCmd) {
      lastCmd = cmd;
      dispatchCmd(cmd);
    }
  }
  // All other bytes (e.g. HC-08 "OK+CONN" noise) are silently ignored
}
