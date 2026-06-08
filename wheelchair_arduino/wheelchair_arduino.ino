// Muse 2 Wheelchair - Arduino Motor Controller  v2
// Motor driver : L298N
//
// Wiring (L298N):
//   ENA -> pin 6   Left  speed PWM
//   IN1 -> pin 7   Left  direction A
//   IN2 -> pin 8   Left  direction B
//   IN3 -> pin 11  Right direction A
//   IN4 -> pin 9   Right direction B
//   ENB -> pin 5   Right speed PWM
//
// Motion commands (9600 baud, single ASCII byte):
//   F = Forward      B = Backward     S = Stop
//   L = Turn Left    R = Turn Right   (curved, no spin-in-place)
//   Q = Fwd+Left     E = Fwd+Right
//   G = Bck+Left     H = Bck+Right
//
// Speed commands:
//   1 = SLOW (130 PWM)    2 = MED (179 PWM)    3 = FAST (255 PWM)
//   Arduino flashes LED N times to confirm (1/2/3 flashes).
//   Speed change takes effect on the next motion command AND
//   immediately re-applies to any currently running motion.

#define ENA  6
#define IN1  7
#define IN2  8
#define IN3  11
#define IN4  9
#define ENB  5

// ── Speed table ───────────────────────────────────────────────────────────────
//                         SLOW   MED   FAST   (all 20% slower than original)
const int SPD_MAIN[]  = { 104,   143,   204 };  // straight drive / outer wheel on turn
const int SPD_INNER[] = {  13,    20,    32 };  // inner wheel on curve — low = tight, fast turn (~30% tighter than before = sharper angle)

int  speedIdx   = 1;   // default: MED
char lastCmd    = '\0';

// ── Motor helpers ─────────────────────────────────────────────────────────────
void leftFwd(int s)  { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  analogWrite(ENA,s); }
void leftBwd(int s)  { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); analogWrite(ENA,s); }
void leftStop()      { digitalWrite(IN1,LOW);  digitalWrite(IN2,LOW);  analogWrite(ENA,0); }
void rightFwd(int s) { digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  analogWrite(ENB,s); }
void rightBwd(int s) { digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); analogWrite(ENB,s); }
void rightStop()     { digitalWrite(IN3,LOW);  digitalWrite(IN4,LOW);  analogWrite(ENB,0); }

void cmdForward()    { leftFwd(SPD_MAIN[speedIdx]);  rightFwd(SPD_MAIN[speedIdx]);  }
void cmdBackward()   { leftBwd(SPD_MAIN[speedIdx]);  rightBwd(SPD_MAIN[speedIdx]);  }
// L / R are curved turns (no spin-in-place): both wheels forward, inner slower.
void cmdLeft()       { leftFwd(SPD_MAIN[speedIdx]);  rightFwd(SPD_INNER[speedIdx]); }
void cmdRight()      { leftFwd(SPD_INNER[speedIdx]); rightFwd(SPD_MAIN[speedIdx]);  }
// Curve turns: both wheels drive the SAME direction, inner wheel much slower.
// Low SPD_INNER → tight arc that still moves forward/back (no spin-in-place).
// Wheel assignment kept identical to the original so L/R direction is unchanged.
void cmdCurveLeft()  { leftFwd(SPD_MAIN[speedIdx]);  rightFwd(SPD_INNER[speedIdx]); }
void cmdCurveRight() { leftFwd(SPD_INNER[speedIdx]); rightFwd(SPD_MAIN[speedIdx]);  }
void cmdBackLeft()   { leftBwd(SPD_MAIN[speedIdx]);  rightBwd(SPD_INNER[speedIdx]); }
void cmdBackRight()  { leftBwd(SPD_INNER[speedIdx]); rightBwd(SPD_MAIN[speedIdx]);  }
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

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  cmdStop();
  delay(300);

  // Boot motor test: forward 0.4 s then stop
  cmdForward(); delay(400); cmdStop();

  // 2 LED flashes = booted at MED speed
  flashLED(2);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {

  // ── Incoming serial byte ───────────────────────────────────────────────────
  if (!Serial.available()) return;
  char cmd = (char)Serial.read();

  // ── Speed command: '1' / '2' / '3' ───────────────────────────────────────
  if (cmd >= '1' && cmd <= '3') {
    speedIdx = cmd - '1';
    flashLED(speedIdx + 1);   // 1 flash=slow, 2=med, 3=fast

    // Immediately re-apply new speed to currently running motion
    if (lastCmd != 'S' && lastCmd != '\0') dispatchCmd(lastCmd);
    return;
  }

  // ── Motion command ────────────────────────────────────────────────────────
  if (cmd=='F'||cmd=='B'||cmd=='L'||cmd=='R'||cmd=='S'
      ||cmd=='Q'||cmd=='E'||cmd=='G'||cmd=='H') {

    digitalWrite(LED_BUILTIN, HIGH); delay(25); digitalWrite(LED_BUILTIN, LOW);

    if (cmd != lastCmd) {
      lastCmd = cmd;
      dispatchCmd(cmd);
    }
  }
  // All other bytes (e.g. HC-08 "OK+CONN" noise) are silently ignored
}
