// Joystick → IBT-2 (BTS7960) with HARD STOP + clean current table
// Mega pins: REN=30, LEN=31, RPWM=44, LPWM=45, Relay=22, VRy=A2, SW=D23, R_IS=A0, L_IS=A1
#include <math.h>

#define RELAY_PIN 22
#define REN_PIN   30
#define LEN_PIN   31
#define RPWM_PIN  44
#define LPWM_PIN  45
#define JOY_Y_PIN A2
#define JOY_SW_PIN 23
#define RIS_PIN   A0
#define LIS_PIN   A1

// ----- Tuning -----
const uint8_t MAX_DUTY   = 230;   // cap speed at ends (0..255)
const uint8_t MIN_MOVE   = 70;    // min duty to overcome stiction
const int     DEAD_BAND  = 40;    // joystick dead zone around center
const float   EXPO       = 2.8;   // slow middle, fast ends (1.0 = linear)
const uint8_t SLEW_STEP  = 12;    // bigger step = snappier response
const uint16_t LOOP_MS   = 10;    // control loop rate

// Hard-stop (electronic brake) parameters
const uint16_t BRAKE_MS  = 120;   // how long to apply brake on stop
const float    BRAKE_I_MAX = 25.0; // abort brake if total current exceeds this

// Current-sense calibration (tune with clamp meter)
const float ADC_VREF   = 5.0;
const float AMPS_PER_V = 10.0;
const int   AVG_IS     = 6;
const int   AVG_JOY    = 8;

// Button behavior
const bool TOGGLE_MODE = true;    // true: press to latch relay; false: hold-to-run

// ----- Internals -----
int  joyCenter = 512;
int  cmdPWM = 0;                  // signed -255..+255
bool relayEnabled = false;
bool lastBtn = false;
unsigned long lastPrint = 0;
int  printCount = 0;

int readAvg(uint8_t pin, int n){ long acc=0; for(int i=0;i<n;i++) acc+=analogRead(pin); return acc/n; }
float readV(uint8_t pin){ long acc=0; for(int i=0;i<AVG_IS;i++) acc+=analogRead(pin); return (acc/float(AVG_IS))*(ADC_VREF/1023.0f); }

void driveMotorSigned(int val){
  val = constrain(val, -255, 255);
  if (val > 0){ analogWrite(RPWM_PIN, val); analogWrite(LPWM_PIN, 0); }
  else if (val < 0){ analogWrite(RPWM_PIN, 0); analogWrite(LPWM_PIN, -val); }
  else { analogWrite(RPWM_PIN, 0); analogWrite(LPWM_PIN, 0); }
}

// brief electronic brake, with current safety
void hardStop(){
  // apply brake (both PWMs high) but bail if current spikes too high
  unsigned long t0 = millis();
  analogWrite(RPWM_PIN, 255);
  analogWrite(LPWM_PIN, 255);
  while (millis() - t0 < BRAKE_MS){
    float iTot = (readV(RIS_PIN) + readV(LIS_PIN)) * AMPS_PER_V;
    if (iTot > BRAKE_I_MAX) break;
  }
  // release to zero
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  cmdPWM = 0;
}

int mapExpoSigned(int centered){
  if (abs(centered) < DEAD_BAND) return 0;
  float span = 512.0f - DEAD_BAND;
  float sgn  = (centered > 0) ? 1.0f : -1.0f;
  float mag  = fminf(1.0f, fabsf(centered) / span);
  float shaped = powf(mag, EXPO);
  int duty = (int)roundf(shaped * MAX_DUTY);
  if (duty > 0 && duty < MIN_MOVE) duty = MIN_MOVE;
  return (int)(sgn * duty);
}

void setup(){
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, LOW);
  pinMode(REN_PIN, OUTPUT);   digitalWrite(REN_PIN, HIGH);
  pinMode(LEN_PIN, OUTPUT);   digitalWrite(LEN_PIN, HIGH);
  pinMode(RPWM_PIN, OUTPUT);  analogWrite(RPWM_PIN, 0);
  pinMode(LPWM_PIN, OUTPUT);  analogWrite(LPWM_PIN, 0);
  pinMode(JOY_SW_PIN, INPUT_PULLUP);

  delay(100);
  joyCenter = readAvg(JOY_Y_PIN, 48);

  // print header
  Serial.println(F("rawY  cent  targ  cmd   IR[A]  IL[A]  ITot  DIR  REL"));
}

void loop(){
  // --- button → relay state ---
  bool btn = (digitalRead(JOY_SW_PIN) == LOW);
  if (TOGGLE_MODE){
    if (btn && !lastBtn) relayEnabled = !relayEnabled;
  } else {
    relayEnabled = btn;
  }
  lastBtn = btn;
  digitalWrite(RELAY_PIN, relayEnabled ? HIGH : LOW);

  // --- joystick to target ---
  int raw = readAvg(JOY_Y_PIN, AVG_JOY);
  int centered = raw - joyCenter;
  int target = relayEnabled ? mapExpoSigned(centered) : 0;

  // If we just entered "stop" state, perform hard stop
  static bool wasMoving = false;
  bool isMovingCmd = (target != 0);
  if (!isMovingCmd && wasMoving){
    hardStop();
  }
  wasMoving = isMovingCmd;

  // Slew to target (snappier thanks to larger SLEW_STEP)
  if (cmdPWM < target) cmdPWM = min(cmdPWM + SLEW_STEP, target);
  else if (cmdPWM > target) cmdPWM = max(cmdPWM - SLEW_STEP, target);

  // If relay is off, force stop
  if (!relayEnabled){
    if (cmdPWM != 0) hardStop();
    cmdPWM = 0;
  }

  // Drive motor
  driveMotorSigned(cmdPWM);

  // Telemetry (10 Hz) with header refresh every ~30 lines
  unsigned long now = millis();
  if (now - lastPrint >= 100){
    float vR = readV(RIS_PIN), vL = readV(LIS_PIN);
    float iR = vR * AMPS_PER_V, iL = vL * AMPS_PER_V;
    float iTot = iR + iL;
    const char* dir = (cmdPWM>0) ? "FWD" : (cmdPWM<0) ? "REV" : "----";
    const char* rel = relayEnabled ? "ON " : "OFF";

    // fixed-width-ish table
    char buf[120];
    snprintf(buf, sizeof(buf), "%4d %5d %5d %5d  %5.2f  %5.2f  %5.2f  %4s  %3s",
             raw, centered, target, cmdPWM, iR, iL, iTot, dir, rel);
    Serial.println(buf);

    printCount++;
    if (printCount >= 30){
      Serial.println(F("rawY  cent  targ  cmd   IR[A]  IL[A]  ITot  DIR  REL"));
      printCount = 0;
    }
    lastPrint = now;
  }

  delay(LOOP_MS);
}
