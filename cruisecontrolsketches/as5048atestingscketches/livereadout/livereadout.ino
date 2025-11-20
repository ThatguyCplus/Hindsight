// IBT-2 / BTS7960 limit-switch sanity test (Arduino Mega) + AS5048A logging
#define RELAY_PIN 22
#define REN_PIN   30
#define LEN_PIN   31
#define RPWM_PIN  44
#define LPWM_PIN  45

#define AS5048_PWM_PIN 2

const uint8_t DUTY_FWD   = 150;
const uint8_t DUTY_REV   = 180;
const uint16_t FWD_MS    = 6200;
const uint16_t REV_MS    = 8000;
const uint16_t COAST_MS  = 700;

// logging rate (ms between printed samples)
const uint16_t LOG_DT_MS = 10;   // 10 ms → 100 Hz

unsigned long t0 = 0;            // reference time

float readAngleDegFast() {
  // blocking but fine for this
  unsigned long highTime = pulseIn(AS5048_PWM_PIN, HIGH, 25000);
  unsigned long lowTime  = pulseIn(AS5048_PWM_PIN, LOW,  25000);

  if (highTime == 0 || lowTime == 0) {
    return -1.0;
  }

  float period = (float)(highTime + lowTime);
  float duty   = (float)highTime / period;
  float angle  = duty * 360.0f;
  return angle;
}

// dir = +1 FWD, -1 REV, 0 COAST
void moveWithTelemetry(int dir, uint16_t duration_ms) {
  unsigned long start = millis();
  static unsigned long lastLog = 0;

  while (millis() - start < duration_ms) {
    // motor command
    if (dir > 0) {
      analogWrite(RPWM_PIN, DUTY_FWD);
      analogWrite(LPWM_PIN, 0);
    } else if (dir < 0) {
      analogWrite(RPWM_PIN, 0);
      analogWrite(LPWM_PIN, DUTY_REV);
    } else {
      analogWrite(RPWM_PIN, 0);
      analogWrite(LPWM_PIN, 0);
    }

    float angle = readAngleDegFast();

    unsigned long now = millis();
    if (angle >= 0.0 && (now - lastLog) >= LOG_DT_MS) {
      unsigned long t = now - t0;  // time since start of experiment (ms)

      // CSV-style: time_ms,angle_deg
      Serial.print(t);
      Serial.print(',');
      Serial.println(angle);

      lastLog = now;
    }

    // tiny delay so we don't hammer everything
    delay(1);
  }
}

void setup() {
  Serial.begin(115200);  // faster for logging

  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, HIGH);
  pinMode(REN_PIN,   OUTPUT); digitalWrite(REN_PIN, HIGH);
  pinMode(LEN_PIN,   OUTPUT); digitalWrite(LEN_PIN, HIGH);
  pinMode(RPWM_PIN,  OUTPUT);
  pinMode(LPWM_PIN,  OUTPUT);

  pinMode(AS5048_PWM_PIN, INPUT);

  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  delay(200);

  t0 = millis();

  // header row for CSV
  Serial.println("time_ms,angle_deg");
}

void loop() {
  // one full cycle, logged
  moveWithTelemetry(+1, FWD_MS);   // FWD
  moveWithTelemetry(0,  COAST_MS); // COAST
  moveWithTelemetry(-1, REV_MS);   // REV
  moveWithTelemetry(0,  COAST_MS); // COAST

  // OPTIONAL: stop after one cycle:
  // while (1); // uncomment if you don't want continuous repeats
}
