// IBT-2 / BTS7960 limit-switch sanity test (Arduino Mega)
// Wiring: REN=30, LEN=31, RPWM=44, LPWM=45, Relay=22
#define RELAY_PIN 22
#define REN_PIN   30
#define LEN_PIN   31
#define RPWM_PIN  44
#define LPWM_PIN  45

const uint8_t DUTY_FWD   = 150;   // gentle forward
const uint8_t DUTY_REV   = 180;   // slightly stronger reverse to clear limit
const uint16_t FWD_MS    = 6200;   // short forward pulse
const uint16_t REV_MS    = 8000;  // long reverse pulse to re-arm the forward limit
const uint16_t COAST_MS  = 700;

void setup() {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, HIGH);
  pinMode(REN_PIN,   OUTPUT); digitalWrite(REN_PIN, HIGH);
  pinMode(LEN_PIN,   OUTPUT); digitalWrite(LEN_PIN, HIGH);
  pinMode(RPWM_PIN,  OUTPUT);
  pinMode(LPWM_PIN,  OUTPUT);
  // coast
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  delay(200);
  Serial.println("Limit test: FWD(short) -> coast -> REV(long) -> coast (repeat)");
}

void loop() {
  // FWD (short)
  Serial.println("FWD");
  analogWrite(RPWM_PIN, DUTY_FWD);
  analogWrite(LPWM_PIN, 0);
  delay(FWD_MS);

  // Coast
  Serial.println("COAST");
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  delay(COAST_MS);

  // REV (long to reset forward limit switch)
  Serial.println("REV");
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, DUTY_REV);
  delay(REV_MS);

  // Coast
  Serial.println("COAST");
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  delay(COAST_MS);
}
