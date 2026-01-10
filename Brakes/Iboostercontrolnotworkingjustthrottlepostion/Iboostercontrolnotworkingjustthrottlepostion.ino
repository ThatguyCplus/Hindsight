/*
 * Tesla Gen2 iBooster Controller (Yaw CAN only)
 * Hardware:
 *   - Arduino Mega
 *   - MCP2515 + transceiver on Yaw CAN (500 kbps)
 *   - MCP2515 crystal: set MCP_CLOCK below (MCP_8MHZ or MCP_16MHZ)
 *
 * Wiring (Mega):
 *   SCK  -> 52
 *   MOSI -> 51
 *   MISO -> 50
 *   CS   -> 4   (change CS_PIN below if needed)
 *   INT  -> not used (polling)
 *   VCC  -> 5V, GND -> GND
 *
 * Serial commands (115200 baud):
 *   r : toggle raw CAN log
 *   e : enable external request (take control)
 *   d : disable external request (driver only, flow to zero-point)
 *   b : apply brakes   (flow_rate > zero point)
 *   s : release brakes (flow_rate < zero point)
 *   z : hold pressure  (flow_rate = zero point)
 */

#include <SPI.h>
#include <mcp2515.h>

// ---------- Config ----------
#define CS_PIN        4
#define CAN_SPEED     CAN_500KBPS
#define MCP_CLOCK     MCP_8MHZ    // change to MCP_16MHZ if your MCP2515 is 16 MHz

// IDs (Yaw CAN)
#define ID_VEHICLE_STATUS  0x38B  // BO_ 907
#define ID_BRAKE_REQUEST   0x38C  // BO_ 908
#define ID_VEHICLE_ALIVE   0x38D  // BO_ 909
#define ID_STATUS          0x39D  // BO_ 925

// Timing
#define TX_PERIOD_MS       10
#define DASH_PERIOD_MS     500

// Flow rate
#define FLOW_ZERO          0x7E00  // 32256
#define FLOW_RANGE         5120    // +/- span for quick apply/release

// CRC8 SAE J1850
#define CRC_POLY   0x1D
#define CRC_INIT   0xFF
#define CRC_XOROUT 0xFF

MCP2515 yawCAN(CS_PIN);
uint8_t crc8_lut[256];

// Counters
uint8_t vs_ctr = 0, br_ctr = 0, va_ctr = 0;

// Control state
bool external_request = false;
uint16_t flow_rate = FLOW_ZERO;

// Status (0x39D)
float rod_pos_mm = 0.0f;
uint8_t stat_status = 0;
uint8_t stat_internal = 0;
uint8_t stat_driver = 0;
uint8_t stat_counter = 0;
bool stat_crc_ok = false;
unsigned long stat_age_ms = 0;

// Logging
bool raw_log = false;

// Timing
unsigned long last_tx = 0;
unsigned long last_dash = 0;
unsigned long t0 = 0;

// ---------- CRC ----------
void gen_crc8() {
  for (uint16_t i = 0; i < 256; i++) {
    uint8_t c = i;
    for (uint8_t b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ CRC_POLY) : (uint8_t)(c << 1);
    crc8_lut[i] = c;
  }
}
uint8_t crc8(const uint8_t *d, int len) {
  uint8_t c = CRC_INIT;
  for (int i = 1; i < len; i++) {
    c ^= d[i];
    c = crc8_lut[c];
  }
  return c ^ CRC_XOROUT;
}

// ---------- TX frames ----------
void send_38B() {
  struct can_frame m;
  m.can_id = ID_VEHICLE_STATUS;
  m.can_dlc = 4;
  uint8_t *d = m.data;
  d[0] = 0;
  d[1] = (vs_ctr & 0x0F);
  if (external_request) d[1] |= 0x40;  // external_brake_request bit
  d[2] = 0x05; // unknown3 low
  d[3] = 0x00; // unknown3 high
  d[0] = crc8(d, 4);
  yawCAN.sendMessage(&m);
  vs_ctr = (vs_ctr + 1) & 0x0F;
}

void send_38C() {
  struct can_frame m;
  m.can_id = ID_BRAKE_REQUEST;
  m.can_dlc = 4;
  uint8_t *d = m.data;
  d[0] = 0;
  d[1] = (br_ctr & 0x0F);
  if (external_request) d[1] |= 0x40;  // external_request bit
  d[2] = (uint8_t)(flow_rate & 0xFF);
  d[3] = (uint8_t)((flow_rate >> 8) & 0xFF);
  d[0] = crc8(d, 4);
  yawCAN.sendMessage(&m);
  br_ctr = (br_ctr + 1) & 0x0F;
}

void send_38D() {
  struct can_frame m;
  m.can_id = ID_VEHICLE_ALIVE;
  m.can_dlc = 4;
  uint8_t *d = m.data;
  d[0] = 0;
  d[1] = (va_ctr & 0x0F); // counter
  d[2] = 0x47;            // known working value
  d[3] = 0x00;
  d[0] = crc8(d, 4);
  yawCAN.sendMessage(&m);
  va_ctr = (va_ctr + 1) & 0x0F;
}

// ---------- RX parsing 0x39D ----------
void parse_39D(const struct can_frame &m) {
  const uint8_t *d = m.data;
  uint8_t c = crc8(d, m.can_dlc);
  stat_crc_ok = (c == d[0]);
  stat_counter = d[1] & 0x0F;

  // Approx decode per DBC (21|12, scale 0.015625, offset -5)
  uint16_t raw12 = (uint16_t)d[2] | ((uint16_t)d[3] << 8);
  rod_pos_mm = raw12 * 0.015625f - 5.0f;

  stat_status       = (d[1] >> 4) & 0x07;     // (12|3) approx
  stat_driver       = d[2] & 0x03;            // (16|2)
  stat_internal     = (d[2] >> 2) & 0x07;     // (18|3)
  stat_age_ms = millis();
}

// ---------- Process RX ----------
void poll_rx() {
  struct can_frame m;
  if (yawCAN.readMessage(&m) == MCP2515::ERROR_OK) {
    uint16_t id = m.can_id & 0x7FF;
    if (raw_log) {
      Serial.print("YCAN "); Serial.print(id, HEX);
      Serial.print(" ["); Serial.print(m.can_dlc); Serial.print("] ");
      for (uint8_t i = 0; i < m.can_dlc; i++) {
        if (m.data[i] < 0x10) Serial.print('0');
        Serial.print(m.data[i], HEX); Serial.print(' ');
      }
      Serial.println();
    }
    if (id == ID_STATUS && m.can_dlc >= 4) parse_39D(m);
  }
}

// ---------- Serial commands ----------
void handle_cmd() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'r': case 'R':
      raw_log = !raw_log;
      Serial.print("Raw log: "); Serial.println(raw_log ? "ON" : "OFF");
      break;
    case 'e': case 'E':
      external_request = true;
      Serial.println("External request ENABLED");
      break;
    case 'd': case 'D':
      external_request = false;
      flow_rate = FLOW_ZERO;
      Serial.println("External request DISABLED, flow=zero");
      break;
    case 'b': case 'B':
      external_request = true;
      flow_rate = FLOW_ZERO + (FLOW_RANGE / 2);
      Serial.print("Brake APPLY, flow="); Serial.println(flow_rate);
      break;
    case 's': case 'S':
      external_request = true;
      flow_rate = FLOW_ZERO - (FLOW_RANGE / 2);
      Serial.print("Brake RELEASE, flow="); Serial.println(flow_rate);
      break;
    case 'z': case 'Z':
      external_request = true;
      flow_rate = FLOW_ZERO;
      Serial.println("Brake HOLD (zero)");
      break;
    default: break;
  }
  while (Serial.available()) Serial.read(); // flush
}

// ---------- Dashboard ----------
void dash() {
  Serial.println("=== Gen2 iBooster Yaw Controller ===");
  Serial.print("t = "); Serial.print((millis() - t0) / 1000); Serial.println(" s");

  Serial.print("ext_request: "); Serial.println(external_request ? "ON" : "OFF");
  Serial.print("flow_rate  : "); Serial.println(flow_rate);

  Serial.println("Status 0x39D:");
  Serial.print("  rod_pos(mm): "); Serial.println(rod_pos_mm, 2);
  Serial.print("  status     : "); Serial.println(stat_status);
  Serial.print("  internal   : "); Serial.println(stat_internal);
  Serial.print("  drv_brake  : "); Serial.println(stat_driver);
  Serial.print("  counter    : "); Serial.println(stat_counter);
  Serial.print("  CRC        : "); Serial.println(stat_crc_ok ? "OK" : "FAIL/no data");
  Serial.print("  age (ms)   : ");
  if (stat_age_ms == 0) Serial.println("no frame"); else Serial.println(millis() - stat_age_ms);

  Serial.println("Commands: r raw | e enable | d disable | b apply | s release | z hold");
  Serial.println();
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println("\nInit...");
  SPI.begin();
  gen_crc8();

  if (yawCAN.reset() == MCP2515::ERROR_OK) Serial.println("MCP2515 reset OK");
  else Serial.println("MCP2515 reset FAIL");

  if (yawCAN.setBitrate(CAN_SPEED, MCP_CLOCK) == MCP2515::ERROR_OK) Serial.println("Bitrate OK");
  else Serial.println("Bitrate FAIL");

  if (yawCAN.setNormalMode() == MCP2515::ERROR_OK) Serial.println("Normal mode OK");
  else Serial.println("Normal mode FAIL");

  t0 = millis();
  last_tx = t0;
  last_dash = t0;

  Serial.println("Start sending 38B/38C/38D at 10 ms. Power iBooster AFTER this is running.");
  Serial.println("Use 'e' then 'b'/'s'/'z' to move brakes.");
  Serial.println();
}

void loop() {
  unsigned long now = millis();

  // Periodic TX
  if (now - last_tx >= TX_PERIOD_MS) {
    last_tx = now;
    send_38B();
    send_38C();
    send_38D();
  }

  poll_rx();
  handle_cmd();

  if (now - last_dash >= DASH_PERIOD_MS) {
    last_dash = now;
    dash();
  }
}