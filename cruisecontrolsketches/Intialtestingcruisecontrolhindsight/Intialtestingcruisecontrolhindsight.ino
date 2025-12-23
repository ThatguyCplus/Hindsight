/*
 * iBooster Bench Controller - "One Program That Rules It All"
 * Platform: Arduino Mega 2560 + 2x MCP2515 (8MHz)
 *
 * Goals:
 *  - Mimic wake/control traffic for Bosch Gen2 iBooster (Tesla/Honda variant)
 *  - Bench-safe: ARM gate, timeout, immediate stop
 *  - Debug-friendly: TX/RX bus select, CRC mode select + CRC auto-score from RX
 *
 * WIRING:
 *  - SPI: SCK(52), MOSI(51), MISO(50)
 *  - CAN VEH (Bus A): CS=5, INT=2
 *  - CAN YAW (Bus B): CS=4, INT=3
 */

#include <SPI.h>
#include <mcp_can.h>

// -------------------------- Pins --------------------------
#define CAN0_CS  5
#define CAN0_INT 2
#define CAN1_CS  4
#define CAN1_INT 3

// -------------------------- CAN ---------------------------
static const uint32_t CAN_BAUD = CAN_500KBPS;
static const uint8_t  CAN_CLK  = MCP_8MHZ;

// -------------------------- iBooster constants --------------------------
// NOTE: These are the values you were using; keep them unless your unit needs different ranges.
static const uint16_t Q_TARGET_NEUTRAL   = 0x7E00;
static const uint16_t Q_TARGET_MAX_BRAKE = 0x8200;   // "max safe bench" target
static const uint16_t Q_TARGET_MIN       = 0x6A00;
static const uint16_t Q_TARGET_MAX       = 0x9200;

static const uint8_t  P_LIMIT_EXTERNAL = 120; // external pressure limit (scaled below)

// From your reference assumptions
static const uint8_t  P_MC       = 0;
static const uint8_t  P_MC_QF    = 1;
static const uint8_t  P_EST_MAX  = 250;
static const uint8_t  P_EST_MAX_QF = 1;
static const uint8_t  VEHICLE_QF = 1;

// CRC8 SAE J1850 poly
static const uint8_t CRC_POLY = 0x1D;

// -------------------------- Devices --------------------------
MCP_CAN CAN_VEH(CAN0_CS);
MCP_CAN CAN_YAW(CAN1_CS);

// -------------------------- Modes --------------------------
enum ControlMode : uint8_t {
  MODE_NEUTRAL = 0,
  MODE_RAMP_BRAKE,
  MODE_RAMP_RELEASE
};

// -------------------------- CRC Modes --------------------------
struct CrcMode {
  const char* name;
  uint8_t init;
  uint8_t xorout;
};

// Common Bosch/J1850 variants you might see in the wild
static const CrcMode CRC_MODES[] = {
  { "INIT00_XOR00", 0x00, 0x00 },
  { "INIT00_XORFF", 0x00, 0xFF },
  { "INITFF_XOR00", 0xFF, 0x00 },
  { "INITFF_XORFF", 0xFF, 0xFF },
};
static const uint8_t CRC_MODE_COUNT = sizeof(CRC_MODES) / sizeof(CRC_MODES[0]);

// -------------------------- State --------------------------
static bool ignition_status = false;
static bool verbose_mode = false;

// Bench safety
static bool armed = false;
static uint32_t last_user_activity_ms = 0;
static const uint32_t IDLE_DISARM_MS = 10000; // 10s of no serial input => disarm + neutral

// Control behavior
static ControlMode mode = MODE_NEUTRAL;
static bool neutral_holds_qf = true; // if true, keep QF=1 while commanding neutral
static uint8_t crc_mode_idx_tx = 3;  // default to INITFF_XORFF (common-ish)

// Target control
static uint16_t q_target = Q_TARGET_NEUTRAL;
static uint8_t  q_target_qf = 0; // 0 ignore, 1 active

// Ramp tuning
static uint8_t ramp_step_per_tick = 1;  // counts per 10ms tick
static uint16_t manual_step = 4;        // counts per keypress (+/-)

// Counters required by iBooster safety logic
static uint8_t cnt_100hz = 0; // for 0x38C and 0x38D (coupled)
static uint8_t cnt_50hz  = 0; // for 0x38B

// Bus routing (super useful when wiring is ambiguous)
enum BusSel : uint8_t { BUS_VEH = 0, BUS_YAW = 1 };
static BusSel tx_bus = BUS_YAW;
static BusSel rx_bus = BUS_YAW;

// Timing
static uint32_t last_10ms = 0;
static uint32_t last_20ms = 0;
static uint32_t last_dash = 0;

// Diagnostics
struct {
  uint32_t rx_count = 0;
  uint32_t tx_count = 0;
  uint32_t tx_fail  = 0;

  uint32_t crc_err_using_selected = 0;

  // CRC auto-score: how many RX frames matched each CRC mode
  uint32_t crc_ok_score[CRC_MODE_COUNT] = {0};

  uint16_t rod_pos = 0;
  uint8_t  ibst_status = 0;
  bool     driver_brake = 0;
} diag;

// -------------------------- Helpers --------------------------
MCP_CAN& CAN_TX() { return (tx_bus == BUS_YAW) ? CAN_YAW : CAN_VEH; }
MCP_CAN& CAN_RX() { return (rx_bus == BUS_YAW) ? CAN_YAW : CAN_VEH; }

uint8_t crc8_j1850_variant(const uint8_t *data, uint8_t len, uint8_t init, uint8_t xorout) {
  uint8_t crc = init;
  // compute over bytes 1..len-1, store in byte0
  for (uint8_t i = 1; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ CRC_POLY);
      else           crc = (uint8_t)(crc << 1);
    }
  }
  crc ^= xorout;
  return crc;
}

uint8_t calc_crc_tx(uint8_t *data, uint8_t len) {
  const CrcMode &m = CRC_MODES[crc_mode_idx_tx];
  return crc8_j1850_variant(data, len, m.init, m.xorout);
}

bool check_crc_with_mode(const uint8_t *data, uint8_t len, uint8_t mode_idx) {
  uint8_t received = data[0];
  uint8_t calc = crc8_j1850_variant(data, len, CRC_MODES[mode_idx].init, CRC_MODES[mode_idx].xorout);
  return received == calc;
}

void print_help() {
  Serial.println();
  Serial.println(F("=== iBooster Bench Controller Commands ==="));
  Serial.println(F("h  : help"));
  Serial.println(F("a  : ARM/DISARM (must be armed to command)"));
  Serial.println(F("!  : EMERGENCY STOP (disarm + neutral)"));
  Serial.println(F("i  : toggle ignition flag bit (0x38B)"));
  Serial.println(F("b  : ramp BRAKE toward max"));
  Serial.println(F("r  : ramp RELEASE toward neutral"));
  Serial.println(F("x  : neutral (stop ramps)"));
  Serial.println(F("+  : nudge target up (more brake)"));
  Serial.println(F("-  : nudge target down (less brake)"));
  Serial.println(F("[  : ramp step --"));
  Serial.println(F("]  : ramp step ++"));
  Serial.println(F("v  : verbose RX dump toggle"));
  Serial.println(F("t  : toggle TX bus (YAW/VEH)"));
  Serial.println(F("y  : toggle RX bus (YAW/VEH)"));
  Serial.println(F("c  : cycle TX CRC mode"));
  Serial.println(F("n  : toggle Neutral holds QF"));
  Serial.println(F("m  : print current config"));
  Serial.println();
}

void print_config() {
  Serial.println();
  Serial.println(F("=== Config ==="));
  Serial.print(F("ARMED: ")); Serial.println(armed ? F("YES") : F("NO"));
  Serial.print(F("Ignition flag: ")); Serial.println(ignition_status ? F("ON") : F("OFF"));
  Serial.print(F("Mode: "));
  if (mode == MODE_NEUTRAL) Serial.println(F("NEUTRAL"));
  else if (mode == MODE_RAMP_BRAKE) Serial.println(F("RAMP_BRAKE"));
  else Serial.println(F("RAMP_RELEASE"));

  Serial.print(F("TX bus: ")); Serial.println(tx_bus == BUS_YAW ? F("YAW") : F("VEH"));
  Serial.print(F("RX bus: ")); Serial.println(rx_bus == BUS_YAW ? F("YAW") : F("VEH"));

  Serial.print(F("Neutral holds QF: ")); Serial.println(neutral_holds_qf ? F("YES") : F("NO"));
  Serial.print(F("Ramp step/tick: ")); Serial.println(ramp_step_per_tick);
  Serial.print(F("Manual step: ")); Serial.println(manual_step);

  Serial.print(F("TX CRC mode: ")); Serial.print(crc_mode_idx_tx);
  Serial.print(F(" (")); Serial.print(CRC_MODES[crc_mode_idx_tx].name); Serial.println(F(")"));

  // Show best-matching CRC mode based on RX scoring
  uint8_t best = 0;
  for (uint8_t k = 1; k < CRC_MODE_COUNT; k++) {
    if (diag.crc_ok_score[k] > diag.crc_ok_score[best]) best = k;
  }
  Serial.print(F("RX CRC best score: ")); Serial.print(best);
  Serial.print(F(" (")); Serial.print(CRC_MODES[best].name); Serial.print(F(")  score="));
  Serial.println(diag.crc_ok_score[best]);
  Serial.println();
}

void safe_stop() {
  armed = false;
  mode = MODE_NEUTRAL;
  q_target = Q_TARGET_NEUTRAL;
  q_target_qf = 0;
  Serial.println(F("!! E-STOP: DISARM + NEUTRAL"));
}

// -------------------------- CAN Frame Builders --------------------------
void send_38D_driver_request(uint8_t counter) {
  uint8_t d[8] = {0};

  d[1] = counter;
  d[2] = 0;  // P_TARGET_DRIVER low
  d[3] = 0;  // P_TARGET_DRIVER high
  d[4] = 0;
  d[5] = 0;  // P_MC low
  d[6] = 0x20; // (P_MC_QF << 5) | (P_MC >> 5) -> QF=1 => 0x20
  d[7] = 0;

  d[0] = calc_crc_tx(d, 8);

  byte r = CAN_TX().sendMsgBuf(0x38D, 0, 8, d);
  diag.tx_count++;
  if (r != CAN_OK) diag.tx_fail++;
}

void send_38C_target_request(uint8_t counter, uint16_t q, uint8_t qf) {
  uint8_t d[8] = {0};

  uint16_t p_lim = (uint16_t)(P_LIMIT_EXTERNAL * 2);

  d[1] = counter;
  d[2] = (uint8_t)(p_lim & 0xFF);
  d[3] = (uint8_t)(((p_lim >> 8) & 0x01) | ((q & 0x0F) << 4));
  d[4] = (uint8_t)((q >> 4) & 0xFF);
  d[5] = (uint8_t)(((q >> 12) & 0x0F) | ((qf & 0x01) << 4));
  d[6] = 0;
  d[7] = 0;

  d[0] = calc_crc_tx(d, 8);

  byte r = CAN_TX().sendMsgBuf(0x38C, 0, 8, d);
  diag.tx_count++;
  if (r != CAN_OK) diag.tx_fail++;
}

void send_38B_vehicle_status(uint8_t counter) {
  uint8_t d[8] = {0};

  d[1] = counter;
  d[2] = P_EST_MAX;
  d[3] = (uint8_t)(P_EST_MAX_QF); // speed low bits assumed 0
  d[4] = 0x00;                    // speed high
  d[5] = (uint8_t)(VEHICLE_QF | ((ignition_status ? 1 : 0) << 3));
  d[6] = 0;
  d[7] = 0;

  d[0] = calc_crc_tx(d, 8);

  byte r = CAN_TX().sendMsgBuf(0x38B, 0, 8, d);
  diag.tx_count++;
  if (r != CAN_OK) diag.tx_fail++;
}

// -------------------------- Receive/Decode --------------------------
void read_incoming_nonblocking() {
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  while (CAN_RX().checkReceive() == CAN_MSGAVAIL) {
    CAN_RX().readMsgBuf(&rxId, &len, rxBuf);
    diag.rx_count++;

    // CRC autoscore (only meaningful if len==8 and CRC lives in byte0)
    if (len == 8) {
      for (uint8_t k = 0; k < CRC_MODE_COUNT; k++) {
        if (check_crc_with_mode(rxBuf, len, k)) diag.crc_ok_score[k]++;
      }
      // also track "selected mode" mismatch rate
      if (!check_crc_with_mode(rxBuf, len, crc_mode_idx_tx)) diag.crc_err_using_selected++;
    }

    // Decode interesting messages
    if (rxId == 0x38E && len >= 5) {
      // rod_pos = ((dat[4] & 0x0F) << 8) | dat[3]
      diag.rod_pos = (uint16_t)(((rxBuf[4] & 0x0F) << 8) | rxBuf[3]);
    } else if (rxId == 0x38F && len >= 8) {
      diag.driver_brake = (rxBuf[2] & 0x01);
      diag.ibst_status  = (uint8_t)((rxBuf[2] >> 3) & 0x07);
    }

    if (verbose_mode) {
      Serial.print(F("RX 0x"));
      Serial.print(rxId, HEX);
      Serial.print(F(" ["));
      for (uint8_t i = 0; i < len; i++) {
        if (rxBuf[i] < 0x10) Serial.print('0');
        Serial.print(rxBuf[i], HEX);
        Serial.print(' ');
      }
      Serial.println(F("]"));
    }
  }
}

// -------------------------- Serial Handling --------------------------
void handle_serial(uint32_t now) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    last_user_activity_ms = now;

    switch (c) {
      case 'h': print_help(); break;

      case 'a':
        armed = !armed;
        Serial.print(F("ARMED: ")); Serial.println(armed ? F("YES") : F("NO"));
        if (!armed) { mode = MODE_NEUTRAL; q_target = Q_TARGET_NEUTRAL; q_target_qf = 0; }
        break;

      case '!': safe_stop(); break;

      case 'i':
        ignition_status = !ignition_status;
        Serial.print(F("Ignition flag: ")); Serial.println(ignition_status ? F("ON") : F("OFF"));
        break;

      case 'b':
        mode = MODE_RAMP_BRAKE;
        Serial.println(F("MODE: RAMP BRAKE"));
        break;

      case 'r':
        mode = MODE_RAMP_RELEASE;
        Serial.println(F("MODE: RAMP RELEASE"));
        break;

      case 'x':
        mode = MODE_NEUTRAL;
        q_target = Q_TARGET_NEUTRAL;
        Serial.println(F("MODE: NEUTRAL"));
        break;

      case '+':
        // more brake = q up (toward max)
        q_target = (uint16_t)min<uint32_t>((uint32_t)q_target + manual_step, Q_TARGET_MAX);
        Serial.print(F("Manual Q: 0x")); Serial.println(q_target, HEX);
        break;

      case '-':
        q_target = (uint16_t)max<int32_t>((int32_t)q_target - (int32_t)manual_step, (int32_t)Q_TARGET_MIN);
        Serial.print(F("Manual Q: 0x")); Serial.println(q_target, HEX);
        break;

      case '[':
        if (ramp_step_per_tick > 1) ramp_step_per_tick--;
        Serial.print(F("Ramp step: ")); Serial.println(ramp_step_per_tick);
        break;

      case ']':
        if (ramp_step_per_tick < 50) ramp_step_per_tick++;
        Serial.print(F("Ramp step: ")); Serial.println(ramp_step_per_tick);
        break;

      case 'v':
        verbose_mode = !verbose_mode;
        Serial.print(F("Verbose: ")); Serial.println(verbose_mode ? F("ON") : F("OFF"));
        break;

      case 't':
        tx_bus = (tx_bus == BUS_YAW) ? BUS_VEH : BUS_YAW;
        Serial.print(F("TX bus: ")); Serial.println(tx_bus == BUS_YAW ? F("YAW") : F("VEH"));
        break;

      case 'y':
        rx_bus = (rx_bus == BUS_YAW) ? BUS_VEH : BUS_YAW;
        Serial.print(F("RX bus: ")); Serial.println(rx_bus == BUS_YAW ? F("YAW") : F("VEH"));
        break;

      case 'c':
        crc_mode_idx_tx = (uint8_t)((crc_mode_idx_tx + 1) % CRC_MODE_COUNT);
        Serial.print(F("TX CRC mode: ")); Serial.print(crc_mode_idx_tx);
        Serial.print(F(" (")); Serial.print(CRC_MODES[crc_mode_idx_tx].name); Serial.println(F(")"));
        break;

      case 'n':
        neutral_holds_qf = !neutral_holds_qf;
        Serial.print(F("Neutral holds QF: ")); Serial.println(neutral_holds_qf ? F("YES") : F("NO"));
        break;

      case 'm':
        print_config();
        break;

      default:
        // ignore unknown keys
        break;
    }
  }
}

// -------------------------- Setup --------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {;}

  Serial.println(F("--- iBooster Bench Controller (All-in-One) ---"));
  Serial.println(F("Init MCP2515s..."));

  if (CAN_VEH.begin(MCP_ANY, CAN_BAUD, CAN_CLK) == CAN_OK) {
    Serial.println(F("CAN VEH: OK"));
    CAN_VEH.setMode(MCP_NORMAL);
  } else {
    Serial.println(F("CAN VEH: FAILED"));
  }

  if (CAN_YAW.begin(MCP_ANY, CAN_BAUD, CAN_CLK) == CAN_OK) {
    Serial.println(F("CAN YAW: OK"));
    CAN_YAW.setMode(MCP_NORMAL);
  } else {
    Serial.println(F("CAN YAW: FAILED"));
  }

  last_user_activity_ms = millis();
  print_help();
  print_config();
}

// -------------------------- Loop --------------------------
void loop() {
  uint32_t now = millis();

  // 1) Always service serial + RX as fast as possible
  handle_serial(now);
  read_incoming_nonblocking();

  // 2) Auto-disarm on idle (bench safety)
  if (armed && (now - last_user_activity_ms > IDLE_DISARM_MS)) {
    Serial.println(F("Idle timeout -> DISARM + NEUTRAL"));
    armed = false;
    mode = MODE_NEUTRAL;
    q_target = Q_TARGET_NEUTRAL;
    q_target_qf = 0;
  }

  // 3) Update control logic
  if (!armed) {
    // disarmed => neutral, QF off
    q_target = Q_TARGET_NEUTRAL;
    q_target_qf = 0;
    mode = MODE_NEUTRAL; // force
  } else {
    // armed => choose QF behavior + ramp logic
    if (mode == MODE_RAMP_BRAKE) {
      q_target_qf = 1;
      if (q_target < Q_TARGET_MAX_BRAKE) {
        uint32_t next = (uint32_t)q_target + ramp_step_per_tick;
        q_target = (uint16_t)min<uint32_t>(next, Q_TARGET_MAX_BRAKE);
      }
    } else if (mode == MODE_RAMP_RELEASE) {
      q_target_qf = 1;
      if (q_target > Q_TARGET_NEUTRAL) {
        int32_t next = (int32_t)q_target - (int32_t)ramp_step_per_tick;
        q_target = (uint16_t)max<int32_t>(next, (int32_t)Q_TARGET_NEUTRAL);
      } else {
        q_target = Q_TARGET_NEUTRAL;
      }
    } else { // neutral
      q_target = Q_TARGET_NEUTRAL;
      q_target_qf = neutral_holds_qf ? 1 : 0;
    }
  }

  // 4) 100Hz tick (10ms): send 0x38D + 0x38C with coupled counter
  if ((uint32_t)(now - last_10ms) >= 10) {
    last_10ms += 10; // reduces drift vs now=millis
    send_38D_driver_request(cnt_100hz);
    send_38C_target_request(cnt_100hz, q_target, q_target_qf);
    cnt_100hz = (uint8_t)((cnt_100hz + 1) & 0x0F);
  }

  // 5) 50Hz tick (20ms): send 0x38B
  if ((uint32_t)(now - last_20ms) >= 20) {
    last_20ms += 20;
    send_38B_vehicle_status(cnt_50hz);
    cnt_50hz = (uint8_t)((cnt_50hz + 1) & 0x0F);
  }

  // 6) Dashboard (250ms)
  if ((uint32_t)(now - last_dash) >= 250) {
    last_dash = now;

    // best CRC mode based on RX scoring
    uint8_t best = 0;
    for (uint8_t k = 1; k < CRC_MODE_COUNT; k++) {
      if (diag.crc_ok_score[k] > diag.crc_ok_score[best]) best = k;
    }

    Serial.print(F("ARM ")); Serial.print(armed ? 1 : 0);
    Serial.print(F(" | Ign ")); Serial.print(ignition_status ? 1 : 0);
    Serial.print(F(" | Stat ")); Serial.print(diag.ibst_status);
    Serial.print(F(" | Rod ")); Serial.print(diag.rod_pos);
    Serial.print(F(" | Drv ")); Serial.print(diag.driver_brake ? 1 : 0);
    Serial.print(F(" | Q 0x")); Serial.print(q_target, HEX);
    Serial.print(F(" QF ")); Serial.print(q_target_qf);

    Serial.print(F(" | TXbus ")); Serial.print(tx_bus == BUS_YAW ? F("Y") : F("V"));
    Serial.print(F(" RXbus ")); Serial.print(rx_bus == BUS_YAW ? F("Y") : F("V"));

    Serial.print(F(" | TXCRC ")); Serial.print(CRC_MODES[crc_mode_idx_tx].name);
    Serial.print(F(" | RXbest ")); Serial.print(CRC_MODES[best].name);

    Serial.print(F(" | RX# ")); Serial.print(diag.rx_count);
    Serial.print(F(" TX# ")); Serial.print(diag.tx_count);
    Serial.print(F(" TXfail ")); Serial.print(diag.tx_fail);
    Serial.print(F(" SelCRCErr ")); Serial.print(diag.crc_err_using_selected);
    Serial.println();
  }
}
