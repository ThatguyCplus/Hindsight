/*
  iBooster "EXTERNAL_BRAKE_REQUEST.csv" exact mimic
  Mega2560 + MCP2515 (Yaw CAN @ 500kbps)

  Every 10ms send:
    0x38B: [CRC] [0x40|cnt] 00 05
    0x38C: [CRC] [0x40|cnt] 00 83   (flow_rate = 0x8300)
    0x38D: [CRC] [cnt]      00 00 00 50 47

  CRC8 SAE J1850: poly=0x1D, init=0xFF, xorout=0xFF
  CRC computed over bytes AFTER the CRC byte only.
*/

#include <SPI.h>
#include <mcp2515.h>

#define CS_PIN     4
#define CAN_SPEED  CAN_500KBPS
#define MCP_CLOCK  MCP_8MHZ   // change to MCP_16MHZ if your MCP2515 crystal is 16MHz

MCP2515 can(CS_PIN);

static uint8_t cnt = 0;  // 0..15
static uint32_t next_tick_us = 0;
static const uint32_t PERIOD_US = 10000; // 10ms

uint8_t crc8_j1850(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x1D);
      else           crc = (uint8_t)(crc << 1);
    }
  }
  return (uint8_t)(crc ^ 0xFF);
}

static inline void send_frame(uint16_t id, const uint8_t *data, uint8_t dlc) {
  struct can_frame frame;
  frame.can_id = id;
  frame.can_dlc = dlc;
  for (uint8_t i = 0; i < dlc; i++) frame.data[i] = data[i];
  can.sendMessage(&frame);
}

void setup() {
  SPI.begin();
  can.reset();
  can.setBitrate(CAN_SPEED, MCP_CLOCK);
  can.setNormalMode();
  next_tick_us = micros();
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - next_tick_us) >= 0) {
    next_tick_us += PERIOD_US;

    // --- 0x38B ---
    {
      uint8_t payload[3] = { (uint8_t)(0x40 | (cnt & 0x0F)), 0x00, 0x05 };
      uint8_t out[4] = { crc8_j1850(payload, 3), payload[0], payload[1], payload[2] };
      send_frame(0x38B, out, 4);
    }

    // --- 0x38C (flow 0x8300 -> bytes 00 83) ---
    {
      uint8_t payload[3] = { (uint8_t)(0x40 | (cnt & 0x0F)), 0x00, 0x83 };
      uint8_t out[4] = { crc8_j1850(payload, 3), payload[0], payload[1], payload[2] };
      send_frame(0x38C, out, 4);
    }

    // --- 0x38D ---
    {
      uint8_t payload[6] = { (uint8_t)(cnt & 0x0F), 0x00, 0x00, 0x00, 0x50, 0x47 };
      uint8_t out[7] = { crc8_j1850(payload, 6), payload[0], payload[1], payload[2], payload[3], payload[4], payload[5] };
      send_frame(0x38D, out, 7);
    }

    cnt = (uint8_t)((cnt + 1) & 0x0F);
  }
}
