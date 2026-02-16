#pragma once
#include <stdint.h>
#include <RF24.h>
// Must match on both ends exactly

struct CommandPacket {
  uint16_t fwd;
  uint16_t rev;
  uint16_t lft;
  uint16_t rgt;
  int8_t   rw;
};

struct TelemetryPacket {
  uint32_t packet_counter;
  uint32_t last_rx_ms;
  uint16_t pwm_fwd;
  uint16_t pwm_rev;
  uint16_t pwm_lft;
  uint16_t pwm_rgt;
  int8_t   rw_state;
  uint8_t  watchdog_active;
};
