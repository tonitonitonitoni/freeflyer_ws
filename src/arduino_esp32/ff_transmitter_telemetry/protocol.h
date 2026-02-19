#pragma once
#include <stdint.h>
#include <RF24.h>
// Must match on both ends exactly

struct CommandPacket {
  bool     fwd;
  bool     rev;
  bool     lft;
  bool     rgt;
  int8_t   rw;
};

struct TelemetryPacket {
  uint32_t packet_counter;
  uint32_t last_rx_ms;
  bool     fwd;
  bool     rev;
  bool     lft;
  bool     rgt;
  int8_t   rw_state;
  bool     watchdog_active;
};
