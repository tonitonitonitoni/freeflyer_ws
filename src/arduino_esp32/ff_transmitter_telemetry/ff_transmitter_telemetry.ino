// ===============================
// ff_transmitter.ino
// Serial -> RF24 bridge with Telemetry
// ===============================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "protocol.h"
// ---------------- RF ----------------
#define CE_PIN 9
#define CSN_PIN 10
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "FFCMD";

// ---------------- Serial ----------------
#define SERIAL_BUFFER_SIZE 128
#define WATCHDOG_TIMEOUT_MS 200

unsigned long last_cmd_time = 0;
char serial_buffer[SERIAL_BUFFER_SIZE];
int buffer_index = 0;

// ----------------------------------------------------

void printTelemetry(const TelemetryPacket &t)
{
  Serial.print("$TEL,");
  Serial.print(t.packet_counter); Serial.print(",");
  Serial.print(t.last_rx_ms); Serial.print(",");
  Serial.print(t.fwd); Serial.print(",");
  Serial.print(t.rev); Serial.print(",");
  Serial.print(t.lft); Serial.print(",");
  Serial.print(t.rgt); Serial.print(",");
  Serial.print(t.rw_state); Serial.print(",");
  Serial.print(t.watchdog_active); Serial.print(",");
  Serial.println();
  
}

// ----------------------------------------------------

void sendPacket(const CommandPacket &pkt)
{
  bool ok = radio.write(&pkt, sizeof(pkt));

  if (!ok) {
    Serial.println("RF Send Failed");
    return;
  }

  if (radio.isAckPayloadAvailable()) {
    TelemetryPacket telem;
    radio.read(&telem, sizeof(telem));
    printTelemetry(telem);
  }
  else {
    Serial.println("No ACK payload");
  }
}

// ----------------------------------------------------

void sendNeutral()
{
  CommandPacket pkt = {0, 0, 0, 0, 0};
  sendPacket(pkt);
}

// ----------------------------------------------------

void sendCommand(bool fwd, bool rev, bool lft, bool rgt, int rw)
{
  CommandPacket pkt;

  pkt.fwd = fwd;
  pkt.rev = rev;
  pkt.lft = lft;
  pkt.rgt = rgt;
  pkt.rw  = (rw > 0) ? 1 : (rw < 0 ? -1 : 0);

  sendPacket(pkt);
}


// ----------------------------------------------------
void parseLine(char* line)
{
  if (strncmp(line, "$CMD,", 5) != 0)
    return;

  int fwd, rev, lft, rgt;
  int rw;

  int parsed = sscanf(
    line,
    "$CMD,%d,%d,%d,%d,%d",
    &fwd, &rev, &lft, &rgt, &rw);

  if (parsed == 5) {

    // Convert strictly to boolean (nonzero = true)
    bool bfwd = (fwd != 0);
    bool brev = (rev != 0);
    bool blft = (lft != 0);
    bool brgt = (rgt != 0);

    sendCommand(bfwd, brev, blft, brgt, rw);
    last_cmd_time = millis();
  }
}

// ----------------------------------------------------

void setup()
{
  Serial.begin(115200);

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setRetries(5, 15);
  radio.setAutoAck(true);
  radio.enableAckPayload();

  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println("FF Transmitter Ready with Telemetry");
}

// ----------------------------------------------------

void loop()
{
  // ---- Serial input ----
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      serial_buffer[buffer_index] = '\0';
      parseLine(serial_buffer);
      buffer_index = 0;
    } else {
      if (buffer_index < SERIAL_BUFFER_SIZE - 1)
        serial_buffer[buffer_index++] = c;
    }
  }

  // ---- Watchdog ----
  if (millis() - last_cmd_time > WATCHDOG_TIMEOUT_MS) {
    sendNeutral();
  }
}
