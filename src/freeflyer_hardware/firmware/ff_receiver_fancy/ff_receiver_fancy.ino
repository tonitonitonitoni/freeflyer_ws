#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "ESC.h"
#include "protocol.h"

const int rwmotorCW {6};
const int rwmotorCCW {7};
const int LED_PIN {13};

// ---------------- RF ----------------
const int CE_PIN {9};
const int CSN_PIN {10};
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "FFCMD";

enum ThrusterID {
  FWD = 0,
  REV,
  LFT,
  RGT,
  NUM_THRUSTERS
};

const int PINS[NUM_THRUSTERS] {2, 3, 4, 5};
const int PWM_MIN[NUM_THRUSTERS] {1200, 1200, 1200, 1200};
const int PWM_SAFE_MAX[NUM_THRUSTERS] {1500, 1500, 1500, 1500};
const int PWM_OFF {1000};


struct Thruster {
  ESC esc;
  int pwm_off;
  int pwm_min;
  int pwm_max_safe;

  Thruster(uint8_t pin,
           int off,
           int min_pwm,
           int max_pwm)
  : esc(pin, 1000, 2000, 500),
    pwm_off(off),
    pwm_min(min_pwm),
    pwm_max_safe(max_pwm)
  {}

  int normalizedToPWM(uint16_t val)
  {
    if (val == 0)
      return pwm_off;

    if (val > 1000)
      val = 1000;

    float u = val / 1000.0f;
    return pwm_min + (int)((pwm_max_safe - pwm_min) * u);
  }

  uint16_t setNormalized(uint16_t val)
  {
    uint16_t pwm = normalizedToPWM(val);
    esc.speed(pwm);
    return pwm;
  }

  void setNeutral()
  {
    esc.speed(pwm_off);
  }

  void arm()
  {
    esc.arm();
  }
};


Thruster thrusters[NUM_THRUSTERS] = {
  for (int i = 0; i < NUM_THRUSTERS; i++)
  // pin,  OFF,  MIN,  MAX_SAFE
  Thruster(PINS[i], PWM_OFF, PWM_MIN[i], PWM_SAFE_MAX[i]),
};


void setNeutral()
{
  for (int i = 0; i < NUM_THRUSTERS; i++)
    thrusters[i].setNeutral();

  digitalWrite(rwmotorCW, LOW);
  digitalWrite(rwmotorCCW, LOW);
}

// ---------------- Timing ----------------
#define WATCHDOG_TIMEOUT_MS 400   // slightly relaxed
unsigned long last_packet_time = 0;

uint32_t packet_counter = 0;


void setup(){
  Serial.begin(115200);

  pinMode(rwmotorCW, OUTPUT);
  pinMode(rwmotorCCW, OUTPUT);

  for (int i = 0; i < NUM_THRUSTERS; i++)
  thrusters[i].arm();

  delay(1000);
  setNeutral();

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setRetries(5, 15);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println("FF Receiver Ready with Telemetry");
}

// ---------------- Loop ----------------
void loop()
{
  static uint16_t last_pwm[NUM_THRUSTERS];
  static int8_t   last_rw_state = 0;
  static bool watchdog_active = false;

  if (radio.available()) {

    CommandPacket pkt;
    radio.read(&pkt, sizeof(pkt));

    packet_counter++;
    last_packet_time = millis();
    watchdog_active = false;

    // ---- Apply thruster commands ----
    uint16_t inputs[NUM_THRUSTERS] = {
      pkt.fwd,
      pkt.rev,
      pkt.lft,
      pkt.rgt
    };

    for (int i = 0; i < NUM_THRUSTERS; i++) {
      last_pwm[i] = thrusters[i].setNormalized(inputs[i]);
    }

    // ---- Reaction wheel ----
    last_rw_state = pkt.rw;

    if (pkt.rw > 0) {
      digitalWrite(rwmotorCW, HIGH);
      digitalWrite(rwmotorCCW, LOW);
    }
    else if (pkt.rw < 0) {
      digitalWrite(rwmotorCW, LOW);
      digitalWrite(rwmotorCCW, HIGH);
    }
    else {
      digitalWrite(rwmotorCW, LOW);
      digitalWrite(rwmotorCCW, LOW);
    }

    // ---- Build telemetry ----
    TelemetryPacket telem;
    telem.packet_counter  = packet_counter;
    telem.last_rx_ms      = millis();
    telem.pwm_fwd         = last_pwm[FWD];
    telem.pwm_rev         = last_pwm[REV];
    telem.pwm_lft         = last_pwm[LFT];
    telem.pwm_rgt         = last_pwm[RGT];
    telem.rw_state        = last_rw_state;
    telem.watchdog_active = 0;

    radio.writeAckPayload(0, &telem, sizeof(telem));
  }

  // ---- Watchdog ----
  if (millis() - last_packet_time > WATCHDOG_TIMEOUT_MS) {
    if (!watchdog_active) {
      setNeutral();
      watchdog_active = true;
    }
  }
}
