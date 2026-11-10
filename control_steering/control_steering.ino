// Actuator board for the ramp: steering actuator + jack, each driven by a pair
// of relays wired to reverse polarity.
//
// The host sends the complete desired state in one packet, many times a second.
// That stream is also the heartbeat: if it stops, every output drops.
//
//   host -> board (6 bytes):  0xA5, steer, jack, seq, reserved, crc8
//   board -> host (8 bytes):  0x5A, version, steer, jack, flags, seq, wd_trips, crc8
//
// steer: +1 right, -1 left, 0 off.   jack: +1 up, -1 down, 0 off.
// crc8 (poly 0x07) covers everything between the start byte and the crc.
// The same constants live in control_steering.py -- change them together.

const uint8_t FW_VERSION = 2;

const uint8_t PIN_RIGHT = A9, PIN_LEFT = A8;
const uint8_t PIN_UP = A6, PIN_DOWN = A7;
const uint8_t PIN_LED = 13;

const uint32_t HEARTBEAT_TIMEOUT_MS = 250;   // no valid packet for this long -> all outputs off
const uint32_t REVERSE_DEADTIME_MS = 100;    // both relays off this long before reversing
const uint32_t STATUS_PERIOD_MS = 20;
// Longest one continuous push in a single direction. The steering actuator has
// no limit switches, so this bounds a stuck command. Release and press again to
// keep going.
const uint32_t STEER_MAX_ON_MS = 3000;
const uint32_t JACK_MAX_ON_MS = 60000;

const uint8_t CMD_START = 0xA5, CMD_LEN = 6;
const uint8_t STATUS_START = 0x5A, STATUS_LEN = 8;
const uint8_t FLAG_TIMED_OUT = 1, FLAG_STEER_CAPPED = 2, FLAG_JACK_CAPPED = 4;

struct Axis {
  uint8_t pinPos, pinNeg;
  uint32_t maxOnMs;
  int8_t want;        // last commanded direction
  int8_t out;         // direction actually being driven
  int8_t lastDir;     // direction of the most recent drive
  bool capped;
  uint32_t onSince, offSince;
};

Axis steer = {PIN_RIGHT, PIN_LEFT, STEER_MAX_ON_MS, 0, 0, 0, false, 0, 0};
Axis jack = {PIN_UP, PIN_DOWN, JACK_MAX_ON_MS, 0, 0, 0, false, 0, 0};

uint8_t rx[CMD_LEN];
uint8_t rxLen = 0;
uint8_t lastSeq = 0;
uint8_t wdTrips = 0;
bool timedOut = true;            // nothing moves until the first valid packet
uint32_t lastCmdMs = 0;
uint32_t lastStatusMs = 0;

uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

bool validDir(int8_t d) { return d >= -1 && d <= 1; }

void axisOff(Axis &a, uint32_t now) {
  digitalWrite(a.pinPos, LOW);
  digitalWrite(a.pinNeg, LOW);
  if (a.out != 0) {
    a.out = 0;
    a.offSince = now;
  }
}

void axisUpdate(Axis &a, uint32_t now) {
  int8_t want = a.want;
  if (a.capped) {
    if (want == 0) a.capped = false;     // released: allow the next push
    want = 0;
  } else if (a.out != 0 && want == a.out && now - a.onSince >= a.maxOnMs) {
    a.capped = true;
    want = 0;
  }
  if (want == a.out) return;
  axisOff(a, now);
  if (want == 0) return;
  // Reversing: wait with both relays released so they never overlap.
  if (want != a.lastDir && now - a.offSince < REVERSE_DEADTIME_MS) return;
  digitalWrite(want > 0 ? a.pinPos : a.pinNeg, HIGH);
  a.out = want;
  a.lastDir = want;
  a.onSince = now;
}

void readCommands(uint32_t now) {
  while (Serial.available()) {
    rx[rxLen++] = Serial.read();
    if (rxLen < CMD_LEN) continue;
    int8_t s = (int8_t)rx[1], j = (int8_t)rx[2];
    if (rx[0] == CMD_START && crc8(rx + 1, CMD_LEN - 2) == rx[CMD_LEN - 1] && validDir(s) && validDir(j)) {
      steer.want = s;
      jack.want = j;
      lastSeq = rx[3];
      lastCmdMs = now;
      timedOut = false;
      rxLen = 0;
    } else {
      // not a packet: slide one byte and keep looking
      memmove(rx, rx + 1, CMD_LEN - 1);
      rxLen = CMD_LEN - 1;
    }
  }
}

void sendStatus() {
  if (Serial.availableForWrite() < STATUS_LEN) return;   // never block on a host that isn't reading
  uint8_t tx[STATUS_LEN];
  tx[0] = STATUS_START;
  tx[1] = FW_VERSION;
  tx[2] = (uint8_t)steer.out;
  tx[3] = (uint8_t)jack.out;
  tx[4] = (timedOut ? FLAG_TIMED_OUT : 0) | (steer.capped ? FLAG_STEER_CAPPED : 0) | (jack.capped ? FLAG_JACK_CAPPED : 0);
  tx[5] = lastSeq;
  tx[6] = wdTrips;
  tx[7] = crc8(tx + 1, STATUS_LEN - 2);
  Serial.write(tx, STATUS_LEN);
}

void setup() {
  const uint8_t outputs[] = {PIN_RIGHT, PIN_LEFT, PIN_UP, PIN_DOWN};
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(outputs[i], LOW);
    pinMode(outputs[i], OUTPUT);
  }
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  uint32_t now = millis();
  readCommands(now);

  if (!timedOut && now - lastCmdMs > HEARTBEAT_TIMEOUT_MS) {
    timedOut = true;
    if (wdTrips < 255) wdTrips++;
  }
  if (timedOut) {
    steer.want = 0;
    jack.want = 0;
  }
  axisUpdate(steer, now);
  axisUpdate(jack, now);

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    sendStatus();
  }
  // slow blink = host is talking, fast blink = no heartbeat
  digitalWrite(PIN_LED, (now / (timedOut ? 100 : 500)) % 2);
}
