/*
 * MINEVOLVER — Arduino Nano (ATmega328P) port
 *
 * RAM budget: 2048 bytes.  No heap allocation; fixed char[] buffers only.
 * Handles the MEV provisioning protocol (WHO_ARE_YOU / PROVISION / CLEAR_ID)
 * plus minimal od_90, temp, stir, pump sensor responses.
 *
 * Identity is RAM-only on AVR (lost on reboot); the server must re-provision
 * after each power cycle until a flash-backed port is implemented.
 */

#include <PID_v1.h>
#include "identity.h"

// ---- Serial input buffer ------------------------------------------------
#define INPUT_BUF_SIZE 200
static char input_buf[INPUT_BUF_SIZE];
static uint8_t input_len = 0;
static bool string_complete = false;

// ---- Device config ------------------------------------------------------
static const uint8_t NUM_VIALS  = 2;
static const uint8_t NUM_PUMPS  = 6;

static uint32_t mev_seq = 0;

// ---- Sensor state -------------------------------------------------------
static int pd_pin[]       = {A2, A3};
static int temp_pin[]     = {A0, A1};
static int pd_output[]    = {0, 0};
static uint8_t active_vial = 0;
static uint8_t pd_avg      = 20;

static double tempInput[NUM_VIALS]    = {0};
static double tempOutput[NUM_VIALS]   = {0};
static double tempSetpoint[NUM_VIALS] = {60000, 60000};
static int    tempOutputPin[]         = {2, 3};
static int    temp_saved[]            = {255, 255};
static bool   temp_new_input          = false;

PID pid1(&tempInput[0], &tempOutput[0], &tempSetpoint[0], 6500, 20, 2, DIRECT);
PID pid2(&tempInput[1], &tempOutput[1], &tempSetpoint[1], 6500, 20, 2, DIRECT);
PID *allPIDs[NUM_VIALS] = {&pid1, &pid2};

static int stir_saved[]    = {0, 0};
static bool stir_new_input = false;
static int stirPin[]       = {5, 6};

// Pumps — store (timeToPump_ms, interval_ms, running, start_ms) per pump
static int pumpPin[] = {A4, 7, 8, 9, 10, 12};
struct PumpState {
  unsigned long start_ms;
  unsigned long on_ms;
  unsigned long period_ms;
  bool running;
};
static PumpState pumps[NUM_PUMPS];

// ---- Helpers ------------------------------------------------------------

static bool buf_starts(const char *prefix) {
  return strncmp(input_buf, prefix, strlen(prefix)) == 0;
}

// Parse comma-separated fields from input_buf after the address prefix.
// fields[] must be large enough. Returns count of fields found.
static uint8_t parse_fields(const char *addr, char fields[][24], uint8_t max_fields) {
  uint8_t alen = strlen(addr);
  // skip "addr" then one char for r/i/b/e
  const char *p = input_buf + alen + 1;
  uint8_t n = 0;
  while (n < max_fields) {
    const char *end = strchr(p, ',');
    if (!end) {
      // last token — stop before _!
      const char *term = strstr(p, "_!");
      if (!term) term = p + strlen(p);
      uint8_t len = term - p;
      if (len == 0 || strcmp(p, "end") == 0) break;
      if (len >= 24) len = 23;
      strncpy(fields[n], p, len);
      fields[n][len] = '\0';
      n++;
      break;
    }
    uint8_t len = end - p;
    if (len >= 24) len = 23;
    strncpy(fields[n], p, len);
    fields[n][len] = '\0';
    n++;
    p = end + 1;
  }
  return n;
}

static char get_cmd_char(const char *addr) {
  // returns the command char (r/i/a/b/e) immediately after the address name
  return input_buf[strlen(addr)];
}

// Emit "addr b, v0, v1, ..., end\n"
static void send_data(const char *addr, int *vals, uint8_t n) {
  Serial.print(addr);
  Serial.print(F("b,"));
  for (uint8_t i = 0; i < n; i++) {
    Serial.print(vals[i]);
    Serial.print(',');
  }
  Serial.println(F("end"));
}

// Emit echo "addr e, f0, f1, ..., end\n"
static void send_echo(const char *addr, char fields[][24], uint8_t n) {
  Serial.print(addr);
  Serial.print(F("e,"));
  for (uint8_t i = 0; i < n; i++) {
    Serial.print(fields[i]);
    Serial.print(',');
  }
  Serial.println(F("end"));
}

// ---- Pump helpers -------------------------------------------------------

static void pump_init(uint8_t i) {
  pumps[i] = {0, 0, 0, false};
  if (pumpPin[i] == 12) {
    pinMode(12, OUTPUT);
    digitalWrite(12, LOW);
  } else {
    analogWrite(pumpPin[i], 0);
  }
}

static void pump_set(uint8_t i, unsigned long on_ms, unsigned long period_ms) {
  pumps[i].on_ms     = on_ms;
  pumps[i].period_ms = period_ms;
  pumps[i].start_ms  = millis();
  pumps[i].running   = (on_ms > 0);
  if (pumps[i].running) {
    if (pumpPin[i] == 12) digitalWrite(12, HIGH);
    else analogWrite(pumpPin[i], 255);
  }
}

static void pump_update(uint8_t i) {
  if (!pumps[i].running && pumps[i].period_ms == 0) return;
  unsigned long now = millis();
  unsigned long elapsed = now - pumps[i].start_ms;

  if (pumps[i].running && elapsed >= pumps[i].on_ms) {
    pumps[i].running = false;
    if (pumpPin[i] == 12) digitalWrite(12, LOW);
    else analogWrite(pumpPin[i], 0);
  }
  if (!pumps[i].running && pumps[i].period_ms > 0 && elapsed >= pumps[i].period_ms) {
    pumps[i].start_ms = now;
    pumps[i].running  = true;
    if (pumpPin[i] == 12) digitalWrite(12, HIGH);
    else analogWrite(pumpPin[i], 255);
  }
}

// ---- Command handlers ---------------------------------------------------

static void handle_od90() {
  char cmd = get_cmd_char("od_90");
  if (cmd == 'r' || cmd == 'i') {
    send_data("od_90", pd_output, NUM_VIALS);
  }
}

static void handle_temp() {
  char cmd = get_cmd_char("temp");
  if (cmd == 'r' || cmd == 'i') {
    char fields[NUM_VIALS + 1][24];
    parse_fields("temp", fields, NUM_VIALS + 1);
    for (uint8_t i = 0; i < NUM_VIALS; i++)
      temp_saved[i] = atoi(fields[i + 1]);
    temp_new_input = true;
    int vals[NUM_VIALS];
    for (uint8_t i = 0; i < NUM_VIALS; i++) vals[i] = (int)tempInput[i];
    send_data("temp", vals, NUM_VIALS);
  } else if (cmd == 'a' && temp_new_input) {
    for (uint8_t i = 0; i < NUM_VIALS; i++)
      tempSetpoint[i] = (double)temp_saved[i];
    temp_new_input = false;
  }
}

static void handle_stir() {
  char cmd = get_cmd_char("stir");
  if (cmd == 'r' || cmd == 'i') {
    char fields[NUM_VIALS + 1][24];
    uint8_t n = parse_fields("stir", fields, NUM_VIALS + 1);
    for (uint8_t i = 0; i < NUM_VIALS && i + 1 < n; i++)
      stir_saved[i] = atoi(fields[i + 1]);
    stir_new_input = true;
    send_echo("stir", fields + 1, NUM_VIALS);
  } else if (cmd == 'a' && stir_new_input) {
    stir_new_input = false;
    for (uint8_t i = 0; i < NUM_VIALS; i++) {
      int speed = constrain(stir_saved[i] * 5, 0, 255);
      if (speed > 0) { analogWrite(stirPin[i], 255); delay(75); }
      analogWrite(stirPin[i], speed);
    }
  }
}

static void handle_pump() {
  char cmd = get_cmd_char("pump");
  if (cmd == 'r' || cmd == 'i') {
    // fields: cmd_char, p0, p1, ..., p5  where each is "t|period" or "t"
    char fields[NUM_PUMPS + 1][24];
    uint8_t n = parse_fields("pump", fields, NUM_PUMPS + 1);
    send_echo("pump", fields + 1, NUM_PUMPS);

    for (uint8_t i = 0; i < NUM_PUMPS && i + 1 < n; i++) {
      const char *f = fields[i + 1];
      if (strcmp(f, "--") == 0) continue;
      char *pipe = strchr(f, '|');
      unsigned long on_ms = 0, period_ms = 0;
      if (pipe) {
        on_ms     = (unsigned long)(atof(f) * 1000);
        period_ms = (unsigned long)(atol(pipe + 1) * 1000);
      } else {
        on_ms = (unsigned long)(atof(f) * 1000);
      }
      pump_set(i, on_ms, period_ms);
    }
  }
}

// ---- Arduino entry points -----------------------------------------------

void setup() {
  Serial.begin(9600);

  for (uint8_t i = 0; i < NUM_VIALS; i++) {
    allPIDs[i]->SetOutputLimits(0, 255);
    allPIDs[i]->SetMode(AUTOMATIC);
  }
  for (uint8_t i = 0; i < NUM_PUMPS; i++) pump_init(i);
}

void loop() {
  // Read serial into fixed buffer
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (input_len < INPUT_BUF_SIZE - 1) {
      input_buf[input_len++] = c;
    }
    if (c == '!') {
      input_buf[input_len] = '\0';
      string_complete = true;
      break;
    }
  }

  if (string_complete) {
    if (buf_starts("WHO_ARE_YOU")) {
      mev_send_hello(&Serial, ++mev_seq);
    } else if (buf_starts("PROVISION,")) {
      // Parse PROVISION,<device_id>,<owner_id>_!
      char cmd[INPUT_BUF_SIZE];
      strncpy(cmd, input_buf, INPUT_BUF_SIZE);
      char *p1 = strchr(cmd, ',');
      if (p1) {
        char *p2 = strchr(p1 + 1, ',');
        if (p2) {
          *p2 = '\0';
          char *owner = p2 + 1;
          // strip _!
          char *term = strstr(owner, "_!");
          if (term) *term = '\0';
          const char *dev_id = p1 + 1;

          DeviceIdentity existing;
          mev_load_identity(&existing);
          if (mev_identity_valid(&existing)) {
            mev_send_provision_err(&Serial, ++mev_seq, existing.device_id, "already_provisioned");
          } else if (strlen(dev_id) == 0 || strlen(owner) == 0 ||
                     strlen(dev_id) > 31 || strlen(owner) > 31) {
            mev_send_provision_err(&Serial, ++mev_seq, "BLANK", "bad_id_length");
          } else {
            mev_save_identity(dev_id, owner);
            mev_send_provision_ack(&Serial, ++mev_seq, dev_id, owner);
          }
        } else {
          mev_send_provision_err(&Serial, ++mev_seq, "BLANK", "bad_format");
        }
      } else {
        mev_send_provision_err(&Serial, ++mev_seq, "BLANK", "bad_format");
      }
    } else if (buf_starts("CLEAR_ID")) {
      mev_clear_identity();
      mev_send_clear_ack(&Serial, ++mev_seq);
    } else if (buf_starts("od_90")) {
      handle_od90();
    } else if (buf_starts("temp")) {
      handle_temp();
    } else if (buf_starts("stir")) {
      handle_stir();
    } else if (buf_starts("pump")) {
      handle_pump();
    }

    input_len = 0;
    string_complete = false;
  }

  // Sensor reads
  unsigned long total = 0;
  for (uint8_t i = 0; i < pd_avg; i++) total += analogRead(pd_pin[active_vial]);
  pd_output[active_vial] = total / pd_avg;
  active_vial = (active_vial + 1) % NUM_VIALS;

  unsigned long t[NUM_VIALS] = {0};
  for (uint8_t i = 0; i < 3; i++)
    for (uint8_t j = 0; j < NUM_VIALS; j++) t[j] += analogRead(temp_pin[j]);
  for (uint8_t i = 0; i < NUM_VIALS; i++) {
    tempInput[i] = t[i] / 3;
    allPIDs[i]->Compute();
    analogWrite(tempOutputPin[i], 255 - (int)tempOutput[i]);
  }

  for (uint8_t i = 0; i < NUM_PUMPS; i++) pump_update(i);
}
