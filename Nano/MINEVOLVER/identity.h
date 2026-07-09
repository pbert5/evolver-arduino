#pragma once
/*
 * miniEvolver device identity: persistent storage + WHO_ARE_YOU protocol.
 *
 * Primary target: SparkFun SAMD21 Mini (ATSAMD21G18) using FlashStorage_SAMD.
 * Fallback: RAM-only (identity lost on reboot) for non-SAMD targets.
 *
 * Protocol responses use the MEV frame format defined in CLAUDE.md.
 * CRC8: Dallas/Maxim 1-Wire poly=0x31, init=0xFF, applied over payload field.
 */

#include <Arduino.h>

#define MEV_MAGIC         0x4D455601u  /* 'M','E','V', struct_ver=1 */
#define MEV_PROTO_VER     2
#define MEV_FW_VER_MAJOR  0
#define MEV_FW_VER_MINOR  1
#define MEV_DEVICE_TYPE   "minievolver"
#define MEV_FRAME_MAX     200

struct DeviceIdentity {
  uint32_t magic;
  char device_id[32];   /* null-terminated, max 31 chars */
  char owner_id[32];    /* null-terminated, max 31 chars */
  uint8_t proto_ver;
  uint8_t fw_major;
  uint8_t fw_minor;
  uint8_t _pad;
};

/* ---- Storage backend ---- */

#if defined(ARDUINO_ARCH_SAMD)
#include <FlashStorage_SAMD.h>
FlashStorage(mev_flash, DeviceIdentity);

static void _mev_flash_read(DeviceIdentity *out) {
  *out = mev_flash.read();
}
static void _mev_flash_write(const DeviceIdentity *id) {
  mev_flash.write(*id);
}

#else
/* RAM fallback — loses identity on reboot; marks a known portability ceiling */
// ponytail: RAM-only identity for non-SAMD targets; add real flash backend when porting
static DeviceIdentity _mev_ram_store;
static bool _mev_ram_init = false;
static void _mev_flash_read(DeviceIdentity *out) {
  if (!_mev_ram_init) { memset(&_mev_ram_store, 0, sizeof(_mev_ram_store)); _mev_ram_init = true; }
  *out = _mev_ram_store;
}
static void _mev_flash_write(const DeviceIdentity *id) {
  _mev_ram_store = *id;
  _mev_ram_init = true;
}
#endif

/* ---- CRC8 (Dallas/Maxim 1-Wire, poly=0x31, init=0xFF) ---- */

static uint8_t mev_crc8(const char *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len && data[i]; i++) {
    crc ^= (uint8_t)data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
  }
  return crc;
}

/* ---- Public API ---- */

static bool mev_identity_valid(const DeviceIdentity *id) {
  return id->magic == MEV_MAGIC;
}

static void mev_load_identity(DeviceIdentity *out) {
  _mev_flash_read(out);
}

static bool mev_save_identity(const char *device_id, const char *owner_id) {
  if (!device_id || !device_id[0] || !owner_id || !owner_id[0]) return false;
  DeviceIdentity id;
  id.magic     = MEV_MAGIC;
  id.proto_ver = MEV_PROTO_VER;
  id.fw_major  = MEV_FW_VER_MAJOR;
  id.fw_minor  = MEV_FW_VER_MINOR;
  id._pad      = 0;
  strncpy(id.device_id, device_id, sizeof(id.device_id) - 1);
  id.device_id[sizeof(id.device_id) - 1] = '\0';
  strncpy(id.owner_id, owner_id, sizeof(id.owner_id) - 1);
  id.owner_id[sizeof(id.owner_id) - 1] = '\0';
  _mev_flash_write(&id);
  return true;
}

static void mev_clear_identity(void) {
  DeviceIdentity id;
  memset(&id, 0, sizeof(id));
  _mev_flash_write(&id);
}

/* ---- Frame builders (write directly to SerialUSB) ---- */

static void mev_send_hello(Stream *serial, uint32_t seq) {
  DeviceIdentity id;
  mev_load_identity(&id);
  const char *dev_id = (mev_identity_valid(&id) && id.device_id[0]) ? id.device_id : "BLANK";
  const char *owner  = (mev_identity_valid(&id) && id.owner_id[0])  ? id.owner_id  : "BLANK";

  char payload[96];
  snprintf(payload, sizeof(payload),
    "type=%s,proto=%d,fw=%d.%d,id=%s,owner=%s",
    MEV_DEVICE_TYPE, MEV_PROTO_VER, MEV_FW_VER_MAJOR, MEV_FW_VER_MINOR,
    dev_id, owner);

  uint8_t crc = mev_crc8(payload, sizeof(payload));
  char frame[MEV_FRAME_MAX];
  snprintf(frame, sizeof(frame), "MEV|%d|%s|%lu|HELLO|%s|%02X",
    MEV_PROTO_VER, dev_id, (unsigned long)seq, payload, crc);
  serial->println(frame);
}

static void mev_send_provision_ack(Stream *serial, uint32_t seq,
    const char *device_id, const char *owner_id) {
  char payload[80];
  snprintf(payload, sizeof(payload), "id=%s,owner=%s", device_id, owner_id);
  uint8_t crc = mev_crc8(payload, sizeof(payload));
  char frame[MEV_FRAME_MAX];
  snprintf(frame, sizeof(frame), "MEV|%d|%s|%lu|PROVISION_ACK|%s|%02X",
    MEV_PROTO_VER, device_id, (unsigned long)seq, payload, crc);
  serial->println(frame);
}

static void mev_send_provision_err(Stream *serial, uint32_t seq,
    const char *current_id, const char *reason) {
  char payload[64];
  snprintf(payload, sizeof(payload), "reason=%s", reason);
  uint8_t crc = mev_crc8(payload, sizeof(payload));
  char frame[MEV_FRAME_MAX];
  snprintf(frame, sizeof(frame), "MEV|%d|%s|%lu|PROVISION_ERR|%s|%02X",
    MEV_PROTO_VER, current_id, (unsigned long)seq, payload, crc);
  serial->println(frame);
}

static void mev_send_clear_ack(Stream *serial, uint32_t seq) {
  const char *payload = "ok=true";
  uint8_t crc = mev_crc8(payload, 7);
  char frame[MEV_FRAME_MAX];
  snprintf(frame, sizeof(frame), "MEV|%d|BLANK|%lu|CLEAR_ACK|%s|%02X",
    MEV_PROTO_VER, (unsigned long)seq, payload, crc);
  serial->println(frame);
}
