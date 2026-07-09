# Claude Working Agreement — evolver-arduino

## Scope

This repo contains Arduino firmware for the eVOLVER miniEvolver hardware.
Primary target: **SparkFun SAMD21 Mini** (ATSAMD21G18, 256 KB flash, 32 KB SRAM,
native USB, 48 MHz ARM Cortex-M0+, emulated EEPROM via flash).

Secondary target: ATtiny1634 (luminescence module, avr-gcc, separate subdirectory).

Parent workspace: `../evolver/` — the top-level flake pulls this repo in as an input.

## Required First Steps

1. Call `mcp__serena__initial_instructions` and follow the Serena manual.
2. Activate: `mcp__serena__activate_project` path `/home/ash/Documents/work/evolver-arduino`.
3. Use `mcp__mcp-nixos__nix` when changing `flake.nix`, board packages, library
   versions, or devShell packages.

## Serena Tool Usage

Use Serena for `.ino`, `.h`, `.cpp`, `.py` edits. Prefer `find_symbol` with
`include_body=True` → `replace_symbol_body` or `replace_content`. Do not use
`Edit` on files you haven't read via the Read tool.

## Firmware Constraints

Design for SAMD21-class capacity but keep the protocol portable to Nano-class boards:

- **Fixed-size buffers only.** No heap allocation in protocol code.
- **No large JSON parser on-device.** Config stays on the server.
- **Simple CRC8** (Dallas/Maxim polynomial 0x31) for frame validation.
- **FlashStorage_SAMD** library for persistent identity storage (emulated EEPROM).
- **Max frame size**: 200 bytes per message direction.
- **Identity struct**: device_id (31 chars + null), owner_id (31 chars + null),
  proto_version (uint8), fw_version (2× uint8).

## miniEvolver Identity / Provisioning Protocol

### Serial framing
- Existing commands: terminated with `_!` (evolver_si convention)
- Provisioning commands: also `_!` terminated
- Device responses: terminated with `\n`

### Handshake
```
server → device:  WHO_ARE_YOU_!
device → server:  MEV|<proto>|<id_or_BLANK>|<seq>|HELLO|type=minievolver,proto=<p>,fw=<v>,id=<id_or_BLANK>,owner=<owner_or_BLANK>|<CRC8_HEX>\n
```

### Provisioning
```
server → device:  PROVISION,<device_id>,<owner_id>_!
device → server:  MEV|<proto>|<id>|<seq>|PROVISION_ACK|id=<id>,owner=<owner>|<CRC8_HEX>\n
                  or PROVISION_ERR|reason=already_provisioned|...
```

### Identity clear (for testing / reprovisioning)
```
server → device:  CLEAR_ID_!
device → server:  MEV|<proto>|BLANK|<seq>|CLEAR_ACK|ok=true|<CRC8_HEX>\n
```

### CRC8 algorithm
Dallas/Maxim 1-Wire: poly=0x31, init=0xFF. Applied over the payload field only.
Python mirror: `tests/test_protocol.py::crc8()`.

## Board Abstraction

`identity.h` defines `#if defined(ARDUINO_ARCH_SAMD)` / fallback guards so the
identity code compiles on both SAMD21 (FlashStorage) and non-SAMD targets
(RAM-only, non-persistent). Always test both paths when changing identity.h.

## Arduino Library Dependencies

Installed via `nix run .#setup-arduino` (uses arduino-cli):
- `sparkfun:samd` board core
- `FlashStorage_SAMD` (persistent identity storage)
- `PID` (temperature PID)
- `SimpleTimer` (utility timers)
- `evolver_si` (local, in `libraries/evolver_si/`)

## Test Strategy

Protocol tests (`tests/test_protocol.py`) run in pure Python — no hardware needed.
Run via `nix flake check` or `pytest tests/`.

Hardware tests require a physical device and MUST be run manually with an explicit
`--hardware` flag. Never run automatically in CI.

## Commits

Format: `<scope>: <what> — <why>` e.g. `arduino: add WHO_ARE_YOU handler`
Commit firmware and test changes together. Push before updating the parent evolver/ lock.
After any `flake.nix` change, verify `nix flake check` passes.
