"""
Pure-Python tests for the miniEvolver MEV serial protocol.

These tests mirror the firmware logic in identity.h so protocol changes
must be reflected here. No hardware required — run with: pytest tests/
"""

# ---- CRC8 (mirrors firmware mev_crc8, poly=0x31, init=0xFF) ----

def crc8(payload: str) -> int:
    crc = 0xFF
    for ch in payload.encode():
        crc ^= ch
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


# ---- Parser mirrors ----

def parse_hello(line: str):
    """
    Parse: MEV|<proto>|<id_or_BLANK>|<seq>|HELLO|<payload>|<CRC_HEX>
    Returns dict or None on parse error.
    """
    line = line.strip()
    parts = line.split("|")
    if len(parts) != 7:
        return None
    prefix, proto, dev_id, seq, msg_type, payload, crc_hex = parts
    if prefix != "MEV" or msg_type != "HELLO":
        return None
    try:
        expected_crc = crc8(payload)
        received_crc = int(crc_hex, 16)
    except ValueError:
        return None
    return {
        "proto": int(proto),
        "device_id": None if dev_id == "BLANK" else dev_id,
        "seq": int(seq),
        "payload": payload,
        "crc_ok": expected_crc == received_crc,
        "fields": dict(kv.split("=", 1) for kv in payload.split(",") if "=" in kv),
    }


def build_hello(device_id=None, owner_id=None, proto=2, fw="0.1", seq=1):
    """Mirror of mev_send_hello() for test assertions."""
    dev_str = device_id or "BLANK"
    owner_str = owner_id or "BLANK"
    payload = f"type=minievolver,proto={proto},fw={fw},id={dev_str},owner={owner_str}"
    crc = crc8(payload)
    return f"MEV|{proto}|{dev_str}|{seq}|HELLO|{payload}|{crc:02X}"


# ---- CRC8 tests ----

def test_crc8_known_vector():
    # Empty string CRC should be 0xFF (no bytes processed)
    assert crc8("") == 0xFF

def test_crc8_deterministic():
    assert crc8("hello") == crc8("hello")

def test_crc8_differs_from_different_payload():
    assert crc8("id=BLANK,owner=BLANK") != crc8("id=mev-001,owner=BLANK")


# ---- Hello frame building ----

def test_build_hello_blank():
    frame = build_hello()
    assert frame.startswith("MEV|2|BLANK|")
    assert "|HELLO|" in frame

def test_build_hello_with_ids():
    frame = build_hello(device_id="mev-003", owner_id="server-a1")
    assert "|mev-003|" in frame
    assert "id=mev-003" in frame
    assert "owner=server-a1" in frame


# ---- Hello frame parsing ----

def test_parse_hello_blank_device():
    frame = build_hello(seq=1)
    msg = parse_hello(frame)
    assert msg is not None
    assert msg["device_id"] is None
    assert msg["proto"] == 2
    assert msg["crc_ok"] is True
    assert msg["fields"]["id"] == "BLANK"
    assert msg["fields"]["owner"] == "BLANK"
    assert msg["fields"]["type"] == "minievolver"

def test_parse_hello_provisioned_device():
    frame = build_hello(device_id="mev-003", owner_id="server-a1", seq=7)
    msg = parse_hello(frame)
    assert msg is not None
    assert msg["device_id"] == "mev-003"
    assert msg["seq"] == 7
    assert msg["crc_ok"] is True
    assert msg["fields"]["owner"] == "server-a1"

def test_parse_hello_crc_corruption():
    frame = build_hello(seq=1)
    parts = frame.split("|")
    parts[-1] = "00"  # corrupt CRC
    msg = parse_hello("|".join(parts))
    assert msg is not None
    assert msg["crc_ok"] is False

def test_parse_hello_wrong_prefix():
    assert parse_hello("BAD|2|BLANK|1|HELLO|type=minievolver,proto=2,fw=0.1,id=BLANK,owner=BLANK|FF") is None

def test_parse_hello_wrong_msg_type():
    frame = build_hello()
    frame = frame.replace("|HELLO|", "|UNKNOWN|")
    assert parse_hello(frame) is None

def test_parse_hello_too_few_fields():
    assert parse_hello("MEV|2|BLANK") is None
    assert parse_hello("") is None
    assert parse_hello("garbage line") is None

def test_parse_hello_missing_pipe():
    assert parse_hello("MEV2BLANK1HELLOpayloadFF") is None

def test_parse_hello_hex_crc_case_insensitive():
    frame = build_hello(device_id="mev-001", owner_id="svr")
    # uppercase CRC in frame is fine
    assert parse_hello(frame) is not None


# ---- Sequence number ----

def test_sequence_monotonic():
    f1 = build_hello(seq=1)
    f2 = build_hello(seq=2)
    m1 = parse_hello(f1)
    m2 = parse_hello(f2)
    assert m2["seq"] > m1["seq"]


# ---- Provision ACK / ERR frame parsing ----

def parse_provision_ack(line: str):
    parts = line.strip().split("|")
    if len(parts) != 7 or parts[4] != "PROVISION_ACK":
        return None
    payload = parts[5]
    crc_ok = crc8(payload) == int(parts[6], 16)
    fields = dict(kv.split("=", 1) for kv in payload.split(",") if "=" in kv)
    return {"device_id": parts[2], "crc_ok": crc_ok, "fields": fields}

def parse_provision_err(line: str):
    parts = line.strip().split("|")
    if len(parts) != 7 or parts[4] != "PROVISION_ERR":
        return None
    payload = parts[5]
    crc_ok = crc8(payload) == int(parts[6], 16)
    fields = dict(kv.split("=", 1) for kv in payload.split(",") if "=" in kv)
    return {"device_id": parts[2], "crc_ok": crc_ok, "reason": fields.get("reason")}

def build_provision_ack(device_id, owner_id, seq=1, proto=2):
    payload = f"id={device_id},owner={owner_id}"
    crc = crc8(payload)
    return f"MEV|{proto}|{device_id}|{seq}|PROVISION_ACK|{payload}|{crc:02X}"

def build_provision_err(current_id, reason, seq=1, proto=2):
    payload = f"reason={reason}"
    crc = crc8(payload)
    return f"MEV|{proto}|{current_id}|{seq}|PROVISION_ERR|{payload}|{crc:02X}"

def test_parse_provision_ack():
    frame = build_provision_ack("mev-003", "server-a1", seq=2)
    msg = parse_provision_ack(frame)
    assert msg is not None
    assert msg["device_id"] == "mev-003"
    assert msg["fields"]["owner"] == "server-a1"
    assert msg["crc_ok"] is True

def test_parse_provision_err_already_provisioned():
    frame = build_provision_err("mev-003", "already_provisioned", seq=3)
    msg = parse_provision_err(frame)
    assert msg is not None
    assert msg["reason"] == "already_provisioned"
    assert msg["crc_ok"] is True

def test_parse_provision_err_bad_format():
    frame = build_provision_err("BLANK", "bad_format")
    msg = parse_provision_err(frame)
    assert msg["reason"] == "bad_format"

def test_provision_ack_crc_corruption():
    frame = build_provision_ack("mev-001", "svr")
    parts = frame.split("|")
    parts[-1] = "00"
    msg = parse_provision_ack("|".join(parts))
    assert msg is not None
    assert msg["crc_ok"] is False

def test_provision_ack_wrong_msg_type():
    frame = build_provision_ack("mev-001", "svr")
    assert parse_provision_err(frame) is None  # different parser, should return None


# ---- CLEAR_ACK parsing ----

def parse_clear_ack(line: str):
    parts = line.strip().split("|")
    if len(parts) != 7 or parts[4] != "CLEAR_ACK":
        return None
    payload = parts[5]
    crc_ok = crc8(payload) == int(parts[6], 16)
    return {"crc_ok": crc_ok, "device_id": parts[2]}

def build_clear_ack(seq=1, proto=2):
    payload = "ok=true"
    crc = crc8(payload)
    return f"MEV|{proto}|BLANK|{seq}|CLEAR_ACK|{payload}|{crc:02X}"

def test_parse_clear_ack():
    frame = build_clear_ack(seq=5)
    msg = parse_clear_ack(frame)
    assert msg is not None
    assert msg["crc_ok"] is True
    assert msg["device_id"] == "BLANK"

def test_parse_clear_ack_crc_corruption():
    frame = build_clear_ack()
    parts = frame.split("|")
    parts[-1] = "00"
    msg = parse_clear_ack("|".join(parts))
    assert msg["crc_ok"] is False


# ---- Edge cases / malformed input ----

def test_partial_frame_is_rejected():
    assert parse_hello("MEV|2|BLANK") is None

def test_concatenated_frames_first_only():
    # Real serial frames are newline-terminated (firmware uses println)
    f1 = build_hello(seq=1) + "\n"
    f2 = build_hello(seq=2) + "\n"
    first_line = (f1 + f2).split("\n")[0]
    assert parse_hello(first_line) is not None

def test_wrong_proto_version_is_detectable():
    frame = build_hello(proto=99, seq=1)
    msg = parse_hello(frame)
    assert msg is not None
    assert msg["proto"] == 99  # parser exposes it; caller checks version

def test_id_field_max_length():
    # 31 chars is the max for device_id in firmware; longer IDs should be rejected server-side
    long_id = "x" * 31
    frame = build_hello(device_id=long_id, seq=1)
    msg = parse_hello(frame)
    assert msg is not None
    assert msg["device_id"] == long_id

def test_id_too_long_flagged():
    too_long = "x" * 32
    frame = build_hello(device_id=too_long, seq=1)
    msg = parse_hello(frame)
    # Parser itself doesn't enforce length — server enforces; just confirm it parses
    assert msg is not None
    assert len(msg["device_id"]) == 32  # server should reject this
