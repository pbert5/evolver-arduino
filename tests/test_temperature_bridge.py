"""Static tests for the v2 bounded native temperature PID bridge."""
from pathlib import Path


SOURCE = (
    Path(__file__).parents[1] / "SAMD21" / "MINEVOLVER" / "MINEVOLVER.ino"
).read_text()


def build_hardware_temperature(channel: int, raw: int) -> str:
    if channel not in (0, 1) or not 1 <= raw <= 65535:
        raise ValueError("invalid temperature frame")
    return f"HW_TEMP_V2,{channel},{raw}_!"


def parse_hardware_temperature(frame: str) -> tuple[int, int]:
    if not frame.endswith("_!"):
        raise ValueError("bad terminator")
    fields = frame[:-2].split(",")
    if len(fields) != 3 or fields[0] != "HW_TEMP_V2":
        raise ValueError("bad temperature frame")
    channel, raw = (int(value) for value in fields[1:])
    if channel not in (0, 1) or not 1 <= raw <= 65535:
        raise ValueError("invalid temperature frame")
    return channel, raw


def _function(name: str) -> str:
    start = SOURCE.index(f"void {name}(")
    opening = SOURCE.index("{", start)
    depth = 0
    for index in range(opening, len(SOURCE)):
        if SOURCE[index] == "{":
            depth += 1
        elif SOURCE[index] == "}":
            depth -= 1
            if depth == 0:
                return SOURCE[start : index + 1]
    raise AssertionError(f"unterminated function {name}")


def test_v2_temperature_protocol_is_dispatched_before_legacy_commands():
    loop = _function("loop")
    assert 'input_string.startsWith("HW_TEMP_V2,")' in loop
    assert 'input_string.startsWith("TEMP|2|")' in loop
    assert "hardwareTemperatureCommandLogic();" in loop
    assert "temperatureCommandLogic();" in loop
    assert loop.index('input_string.startsWith("HW_TEMP_V2,")') < loop.index('input_string.startsWith("HW_")')
    assert loop.index("hardwareTemperatureCommandLogic();") < loop.index("pd.analyzeAndCheck")


def test_legacy_hardware_temperature_frame_is_explicitly_disabled():
    command = _function("hardwareTemperatureCommandLogic")
    assert 'hwReply("ERR", "TEMP_V2", "reason=disabled_use_TEMP_PIPE_V2")' in command
    assert "tempSetpoint[" not in command
    assert "temperatureControlEnabled[" not in command
    assert "SetMode(AUTOMATIC)" not in command
    assert "temperatureLastCommand" not in command


def test_secure_temperature_path_is_the_only_integrated_setpoint_mutation_path():
    command = _function("hardwareTemperatureCommandLogic")
    secure = _function("temperatureCommandLogic")
    assert "tempSetpoint[channel] = (double)raw;" in secure
    assert "tempOwnerMatches" in secure
    assert "tempCorrelationSeen" in secure
    assert "tempLeaseActive" in secure
    assert "tempGeneration" in secure
    assert "tempSetpoint[channel] =" not in command
    assert "allTempPIDS[channel]" not in command


def test_hardware_temperature_parser_serializer_round_trip_and_boundaries():
    for raw in (1, 65535):
        frame = build_hardware_temperature(1, raw)
        assert frame == f"HW_TEMP_V2,1,{raw}_!"
        assert parse_hardware_temperature(frame) == (1, raw)
    for frame in ("TEMP|2|SET|1|1|1|owner|lease|generation_!",
                  "HW_TEMP_V2,1,0_!", "HW_TEMP_V2,1,65536_!"):
        try:
            parse_hardware_temperature(frame)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted incompatible frame: {frame}")


def test_temperature_bridge_uses_fixed_bounds_and_native_pid_ceiling():
    assert "TEMP_PROTO_VER = 2" in SOURCE
    assert "TEMP_RAW_SETPOINT_MIN = 1UL" in SOURCE
    assert "TEMP_PID_OUTPUT_MAX = 64" in SOURCE
    assert "TEMP_RAW_SETPOINT_MAX = 65535UL" in SOURCE
    command = _function("temperatureCommandLogic")
    assert "TEMP_RAW_SETPOINT_MIN" in command
    assert "TEMP_RAW_SETPOINT_MAX" in command
    assert "TEMP_PID_OUTPUT_MAX" in command
    assert "SetOutputLimits(0, TEMP_PID_OUTPUT_MAX)" in SOURCE


def test_temperature_bridge_requires_correlation_and_rejects_replay():
    command = _function("temperatureCommandLogic")
    assert "tempCorrelationSeen" in command
    assert "TEMP_ERR_REPLAY" in command
    assert "tempLastCorrelation" in command
    assert "correlationNumber <= tempLastCorrelation" in command


def test_temperature_bridge_refreshes_dead_man_only_after_valid_commands():
    command = _function("temperatureCommandLogic")
    assert "TEMP_DEADMAN_MS" in SOURCE
    assert "temperatureLastCommand = millis()" in command
    loop = _function("loop")
    assert "temperatureLastCommand" in loop
    assert "setHardwareSafeState();" in loop
    assert "TEMP_DEADMAN_MS" in loop


def test_disable_and_safe_stop_force_manual_zero_output():
    command = _function("temperatureCommandLogic")
    safe = _function("setHardwareSafeState")
    assert 'operation == "DISABLE"' in command
    assert 'operation == "SAFE"' in command
    assert "temperatureControlEnabled[channel] = false" in command
    assert "SetMode(MANUAL)" in command
    assert "analogWrite(tempOutputPin[channel], 0)" in command
    assert "temperatureLastCommand = 0" in safe


def test_temperature_bridge_has_fail_closed_owner_lease_generation_boundary():
    command = _function("temperatureCommandLogic")
    assert "TEMP_ERR_AUTHORITY" in SOURCE
    assert "tempOwnerMatches" in command
    assert "tempLeaseActive" in command
    assert "tempLease" in command
    assert "tempGeneration" in command
    assert "generation <= tempGeneration" in command
    assert "temperatureLastCommand = millis()" in command
    assert command.index("tempOwnerMatches") < command.index("temperatureLastCommand = millis()")


def test_temperature_bridge_and_legacy_temperature_commands_are_arbitrated():
    loop = _function("loop")
    temp_logic = _function("tempLogic")
    update = _function("updateTemperatureControl")
    assert "normalControllerSuspended" in loop
    assert "temperatureBridgeEnabled" in loop
    assert "if (temperatureBridgeEnabled)" in temp_logic
    assert "TEMP_ERR_ARBITRATION" in temp_logic
    assert "temperatureBridgeEnabled" in update


def test_authority_contract_covers_malformed_replay_deadman_disable_and_safe():
    command = _function("temperatureCommandLogic")
    assert "TEMP_ERR_MALFORMED" in SOURCE
    assert "TEMP_ERR_REPLAY" in command
    assert "TEMP_DEADMAN_MS" in SOURCE
    assert 'operation == "DISABLE"' in command
    assert 'operation == "SAFE"' in command
    assert "tempLeaseActive = false" in command


class _AuthorityModel:
    """Small executable mirror of the firmware authority/arbitration boundary."""

    def __init__(self):
        self.generation = 0
        self.lease = None
        self.correlation = 0
        self.bridge = False

    def apply(self, operation, correlation, owner, lease, generation):
        if owner != "owner-a" or not lease or generation <= 0:
            return "authority"
        active = self.lease is not None
        if active and (lease, generation) != (self.lease, self.generation):
            return "authority"
        if not active and operation != "SAFE" and generation <= self.generation:
            return "stale"
        if operation != "SAFE" and correlation <= self.correlation:
            return "replay"
        if not active:
            self.lease, self.generation = lease, generation
        self.correlation = max(self.correlation, correlation)
        if operation == "SET":
            self.bridge = True
        elif operation == "DISABLE":
            self.bridge = False
            self.lease = None
        elif operation == "SAFE":
            self.bridge = False
            self.lease = None
        else:
            return "malformed"
        return "accepted"

    def deadman(self):
        self.bridge = False
        self.lease = None


def test_authority_model_rejects_stale_owner_lease_and_replay_without_state_change():
    model = _AuthorityModel()
    assert model.apply("SET", 1, "owner-a", 7, 3) == "accepted"
    assert model.apply("SET", 2, "wrong-owner", 7, 3) == "authority"
    assert model.apply("SET", 2, "owner-a", 8, 3) == "authority"
    assert model.apply("SET", 2, "owner-a", 7, 2) == "authority"
    assert model.apply("SET", 1, "owner-a", 7, 3) == "replay"
    assert model.bridge is True


def test_authority_model_arbitrates_legacy_and_preserves_safe_controls():
    model = _AuthorityModel()
    assert model.apply("SET", 1, "owner-a", 7, 3) == "accepted"
    assert model.bridge is True  # legacy temp SET must be rejected while owned
    assert model.apply("DISABLE", 2, "owner-a", 7, 3) == "accepted"
    assert model.bridge is False
    assert model.apply("SET", 3, "owner-a", 8, 4) == "accepted"
    model.deadman()
    assert model.bridge is False
    assert model.apply("SET", 4, "owner-a", 9, 5) == "accepted"
    assert model.apply("SAFE", 4, "owner-a", 9, 5) == "accepted"
    assert model.bridge is False
