"""Static tests for the v2 bounded native temperature PID bridge."""
from pathlib import Path


SOURCE = (
    Path(__file__).parents[1] / "SAMD21" / "MINEVOLVER" / "MINEVOLVER.ino"
).read_text()


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
    assert 'input_string.startsWith("TEMP|2|")' in loop
    assert "temperatureCommandLogic();" in loop
    assert loop.index("temperatureCommandLogic();") < loop.index("pd.analyzeAndCheck")


def test_temperature_bridge_uses_fixed_bounds_and_native_pid_ceiling():
    assert "TEMP_PROTO_VER = 2" in SOURCE
    assert "TEMP_PID_OUTPUT_MAX = 64" in SOURCE
    assert "TEMP_RAW_SETPOINT_MAX = 65535UL" in SOURCE
    command = _function("temperatureCommandLogic")
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
