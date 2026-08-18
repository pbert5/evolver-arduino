"""Static safety contracts for the min-eVOLVER startup and control paths."""
from pathlib import Path


SOURCE = (
    Path(__file__).parents[1] / "SAMD21" / "MINEVOLVER" / "MINEVOLVER.ino"
).read_text()


def _function(name: str) -> str:
    start = SOURCE.index(f"void {name}(")
    depth = 0
    for index in range(SOURCE.index("{", start), len(SOURCE)):
        if SOURCE[index] == "{":
            depth += 1
        elif SOURCE[index] == "}":
            depth -= 1
            if depth == 0:
                return SOURCE[start:index + 1]
    raise AssertionError(f"unterminated function {name}")


def test_boot_forces_every_output_safe_before_usb_wait():
    setup = _function("setup")
    safe = _function("initializeHardwareSafeState")
    assert "initializeHardwareSafeState();" in setup
    assert setup.index("initializeHardwareSafeState();") < setup.index("while(!SerialUSB)")
    assert "analogWrite(tempOutputPin[i], 255)" in safe
    assert "analogWrite(4 + i, 0)" in safe
    assert "pwm.analogWrite(stirOutputPin[i], 0)" in safe
    assert "digitalWrite(12, LOW)" in safe
    assert "analogWrite(pumpOutputPin[i], 0)" in safe


def test_boot_and_safe_state_leave_temperature_control_manual_and_off():
    setup = _function("setup")
    safe = _function("setHardwareSafeState")
    assert "allTempPIDS[i]->SetMode(MANUAL);" in setup
    assert "temperatureControlEnabled[i] = false;" in safe
    assert "analogWrite(tempOutputPin[i], 255)" in safe
    assert "allTempPIDS[i]->SetMode(MANUAL);" in safe


def test_only_applied_normal_temp_target_enables_pid():
    temp_logic = _function("tempLogic")
    exit_test = _function("exitHardwareTestMode")
    update = _function("updateTemperatureControl")
    assert "temperatureControlEnabled[i] = temp_saved_inputs[i] > 0;" in temp_logic
    assert "temperatureControlEnabled[i] ? AUTOMATIC : MANUAL" in temp_logic
    assert "SetMode(AUTOMATIC)" not in exit_test
    assert "if (!temperatureControlEnabled[i])" in update
    assert "analogWrite(tempOutputPin[i], 255)" in update


def test_status_reports_idle_temperature_control_state():
    status = _function("hardwareCommandLogic")
    assert "temp_control=" in status
    assert "mode=" in status
