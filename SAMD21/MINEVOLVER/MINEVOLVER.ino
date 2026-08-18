#include <evolver_si.h>
#include <PID_v1.h>
#include <SAMD21turboPWM.h>
#include "identity.h"

// String Input
String input_string = "";
boolean string_complete = false;
uint32_t mev_seq = 0;

int num_vials = 2;
int active_vial = 0;
int pd_times_averaged = 20;
int pd_output[] = {60000, 60000};

int pd_pin[] = {A2, A3};
int temp_pin[] = {A0, A1};

// General Serial Communication
String comma = ",";
String end_mark = "end";  

// Photodiode Serial Communication
int expectedPDinputs = 2;
String photodiode_address = "od_90";
evolver_si pd("od_90", "_!", expectedPDinputs);
boolean new_PDinput = false;
int saved_PD_averaged = 1000;

// LED Settings
String led_address = "od_led";
evolver_si led("od_led", "_!", num_vials + 1);
boolean new_LEDinput = false;
int saved_LEDinputs[] = {0, 0};

// TEMP
double tempSetpoint[] = {0, 0};
double tempOutput[2], tempInput[2];
int tempOutputPin[] = {2, 3};
String temp_address = "temp";
boolean temp_new_input = false;
evolver_si temp("temp", "_!", num_vials + 1);
int temp_saved_inputs[] = {0, 0};
bool temperatureControlEnabled[] = {false, false};

int Kp = 6500;
int Ki = 20;
int Kd = 2;

PID pid1(&tempInput[0], &tempOutput[0], &tempSetpoint[0], Kp, Ki, Kd, DIRECT);
PID pid2(&tempInput[1], &tempOutput[1], &tempSetpoint[1], Kp, Ki, Kd, DIRECT);
PID *allTempPIDS[16] = {&pid1, &pid2};

// Stir
TurboPWM pwm;
String stir_address = "stir";
evolver_si stir("stir", "_!", num_vials + 1);
boolean stir_new_input = false;
int stir_saved_inputs[] = {0, 0};
int stirInput[] = {1000, 1000};
int stirOutputPin[] = {11, 13};

// Pumps
String pumpAddress = "pump";
const int numPumps = 6;
double timeToPump;
int pumpInterval;
int speedset[2] {0, 255};
evolver_si pump("pump", "_!", numPumps+1);
String pumpSavedInputs[6];
boolean pump_new_input = false;
int pumpOutputPin[] = {6, 7, 8, 9, 10, 12};
byte ipp = 0;

// Commissioning/debug protocol.  It is deliberately separate from evolver_si
// commands: only complete HW_*_! messages reach this code.
const unsigned long HW_IDLE_TIMEOUT_MS = 15000;
const unsigned long HW_MAX_PUMP_MS = 1000;
const unsigned long HW_MAX_STIR_MS = 1000;
const unsigned long HW_MAX_HEATER_MS = 250;
const int HW_MAX_LED_LEVEL = 255;
const int HW_MAX_STIR_LEVEL = 250;
const int HW_MAX_HEATER_LEVEL = 64;
bool hardwareTestMode = false;
bool normalControllerSuspended = false;
unsigned long hardwareLastCommand = 0;

struct HardwarePulse {
  bool active;
  int channel;
  unsigned long endsAt;
};
HardwarePulse pumpPulse = {false, -1, 0};
HardwarePulse stirPulse = {false, -1, 0};
HardwarePulse heaterPulse = {false, -1, 0};

class Pump {
  boolean pumpRunning = false;
  int addr; int timeToPump = 0;
  int pumpInterval = 0;
  unsigned long previousMillis = 0;
  boolean off = false;

  public:
    Pump() {}
    void init(int addrInit) {
      addr = addrInit;
      turnOff();
    }

    void update() {  
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis > timeToPump && pumpRunning) {
        if (pumpOutputPin[addr] != 12) {
          analogWrite(pumpOutputPin[addr], speedset[0]);
        }
        else {
          digitalWrite(12, LOW);  
        }
        pumpRunning = false; 
      }

      if (currentMillis - previousMillis > pumpInterval && !pumpRunning && pumpInterval != 0) {
        previousMillis = currentMillis;
        pumpRunning = true;
        if (pumpOutputPin[addr] != 12) {
          analogWrite(pumpOutputPin[addr], speedset[1]);
        }
        else {
          digitalWrite(12, HIGH);  
        }
      }
    }

    void setPump(float timeToPumpSet, int pumpIntervalSet) {
      timeToPump = timeToPumpSet * 1000;
      pumpInterval = pumpIntervalSet * 1000;

      previousMillis = millis();
      pumpRunning = true;
      if (timeToPump > 0) {
        if (pumpOutputPin[addr] != 12) {
          analogWrite(pumpOutputPin[addr], speedset[1]);
        }
        else {
          digitalWrite(12, HIGH);  
        }
      }
    }

    void turnOff() {
      if (pumpOutputPin[addr] != 12) {
        analogWrite(pumpOutputPin[addr], speedset[0]);
      }
      else {
        digitalWrite(12, LOW);  
      }
      timeToPump = 0;
      pumpInterval = 0;
      pumpRunning = false;
      previousMillis = millis();
    }

    bool isNewChemostat(float newTimeToPump, int newPumpInterval) {
      newTimeToPump = newTimeToPump * 1000;
      newPumpInterval = newPumpInterval * 1000;

      return (newTimeToPump == timeToPump && newPumpInterval == pumpInterval);
    }

    bool isRunning() {
      return pumpRunning;
    }
};

Pump pumps[numPumps];

// Setup-only safe initialization. It intentionally avoids runtime PID and Pump
// methods, so it remains safe before their state has been initialized.
void initializeHardwareSafeState() {
  hardwareTestMode = false;
  normalControllerSuspended = true;
  pumpPulse.active = stirPulse.active = heaterPulse.active = false;
  for (int i = 0; i < numPumps; i++) {
    pinMode(pumpOutputPin[i], OUTPUT);
    if (pumpOutputPin[i] == 12) digitalWrite(12, LOW);
    else analogWrite(pumpOutputPin[i], 0);
    pumpSavedInputs[i] = "--";
  }
  for (int i = 0; i < num_vials; i++) {
    // Set the output latch low before enabling the N-MOSFET gate pin, then
    // explicitly drive zero-duty PWM.  LOW is the heater-safe state.
    digitalWrite(tempOutputPin[i], LOW);
    pinMode(tempOutputPin[i], OUTPUT);
    pinMode(4 + i, OUTPUT);
    analogWrite(tempOutputPin[i], 0);
    analogWrite(4 + i, 0);
    pwm.analogWrite(stirOutputPin[i], 0);
    tempSetpoint[i] = 0;
    temp_saved_inputs[i] = 0;
    tempOutput[i] = 0;
    saved_LEDinputs[i] = 0;
    stir_saved_inputs[i] = 0;
    temperatureControlEnabled[i] = false;
  }
}

void setup() {
  analogReadResolution(16);
  // Configure PWM before writing its physical outputs, then force every
  // actuator OFF before any serial wait or normal control path can run.
  pwm.setClockDivider(255, false);
  pwm.timer(2,256, 35000, true);
  initializeHardwareSafeState();

  for (int i = 0; i < numPumps; i++) {
    pumps[i].init(i);
  }
  for (int i = 0; i < num_vials; i++) {
    allTempPIDS[i]->SetOutputLimits(0,255);
    allTempPIDS[i]->SetMode(MANUAL);
  }

  SerialUSB.begin(9600);
  input_string.reserve(1000);
  // The board may wait indefinitely for USB, but all outputs are already safe.
  while(!SerialUSB);
    
}

void loop() {
  if (!hardwareTestMode) {
    readTempSensors();
    updateTemperatureControl();
  }
  readPD();
  serialEvent();
  if (string_complete) {
    // Provisioning commands are checked before evolver_si parsing so they
    // never collide with sensor addresses.
    if (input_string.startsWith("WHO_ARE_YOU")) {
      mev_send_hello(&SerialUSB, ++mev_seq);
      string_complete = false;
      input_string = "";
      return;
    }
    if (input_string.startsWith("PROVISION,")) {
      provisioningLogic();
      string_complete = false;
      input_string = "";
      return;
    }
    if (input_string.startsWith("CLEAR_ID")) {
      mev_clear_identity();
      mev_send_clear_ack(&SerialUSB, ++mev_seq);
      string_complete = false;
      input_string = "";
      return;
    }
    if (input_string.startsWith("HW_")) {
      hardwareCommandLogic();
      string_complete = false;
      input_string = "";
      return;
    }

    // A normal experiment command explicitly returns control to the normal
    // controller.  It never resumes a pending commissioning pulse.
    if (hardwareTestMode || normalControllerSuspended) exitHardwareTestMode();

    pd.analyzeAndCheck(input_string);
    led.analyzeAndCheck(input_string);
    stir.analyzeAndCheck(input_string);
    temp.analyzeAndCheck(input_string);
    pump.analyzeAndCheck(input_string);

    // Clear input string, avoid accumulation of previous message
    input_string = "";

    if (pd.addressFound) {
      photodiodeLogic();
    }

    if (led.addressFound) {
      ledLogic();
    }

    if (temp.addressFound) {
      tempLogic();
    }

    if (stir.addressFound) {
      stirLogic();
    }

    if (pump.addressFound) {
      pumpLogic();  
    }
  }

  // Must be executed every loop
  string_complete = false;
  input_string = "";
  for (int i = 0; i < numPumps; i++) {
    if (!hardwareTestMode) pumps[i].update();
  }
  updateHardwarePulses();
  if (hardwareTestMode && millis() - hardwareLastCommand > HW_IDLE_TIMEOUT_MS) {
    setHardwareSafeState();
    hardwareTestMode = false;
  }

  if (input_string.length() > 20000) {
    input_string = "";  
  }
}

void hwReply(const char *status, const char *operation, String fields) {
  SerialUSB.print("HW|");
  SerialUSB.print(MEV_HW_PROTO_VER);
  SerialUSB.print("|");
  SerialUSB.print(status);
  SerialUSB.print("|");
  SerialUSB.print(operation);
  if (fields.length()) { SerialUSB.print("|"); SerialUSB.print(fields); }
  SerialUSB.println();
}

bool hwChannel(const String &value, int count, int *channel) {
  if (!value.length()) return false;
  for (unsigned int i = 0; i < value.length(); i++) if (!isDigit(value[i])) return false;
  *channel = value.toInt();
  return *channel >= 0 && *channel < count;
}

bool hwRange(const String &value, int maximum, int *number) {
  if (!value.length()) return false;
  for (unsigned int i = 0; i < value.length(); i++) if (!isDigit(value[i])) return false;
  *number = value.toInt();
  return *number >= 0 && *number <= maximum;
}

void pumpWrite(int channel, bool on) {
  if (pumpOutputPin[channel] == 12) digitalWrite(12, on ? HIGH : LOW);
  else analogWrite(pumpOutputPin[channel], on ? 255 : 0);
}

// This is the sole shutdown path for commissioning outputs.  It also cancels
// normal pump schedules, so an old chemostat interval cannot restart a pump.
void setHardwareSafeState() {
  normalControllerSuspended = true;
  pumpPulse.active = stirPulse.active = heaterPulse.active = false;
  for (int i = 0; i < numPumps; i++) pumps[i].turnOff();
  for (int i = 0; i < num_vials; i++) {
    pwm.analogWrite(stirOutputPin[i], 0);
    analogWrite(tempOutputPin[i], 0);
    analogWrite(4 + i, 0);
    tempOutput[i] = 0;
    temperatureControlEnabled[i] = false;
    allTempPIDS[i]->SetMode(MANUAL);
  }
}

void enterHardwareTestMode() {
  if (!hardwareTestMode) setHardwareSafeState();
  hardwareTestMode = true;
  hardwareLastCommand = millis();
}

void exitHardwareTestMode() {
  setHardwareSafeState();
  hardwareTestMode = false;
  normalControllerSuspended = false;
}

void updateHardwarePulses() {
  unsigned long now = millis();
  if (pumpPulse.active && (long)(now - pumpPulse.endsAt) >= 0) {
    pumpWrite(pumpPulse.channel, false); pumpPulse.active = false;
  }
  if (stirPulse.active && (long)(now - stirPulse.endsAt) >= 0) {
    pwm.analogWrite(stirOutputPin[stirPulse.channel], 0); stirPulse.active = false;
  }
  if (heaterPulse.active && (long)(now - heaterPulse.endsAt) >= 0) {
    analogWrite(tempOutputPin[heaterPulse.channel], 0); heaterPulse.active = false;
  }
}

void hardwareCommandLogic() {
  String cmd = input_string;
  cmd.replace("\r", ""); cmd.replace("\n", ""); cmd.trim();
  if (!cmd.endsWith("_!")) { hwReply("ERR", "UNKNOWN", "reason=bad_terminator"); return; }
  cmd.remove(cmd.length() - 2);
  int commas[4] = {-1, -1, -1, -1}; int found = 0;
  for (unsigned int i = 0; i < cmd.length() && found < 4; i++) if (cmd[i] == ',') commas[found++] = i;
  String operation = found ? cmd.substring(0, commas[0]) : cmd;
  String arg1 = found > 0 ? cmd.substring(commas[0] + 1, found > 1 ? commas[1] : cmd.length()) : "";
  String arg2 = found > 1 ? cmd.substring(commas[1] + 1, found > 2 ? commas[2] : cmd.length()) : "";
  String arg3 = found > 2 ? cmd.substring(commas[2] + 1) : "";
  int channel, duration, level;

  if (operation == "HW_STATUS" && found == 0) {
    DeviceIdentity id; mev_load_identity(&id);
    String dev = (mev_identity_valid(&id) && id.device_id[0]) ? id.device_id : "BLANK";
    bool temperatureControlActive = false;
    for (int i = 0; i < num_vials; i++) temperatureControlActive = temperatureControlActive || temperatureControlEnabled[i];
    String mode = hardwareTestMode ? "hardware_test" : (normalControllerSuspended ? "idle" : "normal");
    hwReply("OK", "STATUS", "sleeves=2,pumps=6,fw=" + String(MEV_FW_VER_MAJOR) + "." + String(MEV_FW_VER_MINOR) + ",id=" + dev + ",hw_proto=1,temp_control=" + (temperatureControlActive ? "on" : "off") + ",mode=" + mode);
    return;
  }
  if ((operation == "HW_SAFE" || operation == "HW_ALL_OFF") && found == 0) {
    setHardwareSafeState(); hardwareTestMode = false;
    hwReply("OK", "SAFE", "outputs=off"); return;
  }
  if (operation == "HW_READ_THERMISTOR" && found == 1 && hwChannel(arg1, num_vials, &channel)) {
    enterHardwareTestMode(); hwReply("OK", "THERMISTOR", "channel=" + String(channel) + ",value=" + String(analogRead(temp_pin[channel]))); return;
  }
  if (operation == "HW_READ_PHOTODIODE" && found == 1 && hwChannel(arg1, num_vials, &channel)) {
    enterHardwareTestMode(); hwReply("OK", "PHOTODIODE", "channel=" + String(channel) + ",value=" + String(analogRead(pd_pin[channel]))); return;
  }
  if (operation == "HW_SET_OD_LED" && found == 2 && hwChannel(arg1, num_vials, &channel) && hwRange(arg2, HW_MAX_LED_LEVEL, &level)) {
    enterHardwareTestMode(); analogWrite(4 + channel, level);
    hwReply("OK", "SET_OD_LED", "channel=" + String(channel) + ",pin=" + String(4 + channel) + ",level=" + String(level)); return;
  }
  if (operation == "HW_PULSE_PUMP" && found == 2 && hwChannel(arg1, numPumps, &channel) && hwRange(arg2, HW_MAX_PUMP_MS, &duration) && duration > 0) {
    enterHardwareTestMode(); pumpWrite(channel, true); pumpPulse = {true, channel, millis() + (unsigned long)duration};
    hwReply("OK", "PULSE_PUMP", "channel=" + String(channel) + ",pin=" + String(pumpOutputPin[channel]) + ",duration_ms=" + String(duration)); return;
  }
  if (operation == "HW_PULSE_STIR" && found == 3 && hwChannel(arg1, num_vials, &channel) && hwRange(arg2, HW_MAX_STIR_MS, &duration) && duration > 0 && hwRange(arg3, HW_MAX_STIR_LEVEL, &level) && level > 0) {
    enterHardwareTestMode(); pwm.analogWrite(stirOutputPin[channel], level); stirPulse = {true, channel, millis() + (unsigned long)duration};
    hwReply("OK", "PULSE_STIR", "channel=" + String(channel) + ",pin=" + String(stirOutputPin[channel]) + ",duration_ms=" + String(duration) + ",level=" + String(level)); return;
  }
  if (operation == "HW_PULSE_HEATER" && found == 3 && hwChannel(arg1, num_vials, &channel) && hwRange(arg2, HW_MAX_HEATER_MS, &duration) && duration > 0 && hwRange(arg3, HW_MAX_HEATER_LEVEL, &level) && level > 0) {
    enterHardwareTestMode(); analogWrite(tempOutputPin[channel], level); heaterPulse = {true, channel, millis() + (unsigned long)duration};
    hwReply("OK", "PULSE_HEATER", "channel=" + String(channel) + ",pin=" + String(tempOutputPin[channel]) + ",duration_ms=" + String(duration) + ",level=" + String(level)); return;
  }
  String responseOperation = operation; responseOperation.replace("HW_", "");
  hwReply("ERR", responseOperation.c_str(), "reason=invalid_command_or_arguments");
}

void serialEvent() {
  while (SerialUSB.available()) {
    char inChar = (char)SerialUSB.read();
    input_string += inChar;
    if (inChar == '!') {
      string_complete = true;
      break;
    }
  }
}

void dataResponse(String addr, int data[]) {
  String outputString = addr + "b,";
  for (int n = 0; n < num_vials; n++) {
      outputString += data[n];
      outputString += comma;
  }  
  outputString += end_mark;
  SerialUSB.println(outputString);
}

void dataResponse(String addr, double data[]) {
  int data_int[2];
  for (int i = 0; i < num_vials; i++) {
    data_int[i] = data[i];
  }
  dataResponse(addr, data_int);
}

void readPD() {
  unsigned long total = 0;
  for (int i=0; i < pd_times_averaged; i++) {
    total = total + analogRead(pd_pin[active_vial]);
    serialEvent();
    if (string_complete) {
      break;
    }
  }
  if (!string_complete) {
    pd_output[active_vial] = total / pd_times_averaged;
    if (active_vial == 1) {
      active_vial = 0;
    }
    else {
      active_vial++;  
    }
  }
}

void readTempSensors() {
  unsigned long total[num_vials];
  int readings[num_vials];
  int times_avg = 3;

  memset(total, 0 , sizeof(total));
  for (int i = 0; i < times_avg; i++) {
    for (int j = 0; j < num_vials; j++) {
      total[j] = total[j] + analogRead(temp_pin[j]);  
    }
  }

  for(int i = 0; i < num_vials; i++) {
    readings[i] = total[i] / times_avg;
    tempInput[i] = readings[i];    
  }

}

void updateTemperatureControl() {
  for (int i = 0; i < num_vials; i++) {
    if (!temperatureControlEnabled[i]) {
      tempOutput[i] = 0;
      analogWrite(tempOutputPin[i], 0);
      continue;
    }
    allTempPIDS[i]->Compute();
    int set_value = tempOutput[i];
    analogWrite(tempOutputPin[i], set_value);
  }
}

void photodiodeLogic() {
  if (pd.input_array[0] == "i" || pd.input_array[0] == "r") {
    saved_PD_averaged = pd.input_array[1].toInt();
    new_PDinput = true;
    
    String outputString = photodiode_address + "b,";
    for (int n = 0; n < num_vials; n++) {
        outputString += pd_output[n];
        outputString += comma;
    }  
    outputString += end_mark;
    SerialUSB.println(outputString);
  
  }
  if (pd.input_array[0] == "a" && new_PDinput) {
    pd_times_averaged = saved_PD_averaged;
    new_PDinput = false;
  }
  pd.addressFound = false;  
}

void ledLogic() {
  if (led.input_array[0] == "i" || led.input_array[0] == "r") {
    for (int n = 1; n < num_vials+1; n++) {
      saved_LEDinputs[n-1] = led.input_array[n].toInt();
      new_LEDinput = true;
    }
    String outputString = led_address + "e,";
    for (int n = 1; n < num_vials + 1; n++) {
      outputString += led.input_array[n];
      outputString += comma;
    }
    outputString += end_mark;
    SerialUSB.println(outputString);
  }
  if (led.input_array[0] == "a" && new_LEDinput) {        
    analogWrite(4, saved_LEDinputs[0]);
    analogWrite(5, saved_LEDinputs[1]);
    new_LEDinput = false;
  }
  input_string = "";
  led.addressFound = false;  
}

void tempLogic() {
  if (temp.input_array[0] == "i" || temp.input_array[0] == "r") {
    for (int i = 1; i < num_vials + 1; i++) {
      temp_saved_inputs[i-1] = temp.input_array[i].toInt();  
    }
    temp_new_input = true;
    
    String outputString = temp_address + "b,";
    for (int n = 0; n < num_vials; n++) {
        outputString += String((int)tempInput[n]);
        outputString += comma;
    }  
    outputString += end_mark;
    SerialUSB.println(outputString);
  
  }
  if (temp.input_array[0] == "a" && temp_new_input) {
    for (int i = 0; i < num_vials; i++) {
      tempSetpoint[i] = (double)temp_saved_inputs[i];
      tempOutput[i] = 0;
      temperatureControlEnabled[i] = temp_saved_inputs[i] > 0;
      allTempPIDS[i]->SetMode(
        temperatureControlEnabled[i] ? AUTOMATIC : MANUAL
      );
      if (!temperatureControlEnabled[i]) {
        analogWrite(tempOutputPin[i], 0);
      }
    }
    temp_new_input = false;
  }
  temp.addressFound = false;  
}

void stirLogic() {
  if(stir.input_array[0] == "i" || stir.input_array[0] == "r") {
    for (int i = 1; i < num_vials+1; i++) {
      stir_saved_inputs[i-1] = stir.input_array[i].toInt();
    }
    String outputString = stir_address + "e,";
    for (int n = 1; n < num_vials + 1; n++) {
      outputString += stir.input_array[n];
      outputString += comma;
    }
    outputString += end_mark;
    SerialUSB.println(outputString);
    stir_new_input = true;
  }
  
  if (stir.input_array[0] == "a" && stir_new_input) {
    stir_new_input = false;
    for (int i = 0; i < num_vials; i++) {
      if ((int)stir_saved_inputs[i] > 0) {           
        pwm.analogWrite(stirOutputPin[i], 500);
        delay(75);
      }       
      pwm.analogWrite(stirOutputPin[i], (int)stir_saved_inputs[i] * 10);
    }
  }
  stir.addressFound = false;  
}

void pumpLogic() {
  if (pump.input_array[0] == "i" || pump.input_array[0] == "r") {
    for (int i = 1; i < numPumps+1; i++) {
      pumpSavedInputs[i-1] = pump.input_array[i];  
    }
    pump_new_input = true;
  
    String outputString = pumpAddress + "e,";
    for (int n = 1; n < numPumps + 1; n++) {
      outputString += pump.input_array[n];
      outputString += comma;
    }
    outputString += end_mark;
    SerialUSB.println(outputString);
  
  }
  if (pump.input_array[0] == "a" && pump_new_input) {
    for (int i = 0; i < numPumps; i++) {
      if (pumpSavedInputs[i] != "--") {
        int splitIndx = pumpSavedInputs[i].indexOf("|");
        int ippSplitIndx = pumpSavedInputs[i].indexOf("|", splitIndx+1);
        if (splitIndx == -1) {
          timeToPump = pumpSavedInputs[i].toFloat();
          pumpInterval = 0;
        }
        else if (ippSplitIndx == -1) {
          timeToPump = pumpSavedInputs[i].substring(0, splitIndx).toFloat();
          pumpInterval = pumpSavedInputs[i].substring(splitIndx+1).toInt();
        }
        else {
          // IPP logic not supported!
          ipp++;
        }
  
        if (pumpInterval != 0 && pumps[i].isNewChemostat(timeToPump, pumpInterval)) {
          // unaltered chemostat
          SerialUSB.print("Unaltered chemostat: ");
          SerialUSB.println(i);
        }
        else if (ippSplitIndx == -1) {
          pumps[i].setPump(timeToPump, pumpInterval);  
        }
      }
    }
    pump_new_input = false;
  }
  pump.addressFound = false;
}

// Handle PROVISION,<device_id>,<owner_id>_! command.
// Refuses silently if already provisioned — requires CLEAR_ID first.
void provisioningLogic() {
  // Strip trailing _! and everything after the payload
  String cmd = input_string;
  cmd.replace("_!", "");
  cmd.trim();
  // Expected: "PROVISION,<device_id>,<owner_id>"
  int firstComma  = cmd.indexOf(',');
  int secondComma = cmd.indexOf(',', firstComma + 1);
  if (firstComma < 0 || secondComma < 0) {
    mev_send_provision_err(&SerialUSB, ++mev_seq, "BLANK", "bad_format");
    return;
  }
  String device_id_s = cmd.substring(firstComma + 1, secondComma);
  String owner_id_s  = cmd.substring(secondComma + 1);
  device_id_s.trim();
  owner_id_s.trim();

  if (device_id_s.length() == 0 || owner_id_s.length() == 0 ||
      device_id_s.length() > 31 || owner_id_s.length() > 31) {
    mev_send_provision_err(&SerialUSB, ++mev_seq, "BLANK", "bad_id_length");
    return;
  }

  DeviceIdentity existing;
  mev_load_identity(&existing);
  if (mev_identity_valid(&existing)) {
    // Already provisioned — require explicit CLEAR_ID before re-provisioning
    mev_send_provision_err(&SerialUSB, ++mev_seq, existing.device_id, "already_provisioned");
    return;
  }

  char dev_buf[32], owner_buf[32];
  device_id_s.toCharArray(dev_buf, sizeof(dev_buf));
  owner_id_s.toCharArray(owner_buf, sizeof(owner_buf));
  mev_save_identity(dev_buf, owner_buf);
  mev_send_provision_ack(&SerialUSB, ++mev_seq, dev_buf, owner_buf);
}
