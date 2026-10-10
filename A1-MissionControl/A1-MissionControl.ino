/*
=================================================================
INDUSTRIAL AUTOMATED FIREWATCH (IAF)
ICTE4005 Robotics Project - Group 5
Curtin University, 2026

Team:
Saf Flatters, Annie (Annabelle) Lewkowski, Devlin MacGlip

MODULE:
Arduino 1 (A1) - Mission Control

PURPOSE:
A1 is the main controller for the IAF system.
- Runs the system state machine
- Controls communication with A2 (Perception) and A3 (Robot Manipulator)
- Polls A2 and A3 for system status
- Controls status lights and the emergency beacon/alarm system
- Prints system status to the Serial Monitor

WIRING:

I2C Communication:
SDA (A4) -> A2 and A3 SDA
SCL (A5) -> A2 and A3 SCL
GND      -> Common GND between all Arduino boards

Hot Work Button:
Pin 6 -> Push button -> GND
Uses INPUT_PULLUP, so no external resistor is required.
First press from STANDBY -> HOT_WORK
Next press from HOT_WORK -> STANDBY


(draft)
Status Lights:
Pin 7 -> Hotwork Mode Green
Pin 8 -> Status light*********
Pin 9 -> Status light*********


Alarm:
Alarm -> Pin TBC*******

SERIAL MONITOR TEST CONTROLS (not required for normal use):
1 = HOT WORK
0 = STANDBY
9 = EMERGENCY STOP
r = Clear FAULT
a = Test: Simulate A2 confirmed fire
b = Test: Simulate A3 ready


SYSTEM SEQUENCE:

SEQ 01 - System startup / STANDBY
SEQ 02 - Operator starts HOT WORK
SEQ 03 - Monitor for fire
SEQ 04 - Possible fire detected
SEQ 05 - Aim robot at target
SEQ 06 - Verify fire with CV + thermal
SEQ 07A - Fire NOT confirmed
SEQ 07B - Fire CONFIRMED
SEQ 08 - Suppress fire
SEQ 09 - Stop suppression and recheck
SEQ 10 - Return robot to PARKED
SEQ 11 - Resume HOT WORK monitoring

=================================================================
*/

#include <Wire.h>

// I2C ADDRESSES
const byte A2_ADDR = 8;
const byte A3_ADDR = 9;

// For testing. FALSE if using the real A3 robot
const bool SIMULATE_A3 = false;

// COMMANDS TO A2
const byte CMD_A2_HOTWORK       = 1;
const byte CMD_A2_VERIFY_TARGET = 2;
const byte CMD_A2_STANDBY       = 3;

// STATUS FROM A2
const byte A2_NO_FIRE        = 0;
const byte A2_POSSIBLE_FIRE  = 1;
const byte A2_FIRE_CONFIRMED = 2;
const byte A2_VERIFYING      = 3;
const byte A2_STANDBY        = 4;

const byte A2_REPLY_LENGTH = 9;

// COMMANDS TO A3
const byte CMD_A3_STANDBY        = 10;
const byte CMD_A3_TARGET         = 11;
const byte CMD_A3_SUPPRESS_START = 12;
const byte CMD_A3_SUPPRESS_STOP  = 13;

// STATUS FROM A3
const byte A3_PARKED               = 20;
const byte A3_READY                = 21;
const byte A3_SUPPRESSING          = 22;
const byte A3_SUPPRESSION_COMPLETE = 23;

// INFORMATION RECEIVED FROM A2
int16_t targetX = 0;
int16_t targetY = 0;

float cvConfidence   = 0.0;
float fireConfidence = 0.0;



// SYSTEM STATES
enum SystemState {
  STANDBY,             // SEQ 01 - System startup / STANDBY
  HOT_WORK,            // SEQ 02 - Operator starts HOT WORK
  TARGETING,           // SEQ 05 - Aim robot at target
  VERIFYING,           // SEQ 06 - Verify fire with CV + thermal
  RETURNING_TO_PARK,   // SEQ 10 - Return robot to PARKED
  FIRE_CONFIRMED,      // SEQ 07B - Fire CONFIRMED
  SUPPRESSING,         // SEQ 08 - Suppress fire
  RECHECK,             // SEQ 09 - Stop suppression and recheck
  FAULT
};

SystemState currentState = STANDBY;

// PINS
const int PIN_HOTWORK_BUTTON = 6;
//const int PIN_PUMP       = 10; // blue led replacing water 
const int PIN_GREEN_LED      = 7; // green is hotworks mode on 
const int PIN_RED_LED        = 8; // Red = Fire confirmed
const int PIN_YELLOW_LED     = 9; // Yellow = fire detected 
const int PIN_ALARM          = 11;

// BUTTON VARIABLES for SEQ 1 - startup / standby OR SEQ 2 - start hot work
bool lastButtonState = HIGH;
unsigned long lastButtonPress = 0;
const unsigned long BUTTON_DEBOUNCE = 300;

// TIMING - required to give A2/A3 time to respond
const unsigned long POLL_INTERVAL    = 500;
const unsigned long SUPPRESSION_TIME = 20000;  // 20 second suppression cycle
const unsigned long STEP_TIMEOUT     = 10000;  // 10 second state timeout

const byte MAX_MISSED_REPLIES = 3;

// ALARM TONES ****
const int FIRE_ALARM_TONE  = 2000;
const int FIRE_ALARM_TONE_UP = 4000; 
const int FAULT_ALARM_TONE = 800;

// SERIAL PRINTING
const bool PRINT_EVERY_POLL = false;

// VARIABLES
byte a2Status = A2_STANDBY;
byte a3Status = A3_PARKED;
unsigned long stateStartTime     = 0;
unsigned long lastPollTime       = 0;
unsigned long lastCountdownPrint = 0;
int alarmTone        = 0;
int alarmTonePlaying = 0;

byte missedReplies     = 0;
byte suppressionCycles = 0;
bool recheckVerifySent = false;

// TEST VARIABLES
bool testA2Confirmed = false;
bool testA3Ready     = false;
byte simA3LastCommand = CMD_A3_STANDBY;
unsigned long simA3AimStart = 0;


// SETUP
// SEQ 01 - SYSTEM STARTUP / STANDBY
// Runs once when A1 is powered on or reset

void setup() {

  // Start Serial Monitor communication
  Serial.begin(9600);
  // Start A1 as the I2C controller
  Wire.begin();
  // Set I2C timeout so A1 does not wait forever for A2/A3
  Wire.setWireTimeout(25000, true);

  // Set output pins
  //pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);
  pinMode(PIN_RED_LED, OUTPUT);
  pinMode(PIN_YELLOW_LED, OUTPUT);
  pinMode(PIN_ALARM, OUTPUT);

  // Hot Work button uses Arduino's internal pull-up resistor
  pinMode(PIN_HOTWORK_BUTTON, INPUT_PULLUP);

  // Make sure outputs are OFF at startup
  //digitalWrite(PIN_PUMP, LOW);
  digitalWrite(PIN_GREEN_LED, LOW);
  digitalWrite(PIN_RED_LED, LOW);
  digitalWrite(PIN_YELLOW_LED, LOW);

  // Allow system to settle after startup
  delay(1000);

  // Print A1 information and test controls to Serial Monitor
  Serial.println(F("FIREWATCH A1 MISSION CONTROLLER"));
  Serial.println(F("Keys: 1=Hot Work  0=Standby  9=E-STOP  r=clear fault"));
  Serial.println(F("Test: a=pretend A2 confirmed  b=pretend A3 ready/done"));
  Serial.println(F("Physical button: STANDBY <-> HOT WORK"));

  // Start the system in STANDBY
  changeState(STANDBY);
}


// MAIN LOOP
// Runs continuously and calls the functions that control the IAF system.
void loop() {
  handleHotWorkButton();    // Checks for operator HOT WORK / STANDBY input
  handleSerialInput();      // Checks for Serial Monitor test commands
  runStateMachine();        // Runs the current step of the system sequence
  updateBeaconAndAlarm();   // Updates warning lights and alarm
}


// PHYSICAL HOT WORK BUTTON
// SEQ 02 - Operator starts HOT WORK
// Toggles between STANDBY and HOT_WORK. Button is ignored during a fire event.

void handleHotWorkButton() {
  bool buttonState = digitalRead(PIN_HOTWORK_BUTTON);
  // Detect new button press and debounce
  if (buttonState == LOW && lastButtonState == HIGH) {
    if (millis() - lastButtonPress > BUTTON_DEBOUNCE) {
      lastButtonPress = millis();

      // SEQ 02 - Start HOT WORK monitoring
      if (currentState == STANDBY) {
        Serial.println(F("BUTTON: HOT WORK MODE selected"));
        changeState(HOT_WORK);
      }

      // Operator manually returns system to STANDBY
      else if (currentState == HOT_WORK) {
        Serial.println(F("BUTTON: STANDBY selected"));
        changeState(STANDBY);
      }

      // Prevent mode changes while handling a fire event
      else {
        Serial.println(F("BUTTON ignored - system is handling a fire/event"));
      }
    }
  }

  // Save button state for next loop
  lastButtonState = buttonState;
}


// SERIAL MONITOR - TESTING AND FAULT RECOVERY ONLY
// Not required during normal system operation.
// Allows manual state changes, simulated A2/A3 responses and fault clearing.
void handleSerialInput() {
  if (!Serial.available()) return;
  char key = Serial.read();
  switch (key) {
    // TEST: Manually start HOT WORK from STANDBY
    case '1':
      if (currentState == STANDBY) {
        Serial.println(F("OPERATOR: HOT WORK MODE selected"));
        changeState(HOT_WORK);
      }
      else {
        Serial.println(F("(Hot Work can only be started from STANDBY)"));
      }
      break;

    // TEST: Manually return system to STANDBY
    case '0':
      if (currentState == FAULT) {
        Serial.println(F("(In FAULT, press r to clear it first, or 9 for E-STOP)"));
      }
      else {
        Serial.println(F("OPERATOR: STANDBY selected"));
        changeState(STANDBY);
      }
      break;

    // EMERGENCY STOP: Immediately return system to STANDBY
    case '9':
      Serial.println(F("EMERGENCY STOP"));
      changeState(STANDBY);
      break;

    // FAULT RECOVERY: Clear FAULT and return to STANDBY
    case 'r':
      if (currentState == FAULT) {
        Serial.println(F("OPERATOR: fault cleared"));
        changeState(STANDBY);
      }
      else {
        Serial.println(F("(no fault to clear)"));
      }
      break;

    // TEST: Simulate A2 confirming a fire
    case 'a':
      testA2Confirmed = true;
      Serial.println(F("TEST pretending A2 = FIRE_CONFIRMED"));
      break;

    // TEST: Simulate A3 reporting READY / COMPLETE
    case 'b':
      testA3Ready = true;
      Serial.println(F("TEST pretending A3 = READY or DONE"));
      break;

    // Ignore line endings from Serial Monitor
    case '\n':
    case '\r':
      break;

    // Catch incorrect test commands
    default:
      Serial.print(F("(unknown key: "));
      Serial.print(key);
      Serial.println(F(")"));
      break;
  }
}


// CHANGE STATE
// Performs the actions required when A1 enters a new system state.
// Resets state timing and communication checks before running state-specific actions.
void changeState(SystemState newState) {
  // Print state transition to Serial Monitor
  Serial.println(F("____________________________________________"));
  Serial.print(F(" STATE: "));
  Serial.print(stateName(currentState));
  Serial.print(F(" -> "));
  Serial.println(stateName(newState));
  Serial.println(F("____________________________________________"));

  // Update state and reset timing / communication tracking
  currentState = newState;
  stateStartTime = millis();
  lastPollTime = millis() - POLL_INTERVAL;
  missedReplies = 0;

  switch (newState) {

// SEQ 01 - SYSTEM STANDBY
    // Stop outputs and tell A2 and A3 to return to STANDBY.
    case STANDBY:
      //digitalWrite(PIN_PUMP, LOW);
      alarmTone = 0;
      suppressionCycles = 0;
      sendCommand(A3_ADDR, CMD_A3_STANDBY);
      sendCommand(A2_ADDR, CMD_A2_STANDBY);
      break;

// SEQ 02 - START HOT WORK
    // Start monitoring by placing A2 in HOT_WORK. A3 remains parked.
    case HOT_WORK:
      //digitalWrite(PIN_PUMP, HIGH);
      alarmTone = 0;
      suppressionCycles = 0;
      sendCommand(A3_ADDR, CMD_A3_STANDBY);
      if (!sendCommand(A2_ADDR, CMD_A2_HOTWORK)) {
        enterFault(F("A2 did not accept HOT_WORK"));
        return;
      }
      break;

// SEQ 05 - AIM ROBOT AT TARGET
    // Send the fire coordinates received from A2 to A3.
    case TARGETING:
      alarmTone = 0;
      testA3Ready = false;
      Serial.print(F("  Target X="));
      Serial.print(targetX);
      Serial.print(F("  Y="));
      Serial.print(targetY);
      Serial.print(F("  CV confidence="));
      Serial.println(cvConfidence);
      if (!sendTargetToA3()) {
        enterFault(F("A3 did not accept TARGET"));
        return;
      }
      break;


// SEQ 06 - VERIFY FIRE
    // Ask A2 to verify the target using fresh CV and thermal data.
    case VERIFYING:
      testA2Confirmed = false;
      if (!sendCommand(A2_ADDR, CMD_A2_VERIFY_TARGET)) {
        enterFault(F("A2 did not accept VERIFY_TARGET"));
        return;
      }
      break;

// SEQ 10 - RETURN ROBOT TO PARKED
    // Stop CV before moving A3 so another fire cannot be detected
    // while the camera is returning to centre.
    case RETURNING_TO_PARK:
      Serial.println(F("  Stopping CV while A3 returns to centre"));
      if (!sendCommand(A2_ADDR, CMD_A2_STANDBY)) {
        enterFault(F("A2 did not accept STANDBY"));
        return;
      }
      if (!sendCommand(A3_ADDR, CMD_A3_STANDBY)) {
        enterFault(F("A3 did not accept STANDBY"));
        return;
      }
      break;

// SEQ 07B - FIRE CONFIRMED
    // Sound the fire alarm and tell A3 to begin suppression.
    case FIRE_CONFIRMED:
      alarmTone = FIRE_ALARM_TONE;
      suppressionCycles++;
      Serial.print(F("ATTENTION!!! FIRE HAS BEEN CONFIRMED! Fire confidence = "));
      Serial.print(fireConfidence);
      Serial.print(F("  suppression cycle #"));
      Serial.println(suppressionCycles);
      if (!sendCommand(A3_ADDR, CMD_A3_SUPPRESS_START)) {
        enterFault(F("A3 did not accept SUPPRESS_START"));
        return;
      }
      break;


// SEQ 08 - SUPPRESS FIRE
    // Start timing the suppression cycle.
    case SUPPRESSING:
      lastCountdownPrint = millis();
      break;

// SEQ 09 - STOP SUPPRESSION AND RECHECK
    // Tell A3 to stop suppression before checking the fire again.
    case RECHECK:
      recheckVerifySent = false;
      testA2Confirmed = false;
      testA3Ready = false;
      if (!sendCommand(A3_ADDR, CMD_A3_SUPPRESS_STOP)) {
        enterFault(F("A3 did not accept SUPPRESS_STOP"));
        return;
      }
      break;

// FAULT - SAFE STATE
    // Stop the system, sound the fault alarm and place A2/A3 in STANDBY.
    case FAULT:
      //digitalWrite(PIN_PUMP, LOW);
      alarmTone = FAULT_ALARM_TONE;
      sendCommand(A3_ADDR, CMD_A3_STANDBY);
      sendCommand(A2_ADDR, CMD_A2_STANDBY);
      Serial.println(F("  Pump OFF. Check wiring, then press r."));
      break;
  }
}


// ENTER FAULT
// Called when A1 detects a communication or system failure.
// Prints the fault reason, then moves the system into the safe FAULT state.
void enterFault(const __FlashStringHelper* reason) {
  // Print the cause of the fault
  Serial.print(F("FAULT: "));
  Serial.println(reason);
  // Enter FAULT state to stop operation and place the system in a safe state
  changeState(FAULT);
}


// STATE MACHINE
// Controls the sequence of the IAF system based on the current state.
// A1 polls A2/A3 for status and moves to the next state when required.
void runStateMachine() {
  switch (currentState) {

// SEQ 01 - STANDBY
    // Wait for the operator to start HOT WORK.
    case STANDBY:
      break;

// SEQ 03 - MONITOR FOR FIRE
    // Poll A2 while it monitors for fire.
    case HOT_WORK:
      if (!timeToPoll())
        break;

      if (!readA2())
        break;

// SEQ 04 - POSSIBLE FIRE DETECTED
      // A2 has found a possible fire - begin targeting.
      if (a2Status == A2_POSSIBLE_FIRE) {
        changeState(TARGETING);
      }
      break;

// SEQ 05 - AIM ROBOT AT TARGET
    // Wait for A3 to aim at the coordinates and report READY.
    case TARGETING:
      // Testing only: simulate A3 reporting READY
      if (testA3Ready) {
        changeState(VERIFYING);
        break;
      }
      // Enter FAULT if A3 fails to aim within the timeout
      if (stateTimedOut()) {
        enterFault(F("A3 never reported READY"));
        break;
      }
      if (!timeToPoll())
        break;
      if (!readA3())
        break;
      // A3 is aimed at target - begin fire verification
      if (a3Status == A3_READY) {
        Serial.println(F("  A3 is aimed - asking A2 to verify"));
        changeState(VERIFYING);
      }
      break;

// SEQ 06 - VERIFY FIRE WITH CV + THERMAL
    // Wait for A2 to determine whether the possible fire is genuine.
    case VERIFYING:
      // Testing only: simulate A2 confirming fire
      if (testA2Confirmed) {
        changeState(FIRE_CONFIRMED);
        break;
      }
      // If verification times out, treat as unconfirmed and return to park
      if (stateTimedOut()) {
        Serial.println(
          F("  A2 did not confirm fire within 10 seconds")
        );
        changeState(RETURNING_TO_PARK);
        break;
      }
      if (!timeToPoll())
        break;
      if (!readA2())
        break;

// SEQ 07B - FIRE CONFIRMED
      if (a2Status == A2_FIRE_CONFIRMED) {
        changeState(FIRE_CONFIRMED);
      }

// SEQ 07A - FIRE NOT CONFIRMED
      // Stop monitoring temporarily and return A3 to park.
      else if (a2Status == A2_NO_FIRE) {
        Serial.println(
          F("  Fire NOT confirmed - stopping CV and returning robot to centre")
        );
        changeState(RETURNING_TO_PARK);
      }
      break;

// SEQ 10 - RETURN ROBOT TO PARKED
    // Wait until A3 physically reaches its parked position.
    // A2 remains in STANDBY so CV cannot detect another target while A3 moves.
    case RETURNING_TO_PARK:
      if (stateTimedOut()) {
        enterFault(F("A3 did not return to PARKED"));
        break;
      }
      if (!timeToPoll())
        break;
      if (!readA3())
        break;

// SEQ 11 - RESUME HOT WORK MONITORING
      // Only restart monitoring once A3 confirms it is physically parked.
      if (a3Status == A3_PARKED) {
        Serial.println(
          F("  A3 physically PARKED - restarting fire monitoring")
        );
        changeState(HOT_WORK);
      }
      break;

// SEQ 07B - FIRE CONFIRMED
    // Fire has been confirmed and suppression has been commanded.
    // Move immediately into the SUPPRESSING state.
    case FIRE_CONFIRMED:
      changeState(SUPPRESSING);
      break;

// SEQ 08 - SUPPRESS FIRE
    // Continue suppression for 20 seconds while monitoring A3.
    case SUPPRESSING: {
      unsigned long elapsed = millis() - stateStartTime;
      // Suppression time complete - stop water and begin recheck
      if (elapsed >= SUPPRESSION_TIME) {
        Serial.println(F("  20 s complete - stopping water"));
        changeState(RECHECK);
        break;
      }
      // Print suppression countdown once per second
      if (millis() - lastCountdownPrint >= 1000) {
        lastCountdownPrint = millis();
        Serial.print(F("  Suppressing... "));
        Serial.print((SUPPRESSION_TIME - elapsed) / 1000);
        Serial.println(F(" s left"));
      }
      // Continue checking A3 during suppression
      if (timeToPoll()) {
        readA3();
      }
      break;
    }

// SEQ 09 - STOP SUPPRESSION AND RECHECK
    // Wait for A3 to confirm suppression has stopped,
    // then ask A2 to verify the same target again.
    case RECHECK:
      if (stateTimedOut()) {
        enterFault(F("recheck did not finish"));
        break;
      }
      // Wait for A3 to confirm suppression has stopped
      if (!recheckVerifySent) {
        if (!testA3Ready) {
          if (!timeToPoll())
            break;

          if (!readA3())
            break;

          if (a3Status != A3_SUPPRESSION_COMPLETE)
            break;
        }
        // A3 has stopped suppression - ask A2 to recheck target
        Serial.println(
          F("  Valve closed - asking A2 to re-verify the same spot")
        );
        if (!sendCommand(A2_ADDR, CMD_A2_VERIFY_TARGET)) {
          enterFault(F("A2 did not accept VERIFY_TARGET"));
          break;
        }
        recheckVerifySent = true;
        break;
      }

      // Testing only: simulate fire still being confirmed
      if (testA2Confirmed) {
        changeState(FIRE_CONFIRMED);
        break;
      }

      if (!timeToPoll())
        break;

      if (!readA2())
        break;

      // Fire still detected - repeat suppression cycle
      if (a2Status == A2_FIRE_CONFIRMED) {
        Serial.println(F("  STILL BURNING - suppressing again"));
        changeState(FIRE_CONFIRMED);
      }

      // Fire extinguished - return robot to park
      else if (a2Status == A2_NO_FIRE) {
        Serial.println(
          F("  FIRE OUT - returning robot to centre")
        );
        changeState(RETURNING_TO_PARK);
      }
      break;

    // FAULT - SAFE STATE
    // Remain here until the operator clears the fault.
    case FAULT:
      break;
  }
}


// I2C SEND COMMAND
// Sends a command byte from A1 to A2 or A3.
// Returns true if the receiving Arduino acknowledges the transmission.
// If A3 simulation is enabled, the command is handled without I2C communication.
bool sendCommand(byte addr, byte cmd) {
  // Testing only: simulate A3 instead of sending the command physically
  if (SIMULATE_A3 && cmd >= CMD_A3_STANDBY) {
    simulateA3Command(cmd);
    return true;
  }
  // Start I2C transmission and send command
  Wire.beginTransmission(addr);
  Wire.write(cmd);

  // Finish transmission and store acknowledgement/error result
  byte result = Wire.endTransmission();

  // Print command and destination to Serial Monitor
  Serial.print(F("  A1 -> "));
  Serial.print(boardName(addr));
  Serial.print(F(": "));
  Serial.print(cmd);
  Serial.print(F(" ("));
  Serial.print(commandName(cmd));
  Serial.print(F(")"));

  // Report failed I2C transmission
  if (result != 0) {
    Serial.print(F("   < NOT RECEIVED (error "));
    Serial.print(result);
    Serial.print(F(")"));
  }

  Serial.println();

  // Successful I2C transmission returns 0
  return result == 0;
}

// SEND TARGET TO A3
// SEQ 05 - Aim robot at target
// Sends the fire X/Y coordinates received from A2 to A3 over I2C.
// Returns true if A3 acknowledges the transmission.
bool sendTargetToA3() {
  // Testing only: simulate A3 receiving the target
  if (SIMULATE_A3) {
    Serial.print(F("  [SIM] X="));
    Serial.print(targetX);
    Serial.print(F(" Y="));
    Serial.print(targetY);
    Serial.print(F("  "));
    simulateA3Command(CMD_A3_TARGET);
    return true;
  }
  // Send TARGET command followed by X/Y coordinates to A3
  Wire.beginTransmission(A3_ADDR);
  Wire.write(CMD_A3_TARGET);
  Wire.write((byte*)&targetX, sizeof(targetX));
  Wire.write((byte*)&targetY, sizeof(targetY));
  byte result = Wire.endTransmission();

  // Print target transmission to Serial Monitor
  Serial.print(F("  A1 -> A3: "));
  Serial.print(CMD_A3_TARGET);
  Serial.print(F(" (TARGET) X="));
  Serial.print(targetX);
  Serial.print(F(" Y="));
  Serial.print(targetY);
  // Report failed I2C transmission
  if (result != 0) {
    Serial.print(F("   < NOT RECEIVED (error "));
    Serial.print(result);
    Serial.print(F(")"));
  }
  Serial.println();
  // I2C result 0 means A3 acknowledged the transmission
  return result == 0;
}


// READ A2
// Requests the current status and fire data from A2 over I2C.
// The data received depends on the status reported by A2.
// SEQ 04 - Possible fire detected, SEQ 07B - Fire CONFIRMED
bool readA2() {
  // Request A2 status/data packet
  byte received =
    Wire.requestFrom(A2_ADDR, A2_REPLY_LENGTH);

  // No response received from A2
  if (received == 0) {
    replyMissed(A2_ADDR);
    return false;
  }
  missedReplies = 0;

  // First byte contains A2's current status
  byte newStatus = Wire.read();

// SEQ 04 - POSSIBLE FIRE DETECTED
  // Receive target coordinates and CV confidence from A2.
  if (newStatus == A2_POSSIBLE_FIRE) {
    Wire.readBytes(
      (byte*)&targetX,
      sizeof(targetX)
    );
    Wire.readBytes(
      (byte*)&targetY,
      sizeof(targetY)
    );
    Wire.readBytes(
      (byte*)&cvConfidence,
      sizeof(cvConfidence)
    );
  }

// SEQ 07B - FIRE CONFIRMED
  // Receive combined fire confidence from A2.
  else if (newStatus == A2_FIRE_CONFIRMED) {
    Wire.readBytes(
      (byte*)&fireConfidence,
      sizeof(fireConfidence)
    );
  }

  // Print A2 response when its status changes
  if (newStatus != a2Status || PRINT_EVERY_POLL) {
    Serial.print(F("  A2 -> A1: "));
    Serial.print(newStatus);
    Serial.print(F(" ("));
    Serial.print(statusName(newStatus));
    Serial.print(F(")"));
    if (newStatus == A2_POSSIBLE_FIRE) {
      Serial.print(F("  X="));
      Serial.print(targetX);
      Serial.print(F(" Y="));
      Serial.print(targetY);
      Serial.print(F(" CV="));
      Serial.print(cvConfidence);
    }
    else if (newStatus == A2_FIRE_CONFIRMED) {
      Serial.print(F("  fireConfidence="));
      Serial.print(fireConfidence);
    }
    Serial.println();
  }
  // Save latest A2 status for the state machine
  a2Status = newStatus;
  return true;
}


// READ A3
// Requests A3's current status over I2C.
// Used to check robot aiming, suppression and return-to-park progress.
// SEQ 05 - A3_READY, SEQ 09 - A3_SUPPRESSION_COMPLETE, SEQ 10 - A3_PARKED
bool readA3() {
  byte newStatus;
  // Testing only: use simulated A3 response
  if (SIMULATE_A3) {
    newStatus = simulatedA3Status();
  }
  else {
    // Request current status byte from A3
    byte received =
      Wire.requestFrom(A3_ADDR, (byte)1);
    // No response received from A3
    if (received == 0) {
      replyMissed(A3_ADDR);
      return false;
    }
    newStatus = Wire.read();
  }
  missedReplies = 0;
  // Print A3 response when its status changes
  if (newStatus != a3Status || PRINT_EVERY_POLL) {
    Serial.print(F("  A3 -> A1: "));
    Serial.print(newStatus);
    Serial.print(F(" ("));
    Serial.print(statusName(newStatus));
    Serial.println(F(")"));
  }
  // Save latest A3 status for the state machine
  a3Status = newStatus;
  return true;
}


/// MISSED I2C REPLY
// Tracks consecutive missed replies from A2 or A3.
// Enters FAULT if a board fails to respond MAX_MISSED_REPLIES times.
void replyMissed(byte addr) {
  missedReplies++;
  Serial.print(F("  ? "));
  Serial.print(boardName(addr));
  Serial.print(F(" did not reply ("));
  Serial.print(missedReplies);
  Serial.print(F("/"));
  Serial.print(MAX_MISSED_REPLIES);
  Serial.println(F(")"));
  // Too many missed replies - communication fault
  if (missedReplies >= MAX_MISSED_REPLIES) {
    enterFault(F("lost comms with a board"));
  }
}

// SIMULATED A3 - TESTING ONLY
// Simulates A3 receiving a command when SIMULATE_A3 is enabled.
// Allows A1 to be tested without the physical Robot Manipulator.
void simulateA3Command(byte cmd) {
  simA3LastCommand = cmd;
  // Simulate time required for A3 to aim at a target
  if (cmd == CMD_A3_TARGET) {
    simA3AimStart = millis();
  }
  Serial.print(F("  [SIM] A1 -> A3: "));
  Serial.print(cmd);
  Serial.print(F(" ("));
  Serial.print(commandName(cmd));
  Serial.println(F(")"));
}

// SIMULATED A3 STATUS - TESTING ONLY
// Returns a simulated A3 status based on the last command received.
byte simulatedA3Status() {
  switch (simA3LastCommand) {
    // SEQ 05 - Simulate A3 taking 2 seconds to aim
    case CMD_A3_TARGET:
      if (millis() - simA3AimStart >= 2000)
        return A3_READY;
      return A3_PARKED;

    // SEQ 08 - Simulate active suppression
    case CMD_A3_SUPPRESS_START:
      return A3_SUPPRESSING;

    // SEQ 09 - Simulate completed suppression
    case CMD_A3_SUPPRESS_STOP:
      return A3_SUPPRESSION_COMPLETE;

    default:
      return A3_PARKED;
  }
}

// POLLING TIMER
// Limits how often A1 requests status updates from A2 and A3.
// Returns true when the next poll is due.
bool timeToPoll() {
  if (millis() - lastPollTime < POLL_INTERVAL)
    return false;
  lastPollTime = millis();
  return true;
}

// STATE TIMEOUT
// Checks whether the current state has exceeded STEP_TIMEOUT.
// Used to prevent the system waiting indefinitely for A2 or A3.
bool stateTimedOut() {
  return millis() - stateStartTime >= STEP_TIMEOUT;
}


// STATUS LIGHTS AND ALARM
// SEQ 01-11 - Provides visual indication of the current system state.
// Yellow = HOT WORK / system activity
// Red + Yellow = confirmed fire / suppression response
// Alarm tone is controlled separately for FIRE and FAULT conditions.
void updateBeaconAndAlarm() {
  // Creates flashing lights by alternating every 250 ms
  bool flashOn = (millis() / 250) % 2;
  switch (currentState) {
    // No status lights
    case STANDBY:
    case FAULT:
      digitalWrite(PIN_GREEN_LED, LOW);
      digitalWrite(PIN_YELLOW_LED, LOW);
      digitalWrite(PIN_RED_LED, LOW);
      break;

// SEQ 02 + 03 - HOT WORK active - steady green
    case HOT_WORK:
      digitalWrite(PIN_GREEN_LED, HIGH);
      digitalWrite(PIN_YELLOW_LED, LOW);
      digitalWrite(PIN_RED_LED, LOW);
      break;

// SEQ 04 + 05 + 06 + 10 - Possible fire being handled - flashing yellow
    case TARGETING:
    case VERIFYING:
    case RETURNING_TO_PARK:
      digitalWrite(PIN_YELLOW_LED, flashOn);
      digitalWrite(PIN_GREEN_LED, LOW);
      digitalWrite(PIN_RED_LED, LOW);
      break;

// SEQ 07B + 08 + 09 - Confirmed fire / suppression - flashing red and yellow
    case FIRE_CONFIRMED:
    case SUPPRESSING:
    case RECHECK:
      digitalWrite(PIN_YELLOW_LED, LOW);
      digitalWrite(PIN_RED_LED, flashOn);
      digitalWrite(PIN_GREEN_LED, LOW);
      break;
    }

// SEQ 07B + 08 + 09 - Fire alarm active during confirmed fire response
    // Update alarm only when the requested tone changes
    if (alarmTone != alarmTonePlaying) {
      if (alarmTone == 0) {
        noTone(PIN_ALARM);
      }
      else {
        tone(PIN_ALARM, alarmTone);
      }
      alarmTonePlaying = alarmTone;
    }
  }

// COMMAND NAMES
// Converts I2C command codes into readable names for Serial Monitor output.
const __FlashStringHelper* commandName(byte code) {
  switch (code) {
    case 1:
      return F("HOT_WORK");
    case 2:
      return F("VERIFY_TARGET");
    case 3:
      return F("STANDBY");
    case 10:
      return F("STANDBY");
    case 11:
      return F("TARGET");
    case 12:
      return F("SUPPRESS_START");
    case 13:
      return F("SUPPRESS_STOP");
    default:
      return F("UNKNOWN");
  }
}

// STATUS NAMES
// Converts A2/A3 status codes into readable names for Serial Monitor output.
const __FlashStringHelper* statusName(byte code) {
  switch (code) {
    case 0:
      return F("NO_FIRE");
    case 1:
      return F("POSSIBLE_FIRE");
    case 2:
      return F("FIRE_CONFIRMED");
    case 3:
      return F("VERIFYING");
    case 4:
      return F("STANDBY");
    case 20:
      return F("PARKED");
    case 21:
      return F("READY");
    case 22:
      return F("SUPPRESSING");
    case 23:
      return F("SUPPRESSION_COMPLETE");
    case 255:
      return F("NO DATA");
    default:
      return F("UNKNOWN");
  }
}

// BOARD NAMES
// Converts I2C addresses into readable board names for Serial Monitor output.
const __FlashStringHelper* boardName(byte addr) {
  if (addr == A2_ADDR)
    return F("A2");
  if (addr == A3_ADDR)
    return F("A3");
  return F("??");
}

// STATE NAMES
// Converts A1 system states into readable names for Serial Monitor output.
const __FlashStringHelper* stateName(SystemState s) {
  switch (s) {
    case STANDBY:
      return F("STANDBY");
    case HOT_WORK:
      return F("HOT_WORK");
    case TARGETING:
      return F("TARGETING");
    case VERIFYING:
      return F("VERIFYING");
    case RETURNING_TO_PARK:
      return F("RETURNING_TO_PARK");
    case FIRE_CONFIRMED:
      return F("FIRE_CONFIRMED");
    case SUPPRESSING:
      return F("SUPPRESSING");
    case RECHECK:
      return F("RECHECK");
    case FAULT:
      return F("FAULT");
  }

  return F("?");
}