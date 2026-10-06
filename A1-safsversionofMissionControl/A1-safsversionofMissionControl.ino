/*
  Firewatch Arduino 1 Mission Controller

  A1 role:
  - I2C controller
  - Runs the state machine
  - Polls A2 (perception) and A3 (robotics / suppression)
  - Beacon/alarm emergency system
  - Prints system status to Serial Monitor

  WIRING:
  SDA -> A4
  SCL -> A5
  Common GND between all boards

  HOT WORK BUTTON:
  Pin 6 -> push button -> GND
  Uses INPUT_PULLUP, so no external resistor required.

  First press from STANDBY -> HOT_WORK
  Next press from HOT_WORK -> STANDBY

  SERIAL MONITOR:
  1 = HOT WORK
  0 = STANDBY
  9 = EMERGENCY STOP
  r = clear FAULT
  a = test: pretend A2 confirmed fire
  b = test: pretend A3 ready
*/

#include <Wire.h>


// ============================================================
// I2C ADDRESSES
// ============================================================

const byte A2_ADDR = 8;
const byte A3_ADDR = 9;

// FALSE = use the real A3 robot
const bool SIMULATE_A3 = false;


// ============================================================
// COMMANDS TO A2
// ============================================================

const byte CMD_A2_HOTWORK       = 1;
const byte CMD_A2_VERIFY_TARGET = 2;
const byte CMD_A2_STANDBY       = 3;


// ============================================================
// STATUS FROM A2
// ============================================================

const byte A2_NO_FIRE        = 0;
const byte A2_POSSIBLE_FIRE  = 1;
const byte A2_FIRE_CONFIRMED = 2;
const byte A2_VERIFYING      = 3;
const byte A2_STANDBY        = 4;

const byte A2_REPLY_LENGTH = 9;


// ============================================================
// COMMANDS TO A3
// ============================================================

const byte CMD_A3_STANDBY        = 10;
const byte CMD_A3_TARGET         = 11;
const byte CMD_A3_SUPPRESS_START = 12;
const byte CMD_A3_SUPPRESS_STOP  = 13;


// ============================================================
// STATUS FROM A3
// ============================================================

const byte A3_PARKED               = 20;
const byte A3_READY                = 21;
const byte A3_SUPPRESSING          = 22;
const byte A3_SUPPRESSION_COMPLETE = 23;


// ============================================================
// INFORMATION RECEIVED FROM A2
// ============================================================

int16_t targetX = 0;
int16_t targetY = 0;

float cvConfidence   = 0.0;
float fireConfidence = 0.0;


// ============================================================
// SYSTEM STATES
// ============================================================

enum SystemState {
  STANDBY,
  HOT_WORK,
  TARGETING,
  VERIFYING,

  // NEW:
  // A2 stays stopped while A3 physically returns to centre.
  // HOT_WORK cannot restart until A3 reports PARKED.
  RETURNING_TO_PARK,

  FIRE_CONFIRMED,
  SUPPRESSING,
  RECHECK,
  FAULT
};

SystemState currentState = STANDBY;


// ============================================================
// PINS
// ============================================================

const int PIN_HOTWORK_BUTTON = 6;
const int PIN_PUMP           = 7;
const int PIN_RED_LED        = 8;
const int PIN_YELLOW_LED     = 9;
const int PIN_ALARM          = 10;


// ============================================================
// BUTTON VARIABLES
// ============================================================

bool lastButtonState = HIGH;
unsigned long lastButtonPress = 0;

const unsigned long BUTTON_DEBOUNCE = 300;


// ============================================================
// TIMING
// ============================================================

const unsigned long POLL_INTERVAL    = 500;
const unsigned long SUPPRESSION_TIME = 20000;
const unsigned long STEP_TIMEOUT     = 10000;

const byte MAX_MISSED_REPLIES = 3;


// ============================================================
// ALARM
// ============================================================

const int FIRE_ALARM_TONE  = 2000;
const int FAULT_ALARM_TONE = 800;


// ============================================================
// SERIAL PRINTING
// ============================================================

const bool PRINT_EVERY_POLL = false;


// ============================================================
// VARIABLES
// ============================================================

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


// ============================================================
// TEST VARIABLES
// ============================================================

bool testA2Confirmed = false;
bool testA3Ready     = false;

byte simA3LastCommand = CMD_A3_STANDBY;
unsigned long simA3AimStart = 0;


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(9600);

  Wire.begin();
  Wire.setWireTimeout(25000, true);

  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_RED_LED, OUTPUT);
  pinMode(PIN_YELLOW_LED, OUTPUT);
  pinMode(PIN_ALARM, OUTPUT);

  pinMode(PIN_HOTWORK_BUTTON, INPUT_PULLUP);

  digitalWrite(PIN_PUMP, LOW);
  digitalWrite(PIN_RED_LED, LOW);
  digitalWrite(PIN_YELLOW_LED, LOW);

  delay(1000);

  Serial.println(F("FIREWATCH A1 MISSION CONTROLLER"));
  Serial.println(F("Keys: 1=Hot Work  0=Standby  9=E-STOP  r=clear fault"));
  Serial.println(F("Test: a=pretend A2 confirmed  b=pretend A3 ready/done"));
  Serial.println(F("Physical button: STANDBY <-> HOT WORK"));

  changeState(STANDBY);
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  handleHotWorkButton();
  handleSerialInput();
  runStateMachine();
  updateBeaconAndAlarm();
}


// ============================================================
// PHYSICAL HOT WORK BUTTON
// ============================================================

void handleHotWorkButton() {

  bool buttonState = digitalRead(PIN_HOTWORK_BUTTON);

  if (buttonState == LOW && lastButtonState == HIGH) {

    if (millis() - lastButtonPress > BUTTON_DEBOUNCE) {

      lastButtonPress = millis();

      if (currentState == STANDBY) {

        Serial.println(F("BUTTON: HOT WORK MODE selected"));
        changeState(HOT_WORK);

      }

      else if (currentState == HOT_WORK) {

        Serial.println(F("BUTTON: STANDBY selected"));
        changeState(STANDBY);

      }

      else {

        Serial.println(F("BUTTON ignored - system is handling a fire/event"));

      }
    }
  }

  lastButtonState = buttonState;
}


// ============================================================
// SERIAL INPUT
// ============================================================

void handleSerialInput() {

  if (!Serial.available()) return;

  char key = Serial.read();

  switch (key) {

    case '1':

      if (currentState == STANDBY) {

        Serial.println(F("OPERATOR: HOT WORK MODE selected"));
        changeState(HOT_WORK);

      }

      else {

        Serial.println(F("(Hot Work can only be started from STANDBY)"));

      }

      break;


    case '0':

      if (currentState == FAULT) {

        Serial.println(F("(In FAULT, press r to clear it first, or 9 for E-STOP)"));

      }

      else {

        Serial.println(F("OPERATOR: STANDBY selected"));
        changeState(STANDBY);

      }

      break;


    case '9':

      Serial.println(F("EMERGENCY STOP"));
      changeState(STANDBY);
      break;


    case 'r':

      if (currentState == FAULT) {

        Serial.println(F("OPERATOR: fault cleared"));
        changeState(STANDBY);

      }

      else {

        Serial.println(F("(no fault to clear)"));

      }

      break;


    case 'a':

      testA2Confirmed = true;
      Serial.println(F("TEST pretending A2 = FIRE_CONFIRMED"));
      break;


    case 'b':

      testA3Ready = true;
      Serial.println(F("TEST pretending A3 = READY or DONE"));
      break;


    case '\n':
    case '\r':

      break;


    default:

      Serial.print(F("(unknown key: "));
      Serial.print(key);
      Serial.println(F(")"));
      break;
  }
}


// ============================================================
// CHANGE STATE
// ============================================================

void changeState(SystemState newState) {

  Serial.println(F("____________________________________________"));
  Serial.print(F(" STATE: "));
  Serial.print(stateName(currentState));
  Serial.print(F(" -> "));
  Serial.println(stateName(newState));
  Serial.println(F("____________________________________________"));

  currentState = newState;
  stateStartTime = millis();
  lastPollTime = millis() - POLL_INTERVAL;
  missedReplies = 0;

  switch (newState) {

    case STANDBY:

      digitalWrite(PIN_PUMP, LOW);

      alarmTone = 0;
      suppressionCycles = 0;

      sendCommand(A3_ADDR, CMD_A3_STANDBY);
      sendCommand(A2_ADDR, CMD_A2_STANDBY);

      break;


    case HOT_WORK:

      digitalWrite(PIN_PUMP, HIGH);

      alarmTone = 0;
      suppressionCycles = 0;

      // A3 should already be parked before we enter HOT_WORK.
      sendCommand(A3_ADDR, CMD_A3_STANDBY);

      if (!sendCommand(A2_ADDR, CMD_A2_HOTWORK)) {

        enterFault(F("A2 did not accept HOT_WORK"));
        return;

      }

      break;


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


    case VERIFYING:

      testA2Confirmed = false;

      if (!sendCommand(A2_ADDR, CMD_A2_VERIFY_TARGET)) {

        enterFault(F("A2 did not accept VERIFY_TARGET"));
        return;

      }

      break;


    // --------------------------------------------------------
    // NEW: RETURNING TO PARK
    // --------------------------------------------------------

    case RETURNING_TO_PARK:

      /*
        IMPORTANT:

        Stop A2 computer vision BEFORE telling A3 to return.

        A2 must not detect and pass another possible fire to A1
        while the camera/robot is still moving back to centre.

        A1 will remain in RETURNING_TO_PARK until A3 physically
        reaches centre and reports A3_PARKED.
      */

      Serial.println(
        F("  Stopping CV while A3 returns to centre")
      );

      if (!sendCommand(A2_ADDR, CMD_A2_STANDBY)) {

        enterFault(F("A2 did not accept STANDBY"));
        return;

      }

      if (!sendCommand(A3_ADDR, CMD_A3_STANDBY)) {

        enterFault(F("A3 did not accept STANDBY"));
        return;

      }

      break;


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


    case SUPPRESSING:

      lastCountdownPrint = millis();
      break;


    case RECHECK:

      recheckVerifySent = false;
      testA2Confirmed = false;
      testA3Ready = false;

      if (!sendCommand(A3_ADDR, CMD_A3_SUPPRESS_STOP)) {

        enterFault(F("A3 did not accept SUPPRESS_STOP"));
        return;

      }

      break;


    case FAULT:

      digitalWrite(PIN_PUMP, LOW);

      alarmTone = FAULT_ALARM_TONE;

      sendCommand(A3_ADDR, CMD_A3_STANDBY);
      sendCommand(A2_ADDR, CMD_A2_STANDBY);

      Serial.println(F("  Pump OFF. Check wiring, then press r."));

      break;
  }
}


// ============================================================
// ENTER FAULT
// ============================================================

void enterFault(const __FlashStringHelper* reason) {

  Serial.print(F("FAULT: "));
  Serial.println(reason);

  changeState(FAULT);
}


// ============================================================
// STATE MACHINE
// ============================================================

void runStateMachine() {

  switch (currentState) {

    // --------------------------------------------------------
    // STANDBY
    // --------------------------------------------------------

    case STANDBY:

      break;


    // --------------------------------------------------------
    // HOT WORK
    // --------------------------------------------------------

    case HOT_WORK:

      if (!timeToPoll())
        break;

      if (!readA2())
        break;

      if (a2Status == A2_POSSIBLE_FIRE) {

        changeState(TARGETING);

      }

      break;


    // --------------------------------------------------------
    // TARGETING
    // --------------------------------------------------------

    case TARGETING:

      if (testA3Ready) {

        changeState(VERIFYING);
        break;

      }

      if (stateTimedOut()) {

        enterFault(F("A3 never reported READY"));
        break;

      }

      if (!timeToPoll())
        break;

      if (!readA3())
        break;

      if (a3Status == A3_READY) {

        Serial.println(F("  A3 is aimed - asking A2 to verify"));
        changeState(VERIFYING);

      }

      break;


    // --------------------------------------------------------
    // VERIFYING
    // --------------------------------------------------------

    case VERIFYING:

      if (testA2Confirmed) {

        changeState(FIRE_CONFIRMED);
        break;

      }

      // If A2 fails to finish verification, stop CV and
      // return the robot to centre before monitoring again.
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


      // Fire confirmed
      if (a2Status == A2_FIRE_CONFIRMED) {

        changeState(FIRE_CONFIRMED);

      }


      // Fire NOT confirmed
      else if (a2Status == A2_NO_FIRE) {

        Serial.println(
          F("  Fire NOT confirmed - stopping CV and returning robot to centre")
        );

        changeState(RETURNING_TO_PARK);

      }

      break;


    // --------------------------------------------------------
    // NEW: RETURNING TO PARK
    // --------------------------------------------------------

    case RETURNING_TO_PARK:

      /*
        A2 is currently in STANDBY, so CV is not allowed to
        generate another possible-fire event.

        Wait here until A3 tells us that the servos have
        physically reached the parked position.
      */

      if (stateTimedOut()) {

        enterFault(F("A3 did not return to PARKED"));
        break;

      }

      if (!timeToPoll())
        break;

      if (!readA3())
        break;


      if (a3Status == A3_PARKED) {

        Serial.println(
          F("  A3 physically PARKED - restarting fire monitoring")
        );

        changeState(HOT_WORK);

      }

      break;


    // --------------------------------------------------------
    // FIRE CONFIRMED
    // --------------------------------------------------------

    case FIRE_CONFIRMED:

      changeState(SUPPRESSING);
      break;


    // --------------------------------------------------------
    // SUPPRESSING
    // --------------------------------------------------------

    case SUPPRESSING: {

      unsigned long elapsed = millis() - stateStartTime;

      if (elapsed >= SUPPRESSION_TIME) {

        Serial.println(F("  20 s complete - stopping water"));

        changeState(RECHECK);
        break;

      }

      if (millis() - lastCountdownPrint >= 1000) {

        lastCountdownPrint = millis();

        Serial.print(F("  Suppressing... "));
        Serial.print((SUPPRESSION_TIME - elapsed) / 1000);
        Serial.println(F(" s left"));

      }

      if (timeToPoll()) {

        readA3();

      }

      break;
    }


    // --------------------------------------------------------
    // RECHECK
    // --------------------------------------------------------

    case RECHECK:

      if (stateTimedOut()) {

        enterFault(F("recheck did not finish"));
        break;

      }

      // Wait for A3 to confirm suppression has stopped.
      if (!recheckVerifySent) {

        if (!testA3Ready) {

          if (!timeToPoll())
            break;

          if (!readA3())
            break;

          if (a3Status != A3_SUPPRESSION_COMPLETE)
            break;

        }

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


      if (testA2Confirmed) {

        changeState(FIRE_CONFIRMED);
        break;

      }


      if (!timeToPoll())
        break;

      if (!readA2())
        break;


      if (a2Status == A2_FIRE_CONFIRMED) {

        Serial.println(F("  STILL BURNING - suppressing again"));

        changeState(FIRE_CONFIRMED);

      }

      else if (a2Status == A2_NO_FIRE) {

        Serial.println(
          F("  FIRE OUT - returning robot to centre")
        );

        // Do not restart HOT_WORK until the camera is centred.
        changeState(RETURNING_TO_PARK);

      }

      break;


    // --------------------------------------------------------
    // FAULT
    // --------------------------------------------------------

    case FAULT:

      break;
  }
}


// ============================================================
// I2C SEND COMMAND
// ============================================================

bool sendCommand(byte addr, byte cmd) {

  if (SIMULATE_A3 && cmd >= CMD_A3_STANDBY) {

    simulateA3Command(cmd);
    return true;

  }

  Wire.beginTransmission(addr);
  Wire.write(cmd);

  byte result = Wire.endTransmission();

  Serial.print(F("  A1 -> "));
  Serial.print(boardName(addr));
  Serial.print(F(": "));
  Serial.print(cmd);
  Serial.print(F(" ("));
  Serial.print(commandName(cmd));
  Serial.print(F(")"));

  if (result != 0) {

    Serial.print(F("   < NOT RECEIVED (error "));
    Serial.print(result);
    Serial.print(F(")"));

  }

  Serial.println();

  return result == 0;
}


// ============================================================
// SEND TARGET TO A3
// ============================================================

bool sendTargetToA3() {

  if (SIMULATE_A3) {

    Serial.print(F("  [SIM] X="));
    Serial.print(targetX);
    Serial.print(F(" Y="));
    Serial.print(targetY);
    Serial.print(F("  "));

    simulateA3Command(CMD_A3_TARGET);

    return true;
  }

  Wire.beginTransmission(A3_ADDR);

  Wire.write(CMD_A3_TARGET);
  Wire.write((byte*)&targetX, sizeof(targetX));
  Wire.write((byte*)&targetY, sizeof(targetY));

  byte result = Wire.endTransmission();

  Serial.print(F("  A1 -> A3: "));
  Serial.print(CMD_A3_TARGET);
  Serial.print(F(" (TARGET) X="));
  Serial.print(targetX);
  Serial.print(F(" Y="));
  Serial.print(targetY);

  if (result != 0) {

    Serial.print(F("   < NOT RECEIVED (error "));
    Serial.print(result);
    Serial.print(F(")"));

  }

  Serial.println();

  return result == 0;
}


// ============================================================
// READ A2
// ============================================================

bool readA2() {

  byte received =
    Wire.requestFrom(A2_ADDR, A2_REPLY_LENGTH);

  if (received == 0) {

    replyMissed(A2_ADDR);
    return false;

  }

  missedReplies = 0;

  byte newStatus = Wire.read();

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

  else if (newStatus == A2_FIRE_CONFIRMED) {

    Wire.readBytes(
      (byte*)&fireConfidence,
      sizeof(fireConfidence)
    );

  }


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

  a2Status = newStatus;

  return true;
}


// ============================================================
// READ A3
// ============================================================

bool readA3() {

  byte newStatus;

  if (SIMULATE_A3) {

    newStatus = simulatedA3Status();

  }

  else {

    byte received =
      Wire.requestFrom(A3_ADDR, (byte)1);

    if (received == 0) {

      replyMissed(A3_ADDR);
      return false;

    }

    newStatus = Wire.read();
  }

  missedReplies = 0;

  if (newStatus != a3Status || PRINT_EVERY_POLL) {

    Serial.print(F("  A3 -> A1: "));
    Serial.print(newStatus);
    Serial.print(F(" ("));
    Serial.print(statusName(newStatus));
    Serial.println(F(")"));

  }

  a3Status = newStatus;

  return true;
}


// ============================================================
// MISSED REPLY
// ============================================================

void replyMissed(byte addr) {

  missedReplies++;

  Serial.print(F("  ? "));
  Serial.print(boardName(addr));
  Serial.print(F(" did not reply ("));
  Serial.print(missedReplies);
  Serial.print(F("/"));
  Serial.print(MAX_MISSED_REPLIES);
  Serial.println(F(")"));

  if (missedReplies >= MAX_MISSED_REPLIES) {

    enterFault(F("lost comms with a board"));

  }
}


// ============================================================
// SIMULATED A3
// ============================================================

void simulateA3Command(byte cmd) {

  simA3LastCommand = cmd;

  if (cmd == CMD_A3_TARGET) {

    simA3AimStart = millis();

  }

  Serial.print(F("  [SIM] A1 -> A3: "));
  Serial.print(cmd);
  Serial.print(F(" ("));
  Serial.print(commandName(cmd));
  Serial.println(F(")"));
}


// ============================================================
// SIMULATED A3 STATUS
// ============================================================

byte simulatedA3Status() {

  switch (simA3LastCommand) {

    case CMD_A3_TARGET:

      if (millis() - simA3AimStart >= 2000)
        return A3_READY;

      return A3_PARKED;


    case CMD_A3_SUPPRESS_START:

      return A3_SUPPRESSING;


    case CMD_A3_SUPPRESS_STOP:

      return A3_SUPPRESSION_COMPLETE;


    default:

      return A3_PARKED;
  }
}


// ============================================================
// POLLING TIMER
// ============================================================

bool timeToPoll() {

  if (millis() - lastPollTime < POLL_INTERVAL)
    return false;

  lastPollTime = millis();

  return true;
}


// ============================================================
// STATE TIMEOUT
// ============================================================

bool stateTimedOut() {

  return millis() - stateStartTime >= STEP_TIMEOUT;
}


// ============================================================
// BEACONS AND ALARM
// ============================================================

void updateBeaconAndAlarm() {

  bool flashOn = (millis() / 250) % 2;

  switch (currentState) {

    case STANDBY:
    case FAULT:

      digitalWrite(PIN_YELLOW_LED, LOW);
      digitalWrite(PIN_RED_LED, LOW);

      break;


    case HOT_WORK:

      digitalWrite(PIN_YELLOW_LED, HIGH);
      digitalWrite(PIN_RED_LED, LOW);

      break;


    case TARGETING:
    case VERIFYING:
    case RETURNING_TO_PARK:

      digitalWrite(PIN_YELLOW_LED, flashOn);
      digitalWrite(PIN_RED_LED, LOW);

      break;


    case FIRE_CONFIRMED:
    case SUPPRESSING:
    case RECHECK:

      digitalWrite(PIN_YELLOW_LED, flashOn);
      digitalWrite(PIN_RED_LED, flashOn);

      break;
  }


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


// ============================================================
// COMMAND NAMES
// ============================================================

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


// ============================================================
// STATUS NAMES
// ============================================================

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


// ============================================================
// BOARD NAMES
// ============================================================

const __FlashStringHelper* boardName(byte addr) {

  if (addr == A2_ADDR)
    return F("A2");

  if (addr == A3_ADDR)
    return F("A3");

  return F("??");
}


// ============================================================
// STATE NAMES
// ============================================================

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