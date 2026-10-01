/* Firewatch Arduino 1 mission contoller - annie 
role of A1: 
-i2c contoller
-runs the state machine
-polls A2 (perception of the fire) and A3 (robotics and pump ect)
-beacon/alarm emerg system 
-prints whats happening to serial monitor as the sequence is happening

settup + wireing:

-SDA to A4, SCL to A5
-GND shared all boards on bus (for i3c to work needs common ground ref not just sda and scl)
-beacon led is PIN 8 
-alarm piezo is PIN 9
-pump relay is  PIN 7

serial monitor commands

1 = operator selects HOT WORK MODE
0 = operator selects STANDBY
9 = EMERGENCY STOP (shut down from any of the states)
r = clear a FAULT (if there is a coms failure) returns to STANDBY
a = (self test) pretend A2 has confirmed a fire
b = (self test) pretend that A3 is ready 


*/

#include <Wire.h>

// I2C addresses (matching other boards wire.begin(x))
const byte A2_ADDR = 8; // perception (saf) using Wire.begin(8) 
const byte A3_ADDR = 9; // robotics/pump (devlan) BUT CHANGE BACK TO 9 8 is only for one arduno test
const bool SIMULATE_A3 = true; // testing purposes - simulating A3 , true when testing


// commands sent TO A2 - matching safs sketch
const byte CMD_A2_HOTWORK = 1; // start the fire monitoring cv
const byte CMD_A2_VERIFY_TARGET = 2;// A3 in position and aimed, do the CV and thermal check
const byte CMD_A2_STANDBY = 3; // stop monitoring (fire out?)

// commands sent BACK from A2 - will always be the first byte of the reply 
const byte A2_NO_FIRE = 0; //monitoring nothing detected
const byte A2_POSSIBLE_FIRE = 1; // +X (2 bytes) + Y (2 bytes) + cvConfidence (4 bytes)
const byte A2_FIRE_CONFIRMED = 2; //FIRE confidence (4 bytes)
const byte A2_VERIFYING =3; // is still calculating the CV and thermal
const byte A2_STANDBY =4; // idle

const byte A2_REPLY_LENGTH = 9; // biggest reply is 1+2+2+4=9


//comands to be sent to A3 (draft got to confirm)
const byte CMD_A3_STANDBY = 10; //park laser off and valve closed 
const byte CMD_A3_TARGET = 11;// X (2bytes) + Y (2 bytes)
const byte CMD_A3_SUPPRESS_START = 12; // open/ laser on
const byte CMD_A3_SUPPRESS_STOP = 13; // close/laser off

//commands send BACK from A3 (draft atm)
const byte A3_PARKED = 20;
const byte A3_READY = 21; //aimed, laser on?
const byte A3_SUPPRESSING = 22;
const byte A3_SUPPRESSION_COMPLETE = 23;

//info recived from A2:
int16_t targetX = 0; // 2 bytes 
int16_t targetY = 0;

float cvConfidence = 0.0;
float fireConfidence = 0.0;

//states : 
enum SystemState {
  STANDBY,//waiting for the operator 
  HOT_WORK,//pump on A2 is monitoring 
  TARGETING,// possible fire found A3 is aiming
  VERIFYING, // A3 aimed and A2 doing CV and thermal checks
  FIRE_CONFIRMED, //alarm on, suppression sent and timer started
  SUPPRESSING, // valve open 20s time
  RECHECK,// suppression done A2 reverifies the same spot
  FAULT //coms failed
};

SystemState currentState = STANDBY;




// PINS
const int PIN_PUMP   = 7;  // pump relay control input only - the relay switches the 12V pump
const int PIN_BEACON = 8;  // beacon led through a resistor to GND
const int PIN_ALARM  = 9;  // piezo

// use millis instead of delay so loop never freezes/gets stuck
// That means A1 can always react to an emergency stop even mid-suppression
const unsigned long POLL_INTERVAL         = 500; // ask A2/A3 for status every 0.5 s
const unsigned long BEACON_FLASH_INTERVAL = 250;// beacon on/off speed flashingggg
const unsigned long SUPPRESSION_TIME      = 20000; // 20 seconds of "water" (if we doing pump) per cycle
const unsigned long STEP_TIMEOUT          = 10000;// max wait for A3 to aim / A2 to verify (in case not responding)
const byte MAX_MISSED_REPLIES = 3;// 3 missed polls in a row = fault

// BEEEEEEEEP BEEEEEEEEEP BEEEEEEEEEEEEEEP
const int FIRE_ALARM_TONE  = 2000; // Hz - fire alarm
const int FAULT_ALARM_TONE = 800;  // Hz - lower tone = fault not a fire 

//prining out the status every poll
// true  = print every single poll reply
// false = only print a reply when the status changes so its much easier to read
//can be changed if needed
const bool PRINT_EVERY_POLL = false;


// VARIBLES 

byte a2Status = A2_STANDBY; // latest status byte from A2
byte a3Status = A3_PARKED; //latest status byt from A3

unsigned long stateStartTime = 0; // what time current state (for timeouts + 20 s timer)
unsigned long lastPollTime = 0;
unsigned long lastBeaconToggle = 0;
unsigned long lastCountdownPrint = 0;

bool beaconFlashing = false;
bool beaconOn = false;
int alarmTone = 0;   // 0 = silent , all is good
int alarmTonePlaying = 0;// the tone actually playing right now, falut or fire

byte missedReplies = 0; // missed polls in a row
byte suppressionCycles = 0; // how many times this fire hase bee sprayed
bool recheckVerifySent = false; 

//just for testing
bool testA2Confirmed = false; // set by 'a' key
bool testA3Ready = false;  // set by 'b' key
byte simA3LastCommand = CMD_A3_STANDBY;  // the last command "A3" got
unsigned long simA3AimStart = 0;         //  when TARGET arrived

// SETTING UP

void setup() {
  Serial.begin(9600);
  Wire.begin(); // no address given = A1 is the I2C controller
  Wire.setWireTimeout(25000, true);  // if the bus locks up, for many resons like a loose wire ect, give up after 25 ms, instead of freezing the whole Arduino forever :(

  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_BEACON, OUTPUT);
  pinMode(PIN_ALARM, OUTPUT);
  digitalWrite(PIN_PUMP, LOW);
  digitalWrite(PIN_BEACON, LOW);

  delay(1000); //give A2 and A3 a second to wake up and say hello

  //showing commands for serial monitor
  Serial.println(F("FIREWATCH A1 MISSION CONTROLLER"));
  Serial.println(F("Keys: 1=Hot Work  0=Standby  9=E-STOP  r=clear fault"));
  Serial.println(F("Test: a=pretend A2 confirmed  b=pretend A3 ready/done"));

  changeState(STANDBY);  // also tells A2 + A3 to go to standby so all 3 boards agreee when booted up
}


// MAIN LOOP
//three jobs to do, looping round
void loop(){
  handleSerialInput(); //was akey pressed? if so then act on it, if not return instantly
  runStateMachine(); //what does the current sate need, looks at current state and does a step for that state, like counting or polling A2 ect
  updateBeaconAndAlarm();// do the lights or sound need to be changed, changes when needed
}



// INPUT BY OPERATOR
// what is entered into the serial monitor + for testing

void handleSerialInput() {
  if (!Serial.available()) return;
  char key = Serial.read();

  switch (key) {
    case '1':
      if (currentState == STANDBY) {
        Serial.println(F("OPERATOR: HOT WORK MODE selected"));
        changeState(HOT_WORK);
      } else {
        Serial.println(F("(Hot Work can only be started from STANDBY)"));
      }
      break;

    case '0':
      if (currentState == FAULT) {
        Serial.println(F("(In FAULT, press r to clear it first, or 9 for E-STOP)"));
      } else {
        Serial.println(F("OPERATOR: STANDBY selected"));
        changeState(STANDBY);  // STANDBY closes the valve + stops pump, even mid-suppression
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
      } else {
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
      break;  // ignore the Enter key

    default:
      Serial.print(F("(unknown key: "));
      Serial.print(key);
      Serial.println(F(")"));
      break;
  }
}


// CHANGE STATES
//all state changes happens here, the changes are printed and the timers reset. does the first actions for the new state once

void changeState(SystemState newState) {
  Serial.println(F("______________________________________________")); 
  Serial.print(F(" STATE: "));
  Serial.print(stateName(currentState));
  Serial.print(F(" -> "));
  Serial.println(stateName(newState));
  Serial.println(F("______________________________________________"));

  currentState = newState;
  stateStartTime = millis();
  lastPollTime = millis() - POLL_INTERVAL;  // so the first poll happens straight away
  missedReplies = 0;


  switch (newState) {
    case STANDBY:
      digitalWrite(PIN_PUMP, LOW);
      beaconFlashing = false;
      alarmTone = 0;
      suppressionCycles = 0;
      sendCommand(A3_ADDR, CMD_A3_STANDBY);  // park, laser off, VALVE CLOSED
      sendCommand(A2_ADDR, CMD_A2_STANDBY);  // stop monitoring
      break;

    case HOT_WORK:
      digitalWrite(PIN_PUMP, HIGH);  // hose is charged the whole time hot work is happening
      beaconFlashing = false;
      alarmTone = 0;
      suppressionCycles = 0;
      sendCommand(A3_ADDR, CMD_A3_STANDBY);  // park A3 (matters when we come back from a fire)
      if (!sendCommand(A2_ADDR, CMD_A2_HOTWORK)) {
        enterFault(F("A2 did not accept HOT_WORK"));
        return;
      }
      break;

    case TARGETING:
      beaconFlashing = true;  // visual warning, something might be burning, robot moving
      alarmTone = 0;          // no siren yet, not confirmed
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
        enterFault(F("A2 did not acccept VERIFY_TARGET"));
        return;
      }
      break;

    case FIRE_CONFIRMED:
      beaconFlashing = true;
      alarmTone = FIRE_ALARM_TONE;  // nowthe siren goes because fire is confirmed !!!
      suppressionCycles++;
      Serial.print(F(" ATENTION!!! FIRE HAS BEEN CONFIRMED! the fire confidence = "));
      Serial.print(fireConfidence);
      Serial.print(F("  suppression cycle #")); // keeping track
      Serial.println(suppressionCycles);
      if (!sendCommand(A3_ADDR, CMD_A3_SUPPRESS_START)) {
        enterFault(F("A3 did not accept SUPPRESS_START"));
        return;
      }
      break;

    case SUPPRESSING:
      lastCountdownPrint = millis();
      break;

    case RECHECK: //if fire is still there
      recheckVerifySent = false;
      testA2Confirmed = false;
      testA3Ready = false;
      if (!sendCommand(A3_ADDR, CMD_A3_SUPPRESS_STOP)) {
        enterFault(F("A3 did not accept SUPPRESS_STOP"));
        return;
      }
      break;

    case FAULT:
      // unknown situation like a board stopped talking. pump off, wait for a human with the alarms goinf off for falult 
      digitalWrite(PIN_PUMP, LOW);
      beaconFlashing = true;
      alarmTone = FAULT_ALARM_TONE;
      sendCommand(A3_ADDR, CMD_A3_STANDBY);  // try to close the valve
      Serial.println(F("  Pump OFF. Check wiring, then press r."));
      break;
  }
}

void enterFault(const __FlashStringHelper* reason) {
  Serial.print(F("FAULT: "));
  Serial.println(reason);
  changeState(FAULT);
}


//STATE MACHINE
//runs each loop and each state does a quick check and then returns

void runStateMachine() {
  switch (currentState) { // looks at the current state and goes to the matching case and only that one runs

    case STANDBY:
      break;  // just waiting for the operator input sm for hot works
    //hotwork mode on
    case HOT_WORK:
      if (!timeToPoll()) break; // ! is if not time to poll yet
      if (!readA2()) break; // ask for status from A2 and if no answer leave , counts it incase there is a fault (after 3 missess)
      if (a2Status == A2_POSSIBLE_FIRE) {//if there is a possible fire, go to targeting, readA2 has the coords stored
        changeState(TARGETING);
      }
      break;
      
    // target in correct priority order
    case TARGETING:
      if (testA3Ready) { changeState(VERIFYING); break; } // for testing.
      if (stateTimedOut()) { enterFault(F("A3 never reported READY")); break; } //if 10 seconds have passed in targeting and no response
      if (!timeToPoll()) break; //polls every 0.5 seconds so if not time dont so it doesnt flood it
      if (!readA3()) break; // ask for status from A3
      if (a3Status == A3_READY) {
        Serial.println(F("  A3 is aimed - asking A2 to verify"));
        changeState(VERIFYING); //print message and move to next state
      }
      break;


    //pump a3 is aimed so thermal sensor is on fire
    //waiting for A2s confirming CV and thermal (both sensors together)
    case VERIFYING:
      if (testA2Confirmed) { changeState(FIRE_CONFIRMED); break; }
      if (stateTimedOut()) { enterFault(F("A2 never finished verifying")); break; } // 10 safty limit 
      if (!timeToPoll()) break;
      if (!readA2()) break;
      if (a2Status == A2_FIRE_CONFIRMED) { 
        changeState(FIRE_CONFIRMED); //real fire start the suppression
      } else if (a2Status == A2_NO_FIRE) {
        Serial.println(F("  Thermal did NOT confirm - false alarm, back to monitoring")); //not fire go back
        changeState(HOT_WORK);
      }
      // A2_VERIFYING /A2_POSSIBLE_FIRE = A2 hasn't finished yet, keep waiting
      break;

    // here to just set the alarm off and log that the fire was confirmed
    case FIRE_CONFIRMED:
      changeState(SUPPRESSING);  //move on
      break;

    // suppressing , the water is on. 
    case SUPPRESSING: {//{} because varible used
      unsigned long elapsed = millis() - stateStartTime; //mssince water started
      if (elapsed >= SUPPRESSION_TIME) {
        Serial.println(F("  20 s complete - stopping water"));
        changeState(RECHECK); //close the valve (or laser?)
        break;
      }
      if (millis() - lastCountdownPrint >= 1000) { // printing the countdowns per second out on neat lines
        lastCountdownPrint = millis();
        Serial.print(F("  Suppressing... "));
        Serial.print((SUPPRESSION_TIME - elapsed) / 1000);
        Serial.println(F(" s left"));
      }
      if (timeToPoll()) readA3();  // keep checking a3 is still alive and well while water flows
      break;
    }

    //water has stoped (or laser?) checking if fire is actualy out or not
    case RECHECK:
      if (stateTimedOut()) { enterFault(F("recheck did not finish")); break; } //10sec safty lim

      // wait for A3 to say the valve is closed pump is done
      if (!recheckVerifySent) {
        if (!testA3Ready) { //b to skip
          if (!timeToPoll()) break;
          if (!readA3()) break;
          if (a3Status != A3_SUPPRESSION_COMPLETE) break; // not off/closed yet still wait
        }
        //
        Serial.println(F("  Valve closed - asking A2 to re-verify the same spot"));
        //if A3 still aimed at the same spot:
        if (!sendCommand(A2_ADDR, CMD_A2_VERIFY_TARGET)) {
          enterFault(F("A2 did not accept VERIFY_TARGET"));
          break;
        }
        recheckVerifySent = true; //the varify has been sent then go to phase 2
        break;
      }

      // is the fire still alive? is there still a fire to put out?
      if (testA2Confirmed) { changeState(FIRE_CONFIRMED); break; } //a key
      if (!timeToPoll()) break;
      if (!readA2()) break;
      //still aimed so no need to target 
      if (a2Status == A2_FIRE_CONFIRMED) {
        Serial.println(F("  STILL BURNING - suppressing again"));
        changeState(FIRE_CONFIRMED);  // A3 is still aimed, so skip TARGETING
      } else if (a2Status == A2_NO_FIRE) {
        Serial.println(F("  FIRE OUT - back to Hot Work monitoring")); //parks 
        changeState(HOT_WORK);
      }
      break;

    case FAULT:
      break;  // waits for the operator to press r (or 9)
  }
}



// I2C SENDING

bool sendCommand(byte addr, byte cmd) {
  if (SIMULATE_A3 && cmd >= CMD_A3_STANDBY) {  // A3 commands are 10-13
    simulateA3Command(cmd);
    return true;                               // pretend A3 heard it
  }
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  byte result = Wire.endTransmission();  // 0 = success, anything else = nobody answered

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

// Sends TARGET + X (2 bytes) + Y (2 bytes) = 5 bytes to A3
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



/// I2C READING
//asks for status and data and returns false if A2 doesnt answer
// Asks A2 for its status (+ data). Returns false if A2 didn't answer.
bool readA2() {
  byte received = Wire.requestFrom(A2_ADDR, A2_REPLY_LENGTH);
  if (received == 0) {
    replyMissed(A2_ADDR);
    return false;
  }
  missedReplies = 0;

  // NOTE: we always get 9 bytes back - if A2 only sent 1, the Uno fills the
  // rest with 255. So we only read the extra bytes the status says are real.
  byte newStatus = Wire.read();

  if (newStatus == A2_POSSIBLE_FIRE) {
    Wire.readBytes((byte*)&targetX, sizeof(targetX));
    Wire.readBytes((byte*)&targetY, sizeof(targetY));
    Wire.readBytes((byte*)&cvConfidence, sizeof(cvConfidence));
  } else if (newStatus == A2_FIRE_CONFIRMED) {
    Wire.readBytes((byte*)&fireConfidence, sizeof(fireConfidence));
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
    } else if (newStatus == A2_FIRE_CONFIRMED) {
      Serial.print(F("  fireConfidence="));
      Serial.print(fireConfidence);
    }
    Serial.println();
  }

  a2Status = newStatus;
  return true;
}

// Asks A3 for its 1-byte status. Returns false if A3 didn't answer.
// Asks A3 for its 1-byte status. Returns false if A3 didn't answer.
bool readA3() {
  byte newStatus;

  if (SIMULATE_A3) {
    newStatus = simulatedA3Status();   // pretend A3 answered
  } else {
    byte received = Wire.requestFrom(A3_ADDR, (byte)1);
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

// One missed reply could just be noise. Three in a row = that board is gone.
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


// extra to help with alarm system and keeping track of timers

// SIMULATED A3 (testing only)
// Remembers the last command sent to A3 and prints it like a real send
void simulateA3Command(byte cmd) {
  simA3LastCommand = cmd;
  if (cmd == CMD_A3_TARGET) simA3AimStart = millis();
  Serial.print(F("  [SIM] A1 -> A3: "));
  Serial.print(cmd);
  Serial.print(F(" ("));
  Serial.print(commandName(cmd));
  Serial.println(F(")"));
}

// Works out what a real A3 would answer, based on the last command.
byte simulatedA3Status() {
  switch (simA3LastCommand) {
    case CMD_A3_TARGET:
      if (millis() - simA3AimStart >= 2000) return A3_READY;  // "aiming" takes 2 s
      return A3_PARKED;
    case CMD_A3_SUPPRESS_START: return A3_SUPPRESSING;
    case CMD_A3_SUPPRESS_STOP:  return A3_SUPPRESSION_COMPLETE;
    default:                    return A3_PARKED;
  }
}


bool timeToPoll() {
  if (millis() - lastPollTime < POLL_INTERVAL) return false;
  lastPollTime = millis();
  return true;
}

bool stateTimedOut() {
  return millis() - stateStartTime >= STEP_TIMEOUT;
}

void updateBeaconAndAlarm() {
  // Beacon: flash without delay()
  if (beaconFlashing) {
    if (millis() - lastBeaconToggle >= BEACON_FLASH_INTERVAL) {
      lastBeaconToggle = millis();
      beaconOn = !beaconOn;
      digitalWrite(PIN_BEACON, beaconOn);
    }
  } else if (beaconOn) {
    beaconOn = false;
    digitalWrite(PIN_BEACON, LOW);
  }

  // Alarm: only call tone()/noTone() when it needs to CHANGE
  // (calling tone() every loop restarts it and it sounds glitchy)
  if (alarmTone != alarmTonePlaying) {
    if (alarmTone == 0) noTone(PIN_ALARM);
    else tone(PIN_ALARM, alarmTone);
    alarmTonePlaying = alarmTone;
  }
}


// NAMES TO PRINT TO SERIAL MONITOR
//commands and replys stay in seperate lists because some numbers reused in diffrent directions

const __FlashStringHelper* commandName(byte code) {
  switch (code) {
    case 1:  return F("HOT_WORK");
    case 2:  return F("VERIFY_TARGET");
    case 3:  return F("STANDBY");
    case 10: return F("STANDBY");
    case 11: return F("TARGET");
    case 12: return F("SUPPRESS_START");
    case 13: return F("SUPPRESS_STOP");
    default: return F("UNKNOWN");
  }
}

const __FlashStringHelper* statusName(byte code) {
  switch (code) {
    case 0:  return F("NO_FIRE");
    case 1:  return F("POSSIBLE_FIRE");
    case 2:  return F("FIRE_CONFIRMED");
    case 3:  return F("VERIFYING");
    case 4:  return F("STANDBY");
    case 20: return F("PARKED");
    case 21: return F("READY");
    case 22: return F("SUPPRESSING");
    case 23: return F("SUPPRESSION_COMPLETE");
    case 255: return F("NO DATA");
    default: return F("UNKNOWN");
  }
}

const __FlashStringHelper* boardName(byte addr) {
  if (addr == A2_ADDR) return F("A2");
  if (addr == A3_ADDR) return F("A3");
  return F("??");
}

const __FlashStringHelper* stateName(SystemState s) {
  switch (s) {
    case STANDBY:        return F("STANDBY");
    case HOT_WORK:       return F("HOT_WORK");
    case TARGETING:      return F("TARGETING");
    case VERIFYING:      return F("VERIFYING");
    case FIRE_CONFIRMED: return F("FIRE_CONFIRMED");
    case SUPPRESSING:    return F("SUPPRESSING");
    case RECHECK:        return F("RECHECK");
    case FAULT:          return F("FAULT");
  }
  return F("?");
}





