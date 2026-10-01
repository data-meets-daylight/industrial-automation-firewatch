
/* firewatch testing for monitor

pretends to be both A2 and A3

wireing is:

A4 to A4 (SDA)
A5 to A5 (SCL)

GND to GND (must share)

serial monitor for this bord 115200 baud
f = fire appears - A2 reporting possible fire
x fire goes out - no fire

*/


#include <Wire.h>

const byte FAKE_ADDR = 8;  // answers on 8 (A2) AND 9 (A3) - see setup()

// same numbers as A1
const byte CMD_A2_HOTWORK = 1;
const byte CMD_A2_VERIFY_TARGET = 2;
const byte CMD_A2_STANDBY = 3;

const byte A2_NO_FIRE = 0;
const byte A2_POSSIBLE_FIRE = 1;
const byte A2_FIRE_CONFIRMED = 2;
const byte A2_STANDBY = 4;

const byte CMD_A3_STANDBY = 10;
const byte CMD_A3_TARGET = 11;
const byte CMD_A3_SUPPRESS_START = 12;
const byte CMD_A3_SUPPRESS_STOP = 13;

const byte A3_PARKED = 20;
const byte A3_READY = 21;
const byte A3_SUPPRESSING = 22;
const byte A3_SUPPRESSION_COMPLETE = 23;

const unsigned long FAKE_AIM_TIME = 2000;  // pretend the servos take 2 s to aim

// Fake fire data , Saf's A2 would send
int16_t fakeX = 320;
int16_t fakeY = 240;
float fakeCvConfidence = 0.86;
float fakeFireConfidence = 0.93;

// Which board we are pretending to be  set by the last command A1 sent, A1 only ever polls the board it just talked to 
enum FakeRole {
  A2_IN_STANDBY,
  A2_MONITORING,
  A2_VERIFYING_TARGET,
  A3_IS_PARKED,
  A3_IS_AIMING,
  A3_IS_SUPPRESSING,
  A3_IS_DONE
};

volatile FakeRole role = A2_IN_STANDBY;
volatile bool fireVisible = false;
volatile unsigned long aimStartTime = 0;

// for printing outside the interrupt
volatile bool newCommand = false;
volatile byte lastCommand = 0;
volatile int16_t receivedX = 0;
volatile int16_t receivedY = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin(FAKE_ADDR);

  // Address mask, ignore the lowest address bit, so we answer BOTH 8 and 9.
  // (8 and 9 only differ in that one bit.)
  TWAMR = (1 << TWAM0);

  Wire.onReceive(receiveFromA1);
  Wire.onRequest(replyToA1);

  Serial.println(F("=== FIREWATCH TEST PARTNER (fake A2 + A3) ==="));
  Serial.println(F("Keys: f = fire appears   x = fire goes out"));
}

void loop() {
  // Print what A1 sent
  if (newCommand) {
    newCommand = false;
    Serial.print(F("A1 sent "));
    Serial.print(lastCommand);
    Serial.print(F(" -> "));
    switch (lastCommand) {
      case CMD_A2_HOTWORK:        Serial.println(F("[as A2] HOT_WORK - monitoring")); break;
      case CMD_A2_VERIFY_TARGET:  Serial.println(fireVisible ? F("[as A2] VERIFY - hot! will say FIRE_CONFIRMED")
      : F("[as A2] VERIFY - cold, will say NO_FIRE")); break;
      case CMD_A2_STANDBY:        Serial.println(F("[as A2] STANDBY")); break;
      case CMD_A3_STANDBY:        Serial.println(F("[as A3] STANDBY - parked, valve closed")); break;
      case CMD_A3_TARGET:
        Serial.print(F("[as A3] TARGET X="));
        Serial.print(receivedX);
        Serial.print(F(" Y="));
        Serial.print(receivedY);
        Serial.println(F("  (aiming for 2 s...)"));
        break;
      case CMD_A3_SUPPRESS_START: Serial.println(F("[as A3] SUPPRESS_START - valve OPEN")); break;
      case CMD_A3_SUPPRESS_STOP:  Serial.println(F("[as A3] SUPPRESS_STOP - valve CLOSED")); break;
      default:                    Serial.println(F("unknown command")); break;
    }
  }

  // Keyboard
  if (Serial.available()) {
    char key = Serial.read();
    if (key == 'f') { fireVisible = true;  Serial.println(F(">>> FIRE APPEARS (lighter on)")); }
    if (key == 'x') { fireVisible = false; Serial.println(F(">>> FIRE GOES OUT")); }
  }
}

// Runs automatically when A1 SENDS us something 
void receiveFromA1(int numBytes) {
  byte cmd = Wire.read();

  if (cmd == CMD_A3_TARGET && Wire.available() >= 4) {
    byte xLow = Wire.read();
    byte xHigh = Wire.read();
    byte yLow = Wire.read();
    byte yHigh = Wire.read();
    receivedX = (int16_t)(xLow | (xHigh << 8));
    receivedY = (int16_t)(yLow | (yHigh << 8));
  }
  while (Wire.available()) Wire.read();  // throw away anything extra

  switch (cmd) {
    case CMD_A2_HOTWORK:        role = A2_MONITORING; break;
    case CMD_A2_VERIFY_TARGET:  role = A2_VERIFYING_TARGET; break;
    case CMD_A2_STANDBY:        role = A2_IN_STANDBY; break;
    case CMD_A3_STANDBY:        role = A3_IS_PARKED; break;
    case CMD_A3_TARGET:         role = A3_IS_AIMING; aimStartTime = millis(); break;
    case CMD_A3_SUPPRESS_START: role = A3_IS_SUPPRESSING; break;
    case CMD_A3_SUPPRESS_STOP:  role = A3_IS_DONE; break;
  }

  lastCommand = cmd;
  newCommand = true;
}

// Runs automatically when A1 ASKS us for data 
void replyToA1() {
  byte reply[9];
  byte length = 1;

  switch (role) {
    case A2_IN_STANDBY:
      reply[0] = A2_STANDBY;
      break;

    case A2_MONITORING:
      if (fireVisible) {  // same 9-byte layout as Saf's sendToA1()
        reply[0] = A2_POSSIBLE_FIRE;
        memcpy(&reply[1], &fakeX, 2);
        memcpy(&reply[3], &fakeY, 2);
        memcpy(&reply[5], &fakeCvConfidence, 4);
        length = 9;
      } else {
        reply[0] = A2_NO_FIRE;
      }
      break;

    case A2_VERIFYING_TARGET:
      if (fireVisible) {
        reply[0] = A2_FIRE_CONFIRMED;
        memcpy(&reply[1], &fakeFireConfidence, 4);
        length = 5;
      } else {
        reply[0] = A2_NO_FIRE;
      }
      break;

    case A3_IS_PARKED:      reply[0] = A3_PARKED; break;
    case A3_IS_AIMING:      reply[0] = (millis() - aimStartTime >= FAKE_AIM_TIME) ? A3_READY : A3_PARKED; break;
    case A3_IS_SUPPRESSING: reply[0] = A3_SUPPRESSING; break;
    case A3_IS_DONE:        reply[0] = A3_SUPPRESSION_COMPLETE; break;
  }

  Wire.write(reply, length);
}