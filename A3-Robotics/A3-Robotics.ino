/*
=================================================================
INDUSTRIAL AUTOMATED FIREWATCH (IAF)
ICTE4005 Robotics Project - Group 5
Curtin University, 2026

Team:
Saf Flatters, Annie (Annabelle) Lewkowski, Devlin MacGlip

MODULE:
Arduino 3 (A3) - Robotics

PURPOSE:
A3 controls the movement and fire suppression components of the IAF system.
- Controls the pan and tilt servos
- Receives fire target coordinates from A1 (Mission Control)
- Converts target coordinates into servo angles
- Centres the webcam on the detected fire
- Activates the fire suppression system (mock water pump)
- Reports robot status back to A1

WIRING:

I2C Communication:
SDA (A4) -> A1 SDA
SCL (A5) -> A1 SCL
GND      -> Common GND between all Arduino boards

Pan Servo:
Signal -> Pin 9

Tilt Servo:
Signal -> Pin 10

Laser:
Signal -> Pin 7

Water Pump Indicator:
LED -> Pin 8

SERVO MOUNT LIMITS:
Pan Servo:
Safe minimum    = 20 degrees
Safe maximum    = 175 degrees
Parked position = 100 degrees
Tilt Servo:
Safe minimum    = 105 degrees
Safe maximum    = 180 degrees
Parked position = 155 degrees

IMPORTANT:
- Servo limits were physically tested on the Duinotech pan/tilt mount
  on 6 October 2026.
- Do not change the servo limits to 0-180 degrees without physically
  checking the mount.
- Servo movement is limited to 1 degree per step to prevent sudden
  or violent movement.
- PAN and TILT angles correspond to the physical servo angles
  established during testing.

A3 SYSTEM STATUS:
PARKED               = Robot at home position
READY                = Robot aimed at target position
SUPPRESSING          = Suppression system active
SUPPRESSION_COMPLETE = Suppression system stopped

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
#include <Servo.h>

// I2C ADDRESS
const byte A3_ADDRESS = 9;

// PIN DECLARATIONS
const int PIN_SERVO_X = 9;    // PAN
const int PIN_SERVO_Y = 10;   // TILT
const int PIN_LASER   = 7;
const int WATER_PUMP  = 8;

// TESTED HOME / CENTRE POSITION
const int PARK_X_ANGLE = 100;  // PAN centre
const int PARK_Y_ANGLE = 155;  // TILT centre

// TESTED SAFE SERVO LIMITS
// PAN
const int SERVO_X_MIN = 20;
const int SERVO_X_MAX = 175;
// TILT
const int SERVO_Y_MIN = 105;
const int SERVO_Y_MAX = 180;

// CAMERA MAPPING
const float IMG_WIDTH  = 640.0;
const float IMG_HEIGHT = 480.0;
const float IMG_CX = IMG_WIDTH / 2.0;
const float IMG_CY = IMG_HEIGHT / 2.0;
const float H_FOV = 60.0;
const float V_FOV = 60.0;

// SERVO MOTION CONTROL
// Move slowly to prevent sudden movement.
const unsigned long STEP_INTERVAL_MS = 40;
const int STEP_DEGREES = 1;

// TARGET HOLD TIME
// SEQ 07A - If a possible fire is not confirmed,
// hold the target position for 3 seconds before returning to PARKED.
const unsigned long TARGET_HOLD_TIME = 3000;
bool holdingTarget = false;
unsigned long targetHoldStart = 0;

// SERVOS
Servo servoX;
Servo servoY;

// I2C COMMANDS FROM A1
enum IncomingCommand : byte {
  CMD_NONE           = 0,
  CMD_STANDBY        = 10,
  CMD_TARGET         = 11,
  CMD_SUPPRESS_START = 12,
  CMD_SUPPRESS_STOP  = 13
};

// STATUS SENT TO A1
enum OutgoingStatus : byte {
  STATUS_PARKED               = 20,
  STATUS_READY                = 21,
  STATUS_SUPPRESSING          = 22,
  STATUS_SUPPRESSION_COMPLETE = 23
};

// I2C VARIABLES
volatile IncomingCommand pendingCommand = CMD_NONE;
volatile int16_t pendingX = 0;
volatile int16_t pendingY = 0;
volatile bool commandReady = false;

// CURRENT ROBOT STATUS
byte currentStatus = STATUS_PARKED;
int targetXAngle = PARK_X_ANGLE;
int targetYAngle = PARK_Y_ANGLE;
int currentXAngle = PARK_X_ANGLE;
int currentYAngle = PARK_Y_ANGLE;
bool isMoving = false;
unsigned long lastStepTime = 0;

// TRACK WHETHER WE HAVE AIMED AT A FIRE
bool hasTargetedFire = false;

// SETUP
// SEQ 01 - SYSTEM STARTUP / STANDBY
// Initialises outputs, servos and I2C communication and starts A3 PARKED.
void setup() {
  Serial.begin(9600);
  Serial.println(
    F("FIREWATCH A3 - ROBOTICS + SUPPRESSION")
  );
  pinMode(PIN_LASER, OUTPUT);
  pinMode(WATER_PUMP, OUTPUT);
  digitalWrite(PIN_LASER, LOW);
  digitalWrite(WATER_PUMP, LOW);
  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);
// SEQ 01 - Start at tested physical centre
  servoX.write(PARK_X_ANGLE);
  servoY.write(PARK_Y_ANGLE);
  Wire.begin(A3_ADDRESS);
  Wire.onReceive(receiveFromA1);
  Wire.onRequest(sendToA1);
  currentStatus = STATUS_PARKED;
  Serial.print(F("A3 online - PARKED PAN="));
  Serial.print(PARK_X_ANGLE);
  Serial.print(F(" TILT="));
  Serial.println(PARK_Y_ANGLE);
}

// MAIN LOOP
// Runs continuously and manages A1 commands, target holding and servo movement.
void loop() {
  ProcessA1Command();
  updateTargetHold();
  updateServos();
}

// RECEIVE COMMAND FROM A1
// Receives robot movement and suppression commands from Mission Control.
void receiveFromA1(int numBytes) {
  if (numBytes < 1)
    return;
  byte rawCmd = Wire.read();

// SEQ 05 - RECEIVE FIRE TARGET
  // Expected format: [cmd][X lo][X hi][Y lo][Y hi]
  if (rawCmd == CMD_TARGET) {
    if (
      numBytes >= 5 &&
      Wire.available() >= 4
    ) {
      int16_t x;
      int16_t y;
      Wire.readBytes(
        (byte*)&x,
        sizeof(x)
      );
      Wire.readBytes(
        (byte*)&y,
        sizeof(y)
      );
      pendingX = x;
      pendingY = y;
      pendingCommand = CMD_TARGET;
      commandReady = true;
    }
    else {
      while (Wire.available())
        Wire.read();
    }
  }

// SEQ 01 / 10 - STANDBY / RETURN TO PARKED
  else if (rawCmd == CMD_STANDBY) {
    pendingCommand = CMD_STANDBY;
    commandReady = true;
  }

// SEQ 08 - START FIRE SUPPRESSION
  else if (rawCmd == CMD_SUPPRESS_START) {
    pendingCommand = CMD_SUPPRESS_START;
    commandReady = true;
  }

// SEQ 09 - STOP FIRE SUPPRESSION
  else if (rawCmd == CMD_SUPPRESS_STOP) {
    pendingCommand = CMD_SUPPRESS_STOP;
    commandReady = true;
  }

  // UNKNOWN COMMAND
  else {
    while (Wire.available())
      Wire.read();
  }
}


// SEND STATUS TO A1
// Reports A3's current robot status when requested by Mission Control.
void sendToA1() {
  Wire.write(currentStatus);
}

// PROCESS COMMAND FROM A1
// Performs the action requested by Mission Control.
void ProcessA1Command() {
  if (!commandReady)
    return;
  IncomingCommand cmd;
  int16_t x;
  int16_t y;
  // Safely copy command received by the I2C interrupt
  noInterrupts();
  cmd = pendingCommand;
  x = pendingX;
  y = pendingY;
  pendingCommand = CMD_NONE;
  commandReady = false;
  interrupts();

// SEQ 07A / 10 - RETURN TO PARKED
  if (cmd == CMD_STANDBY) {
    // If A3 has just aimed at a possible fire,
    // hold the target for 3 seconds before returning to PARKED.
    if (
      hasTargetedFire &&
      currentStatus == STATUS_READY
    ) {
      startTargetHold();
    }
    else {
      PARKED();
    }
  }

// SEQ 05 - AIM ROBOT AT TARGET
  else if (cmd == CMD_TARGET) {
    TARGET(x, y);
  }

// SEQ 08 - SUPPRESS FIRE
  else if (cmd == CMD_SUPPRESS_START) {
    SUPPRESS_START();
  }

// SEQ 09 - STOP SUPPRESSION
  else if (cmd == CMD_SUPPRESS_STOP) {
    SUPPRESS_STOP();
  }
}

// START TARGET HOLD
// SEQ 07A - FIRE NOT CONFIRMED
// Holds the robot on the possible-fire target for 3 seconds before parking.
void startTargetHold() {
  isMoving = false;
  // Keep laser ON during hold
  digitalWrite(
    PIN_LASER,
    HIGH
  );
  holdingTarget = true;
  targetHoldStart = millis();
  Serial.println(
    F("Possible fire not confirmed - holding target for 3 seconds")
  );
}

// UPDATE TARGET HOLD
// SEQ 07A - FIRE NOT CONFIRMED
// Waits for the target hold time before beginning the return to PARKED.
void updateTargetHold() {
  if (!holdingTarget)
    return;

  if (
    millis() - targetHoldStart >=
    TARGET_HOLD_TIME
  ) {
    holdingTarget = false;
    Serial.println(
      F("Target hold complete - returning to PARKED")
    );
    PARKED();
  }
}

// PARKED
// SEQ 10 - RETURN ROBOT TO PARKED
// Sets the parked servo angles. A3 reports PARKED only after the
// servos have physically reached the parked position.

void PARKED() {
  holdingTarget = false;
  // Laser stays on while travelling back
  digitalWrite(
    PIN_LASER,
    HIGH
  );
  // Ensure suppression is off while parking
  digitalWrite(
    WATER_PUMP,
    LOW
  );
  targetXAngle = PARK_X_ANGLE;
  targetYAngle = PARK_Y_ANGLE;
  isMoving =
    (currentXAngle != targetXAngle) ||
    (currentYAngle != targetYAngle);

  // Do not set STATUS_PARKED yet.
  // The robot may still physically be moving back to centre.
  // STATUS_PARKED is set in updateServos() once PAN and TILT
  // have physically reached their parked angles.

  hasTargetedFire = false;
  Serial.print(
    F("STANDBY -> returning to PARKED PAN=")
  );
  Serial.print(PARK_X_ANGLE);
  Serial.print(F(" TILT="));
  Serial.println(PARK_Y_ANGLE);

  // Already physically parked
  if (!isMoving) {
    digitalWrite(
      PIN_LASER,
      LOW
    );
    // No movement is required, so it is safe to report PARKED immediately.
    currentStatus = STATUS_PARKED;
  }
}

// TARGET
// SEQ 05 - AIM ROBOT AT TARGET
// Converts fire coordinates from A1 into safe PAN/TILT servo angles.

void TARGET(
  int16_t px,
  int16_t py
) {
  holdingTarget = false;
  hasTargetedFire = true;
  // PIXEL OFFSET FROM IMAGE CENTRE
  float xOffset =
    (float)px - IMG_CX;
  float yOffset =
    IMG_CY - (float)py;
  // CONVERT PIXEL OFFSET TO SERVO ANGLE
  float xAngle =
    PARK_X_ANGLE +
    (xOffset / IMG_WIDTH) *
    H_FOV;
  float yAngle =
    PARK_Y_ANGLE +
    (yOffset / IMG_HEIGHT) *
    V_FOV;

  // HARD SAFETY LIMITS
  xAngle = constrain(
    xAngle,
    SERVO_X_MIN,
    SERVO_X_MAX
  );
  yAngle = constrain(
    yAngle,
    SERVO_Y_MIN,
    SERVO_Y_MAX
  );

  targetXAngle =
    (int)round(xAngle);
  targetYAngle =
    (int)round(yAngle);

  // Laser ON while moving
  digitalWrite(
    PIN_LASER,
    HIGH
  );

  // Robot is moving and is not READY until the target is reached
  currentStatus = STATUS_PARKED;
  isMoving = true;

  Serial.print(F("TARGET px=("));
  Serial.print(px);
  Serial.print(',');
  Serial.print(py);
  Serial.print(F(") -> PAN="));
  Serial.print(targetXAngle);
  Serial.print(F(" TILT="));
  Serial.println(targetYAngle);
}

// SUPPRESSION START
// SEQ 08 - SUPPRESS FIRE
// Activates the mock water pump after A1 confirms a fire.
void SUPPRESS_START() {
  // Fire confirmed - cancel false alarm hold
  holdingTarget = false;
  digitalWrite(
    WATER_PUMP,
    HIGH
  );
  // Keep laser ON during suppression
  digitalWrite(
    PIN_LASER,
    HIGH
  );
  currentStatus =
    STATUS_SUPPRESSING;
  Serial.println(
    F("SUPPRESS_START -> water ejecting")
  );
}

// SUPPRESSION STOP
// SEQ 09 - STOP SUPPRESSION AND RECHECK
// Stops the mock water pump and reports completion to A1.
void SUPPRESS_STOP() {
  digitalWrite(
    WATER_PUMP,
    LOW
  );
  currentStatus =
    STATUS_SUPPRESSION_COMPLETE;
  Serial.println(
    F("SUPPRESS_STOP -> water pump OFF")
  );
}

// SERVO MOTION CONTROL
// SEQ 05 / 10 - Moves the PAN and TILT servos gradually toward
// either the fire target or the PARKED position.
void updateServos() {
  if (!isMoving)
    return;
  if (
    millis() - lastStepTime <
    STEP_INTERVAL_MS
  )
    return;
  lastStepTime = millis();

  // PAN SERVO
  if (currentXAngle < targetXAngle) {
    currentXAngle += STEP_DEGREES;
  }
  else if (currentXAngle > targetXAngle) {
    currentXAngle -= STEP_DEGREES;
  }

  // Safety constrain before physically commanding servo
  currentXAngle = constrain(
    currentXAngle,
    SERVO_X_MIN,
    SERVO_X_MAX
  );
  servoX.write(currentXAngle);

  // TILT SERVO
  if (currentYAngle < targetYAngle) {
    currentYAngle += STEP_DEGREES;
  }
  else if (currentYAngle > targetYAngle) {
    currentYAngle -= STEP_DEGREES;
  }

  // Safety constrain before physically commanding servo
  currentYAngle = constrain(
    currentYAngle,
    SERVO_Y_MIN,
    SERVO_Y_MAX
  );
  servoY.write(currentYAngle);

  // ARRIVED AT TARGET POSITION
  if (
    currentXAngle == targetXAngle &&
    currentYAngle == targetYAngle
  ) {
    isMoving = false;
    // Turn laser off after movement
    digitalWrite(
      PIN_LASER,
      LOW
    );

// SEQ 10 - ROBOT PHYSICALLY PARKED
    // If there is no active fire target, A3 has finished returning home.
    if (!hasTargetedFire) {

      currentStatus =
        STATUS_PARKED;

      Serial.println(
        F("PARKED - robot centred")
      );
    }

// SEQ 05 - ROBOT AIMED AT FIRE TARGET
    // Only report READY if A3 is not currently suppressing
    // or reporting suppression complete.
    else if (
      currentStatus != STATUS_SUPPRESSING &&
      currentStatus != STATUS_SUPPRESSION_COMPLETE
    ) {
        currentStatus =
          STATUS_READY;
        Serial.print(
          F("READY - aimed at PAN=")
        );
        Serial.print(
          currentXAngle
        );
        Serial.print(
          F(" TILT=")
        );
        Serial.println(
          currentYAngle
        );
      }

      // SEQ 10 - ROBOT PHYSICALLY PARKED
      // Both servos have reached the tested home position.
      else {
        currentStatus =
          STATUS_PARKED;
        Serial.println(
          F("PARKED - robot centred")
        );
      }
    }
  }