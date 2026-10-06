/*
  Firewatch Arduino 3 Robot Manipulator - Devlin

  Role of A3:
  - controls the pan and tilt servos
  - interprets fire coordinates sent from A1 into servo angles
  - centres the webcam onto the fire
  - activates the fire suppression system (mock water pump)

  ============================================================
  SERVO MOUNT TESTING - 6 OCTOBER 2026
  ============================================================

  Duinotech 2-servo pan/tilt mount was physically tested
  before integrating it into the Firewatch system.

  PAN SERVO:
  - Signal pin: D9
  - Tested safe minimum: 20 degrees
  - Tested safe maximum: 175 degrees
  - Physical centre / parked position: 100 degrees

  TILT SERVO:
  - Signal pin: D10
  - Tested safe minimum: 105 degrees
  - Tested safe maximum: 180 degrees
  - Physical centre / parked position: 155 degrees

  IMPORTANT:
  - Servo commands are constrained to these tested limits.
  - Do not change limits to 0-180 without physically checking mount.
  - Servo movement is limited to 1 degree per step to prevent
    sudden/violent movement.
  - PAN and TILT angles correspond directly to the physical
    servo angles established during testing.

  ============================================================
  SETUP AND WIRING
  ============================================================

  I2C:
  - SDA -> A4
  - SCL -> A5
  - GND connected between boards

  Outputs:
  - PAN SERVO  -> PIN 9
  - TILT SERVO -> PIN 10
  - LASER      -> PIN 7
  - WATER PUMP LED -> PIN 8

  I2C comms:
  - Home position = PARKED
  - Target position = READY
  - Suppression system on = SUPPRESSING
  - Suppression system off = SUPPRESSION COMPLETE
*/


#include <Wire.h>
#include <Servo.h>


// ============================================================
// I2C ADDRESS
// ============================================================

const byte A3_ADDRESS = 9;


// ============================================================
// PIN DECLARATIONS
// ============================================================

const int PIN_SERVO_X = 9;     // PAN
const int PIN_SERVO_Y = 10;    // TILT
const int PIN_LASER   = 7;
const int WATER_PUMP  = 8;


// ============================================================
// TESTED HOME / CENTRE POSITION
// ============================================================

const int PARK_X_ANGLE = 100;   // PAN centre
const int PARK_Y_ANGLE = 155;   // TILT centre


// ============================================================
// TESTED SAFE SERVO LIMITS
// ============================================================

// PAN
const int SERVO_X_MIN = 20;
const int SERVO_X_MAX = 175;

// TILT
const int SERVO_Y_MIN = 105;
const int SERVO_Y_MAX = 180;


// ============================================================
// CAMERA MAPPING
// ============================================================

const float IMG_WIDTH  = 640.0;
const float IMG_HEIGHT = 480.0;

const float IMG_CX = IMG_WIDTH / 2.0;
const float IMG_CY = IMG_HEIGHT / 2.0;

const float H_FOV = 60.0;
const float V_FOV = 60.0;


// ============================================================
// SERVO MOTION CONTROL
// ============================================================

// Move slowly to prevent sudden movement
const unsigned long STEP_INTERVAL_MS = 40;
const int STEP_DEGREES = 1;


// ============================================================
// TARGET HOLD TIME
// ============================================================

// If possible fire is not confirmed,
// hold position for 3 seconds before returning to PARKED.

const unsigned long TARGET_HOLD_TIME = 3000;

bool holdingTarget = false;
unsigned long targetHoldStart = 0;


// ============================================================
// SERVOS
// ============================================================

Servo servoX;
Servo servoY;


// ============================================================
// I2C COMMANDS FROM A1
// ============================================================

enum IncomingCommand : byte {

  CMD_NONE           = 0,
  CMD_STANDBY        = 10,
  CMD_TARGET         = 11,
  CMD_SUPPRESS_START = 12,
  CMD_SUPPRESS_STOP  = 13

};


// ============================================================
// STATUS SENT TO A1
// ============================================================

enum OutgoingStatus : byte {

  STATUS_PARKED               = 20,
  STATUS_READY                = 21,
  STATUS_SUPPRESSING          = 22,
  STATUS_SUPPRESSION_COMPLETE = 23

};


// ============================================================
// I2C VARIABLES
// ============================================================

volatile IncomingCommand pendingCommand = CMD_NONE;

volatile int16_t pendingX = 0;
volatile int16_t pendingY = 0;

volatile bool commandReady = false;


// ============================================================
// CURRENT ROBOT STATUS
// ============================================================

byte currentStatus = STATUS_PARKED;

int targetXAngle = PARK_X_ANGLE;
int targetYAngle = PARK_Y_ANGLE;

int currentXAngle = PARK_X_ANGLE;
int currentYAngle = PARK_Y_ANGLE;

bool isMoving = false;

unsigned long lastStepTime = 0;


// ============================================================
// TRACK WHETHER WE HAVE AIMED AT A FIRE
// ============================================================

bool hasTargetedFire = false;


// ============================================================
// SETUP
// ============================================================

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

  // Start at tested physical centre
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


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  ProcessA1Command();

  updateTargetHold();

  updateServos();

}


// ============================================================
// RECEIVE COMMAND FROM A1
// ============================================================

void receiveFromA1(int numBytes) {

  if (numBytes < 1)
    return;

  byte rawCmd = Wire.read();


  // TARGET
  if (rawCmd == CMD_TARGET) {

    // Format:
    // [cmd][X lo][X hi][Y lo][Y hi]

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


  // STANDBY
  else if (rawCmd == CMD_STANDBY) {

    pendingCommand = CMD_STANDBY;
    commandReady = true;

  }


  // SUPPRESSION START
  else if (rawCmd == CMD_SUPPRESS_START) {

    pendingCommand = CMD_SUPPRESS_START;
    commandReady = true;

  }


  // SUPPRESSION STOP
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


// ============================================================
// SEND STATUS TO A1
// ============================================================

void sendToA1() {

  Wire.write(currentStatus);

}


// ============================================================
// PROCESS COMMAND FROM A1
// ============================================================

void ProcessA1Command() {

  if (!commandReady)
    return;

  IncomingCommand cmd;

  int16_t x;
  int16_t y;

  noInterrupts();

  cmd = pendingCommand;

  x = pendingX;
  y = pendingY;

  pendingCommand = CMD_NONE;
  commandReady = false;

  interrupts();


  // STANDBY
  if (cmd == CMD_STANDBY) {

    /*
      If the robot has just aimed at a possible fire,
      don't immediately return to centre.

      Hold the target position for 3 seconds first.
    */

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


  // TARGET
  else if (cmd == CMD_TARGET) {

    TARGET(x, y);

  }


  // SUPPRESSION START
  else if (cmd == CMD_SUPPRESS_START) {

    SUPPRESS_START();

  }


  // SUPPRESSION STOP
  else if (cmd == CMD_SUPPRESS_STOP) {

    SUPPRESS_STOP();

  }
}


// ============================================================
// START TARGET HOLD
// ============================================================

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


// ============================================================
// UPDATE TARGET HOLD
// ============================================================

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


// ============================================================
// PARKED
// ============================================================

void PARKED() {

  holdingTarget = false;

  // Laser stays on while travelling back
  digitalWrite(
    PIN_LASER,
    HIGH
  );

  digitalWrite(
    WATER_PUMP,
    LOW
  );

  targetXAngle = PARK_X_ANGLE;
  targetYAngle = PARK_Y_ANGLE;

  isMoving =
    (currentXAngle != targetXAngle) ||
    (currentYAngle != targetYAngle);


  /*
    IMPORTANT PARKING CHANGE:

    Do NOT set currentStatus = STATUS_PARKED here.

    The robot may still physically be moving back to centre.

    STATUS_PARKED will instead be set in updateServos()
    after PAN and TILT have actually reached the parked angles.

    This allows A1 to keep A2/CV stopped until A3 has really
    finished parking.
  */

  hasTargetedFire = false;

  Serial.print(
    F("STANDBY -> returning to PARKED PAN=")
  );

  Serial.print(PARK_X_ANGLE);

  Serial.print(F(" TILT="));

  Serial.println(PARK_Y_ANGLE);

  // Already parked
  if (!isMoving) {

    digitalWrite(
      PIN_LASER,
      LOW
    );

    /*
      If no movement is required, we are already physically
      centred, so it is safe to report PARKED immediately.
    */
    currentStatus = STATUS_PARKED;

  }
}


// ============================================================
// TARGET
// ============================================================

void TARGET(
  int16_t px,
  int16_t py
) {

  holdingTarget = false;

  hasTargetedFire = true;


  // ----------------------------------------------------------
  // PIXEL OFFSET FROM IMAGE CENTRE
  // ----------------------------------------------------------

  float xOffset =
    (float)px - IMG_CX;

  float yOffset =
    IMG_CY - (float)py;


  // ----------------------------------------------------------
  // CONVERT PIXEL OFFSET TO SERVO ANGLE
  // ----------------------------------------------------------

  float xAngle =
    PARK_X_ANGLE +
    (xOffset / IMG_WIDTH) *
    H_FOV;

  float yAngle =
    PARK_Y_ANGLE +
    (yOffset / IMG_HEIGHT) *
    V_FOV;


  // ----------------------------------------------------------
  // HARD SAFETY LIMITS
  // ----------------------------------------------------------

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


// ============================================================
// SUPPRESSION START
// ============================================================

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


// ============================================================
// SUPPRESSION STOP
// ============================================================

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


// ============================================================
// SERVO MOTION CONTROL
// ============================================================

void updateServos() {

  if (!isMoving)
    return;

  if (
    millis() - lastStepTime <
    STEP_INTERVAL_MS
  )
    return;

  lastStepTime = millis();


  // ----------------------------------------------------------
  // PAN SERVO
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // TILT SERVO
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // ARRIVED
  // ----------------------------------------------------------

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


    if (
      currentStatus != STATUS_SUPPRESSING &&
      currentStatus != STATUS_SUPPRESSION_COMPLETE
    ) {

      if (hasTargetedFire) {

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

      else {

        /*
          This is now the point where A3 reports PARKED.

          Because we've reached this section, both servos have
          physically reached their target angles.

          When returning home those targets are:
          PAN = 100
          TILT = 155
        */

        currentStatus =
          STATUS_PARKED;

        Serial.println(
          F("PARKED - robot centred")
        );

      }
    }
  }
}