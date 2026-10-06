#include <Wire.h>
#include <Servo.h>

// ============================================================
// FIREWATCH - ARDUINO 3 ROBOTICS CONTROLLER
// ============================================================


// ============================================================
// I2C
// ============================================================

const byte A3_ADDR = 9;

// Commands FROM A1
const byte CMD_A3_STANDBY         = 10;
const byte CMD_A3_TARGET          = 11;
const byte CMD_A3_SUPPRESS_START  = 12;
const byte CMD_A3_SUPPRESS_STOP   = 13;

// Status TO A1
const byte A3_PARKED               = 20;
const byte A3_READY                = 21;
const byte A3_SUPPRESSING          = 22;
const byte A3_SUPPRESSION_COMPLETE = 23;


// ============================================================
// PINS
// ============================================================

const byte PIN_SERVO_X = 5;
const byte PIN_SERVO_Y = 6;

const byte PIN_SUPPRESSION_LED = 7;


// ============================================================
// SERVOS
// ============================================================

Servo servoX;
Servo servoY;


// ============================================================
// CAMERA
// ============================================================

// Python/OpenCV image size
const int16_t CAMERA_WIDTH  = 640;
const int16_t CAMERA_HEIGHT = 480;

const int16_t CAMERA_CENTER_X = CAMERA_WIDTH / 2;   // 320
const int16_t CAMERA_CENTER_Y = CAMERA_HEIGHT / 2;  // 240


// ============================================================
// SERVO SETTINGS
// ============================================================

// Starting position
const int SERVO_X_CENTER = 90;
const int SERVO_Y_CENTER = 45;

// Physical limits
const int SERVO_X_MIN = 10;
const int SERVO_X_MAX = 170;

const int SERVO_Y_MIN = 5;
const int SERVO_Y_MAX = 90;


// Approximate calibration
const float X_PIXELS_PER_DEGREE = 10.6;
const float Y_PIXELS_PER_DEGREE = 12.0;


// Direction
// Change -1 to +1 if that servo moves the wrong direction
const int X_DIRECTION = -1;
const int Y_DIRECTION = -1;


// ============================================================
// CONTROLLED MOVEMENT
// ============================================================

// Maximum movement during ONE update
const int MAX_SERVO_STEP = 3;

// Wait 30 ms between movements
const unsigned long SERVO_UPDATE_INTERVAL = 30;

// Give up after 10 seconds
const unsigned long TARGET_TIMEOUT = 10000;


// ============================================================
// STATES
// ============================================================

enum A3State {

  STATE_STANDBY,

  STATE_TARGETING,

  STATE_READY,

  STATE_SUPPRESSING,

  STATE_SUPPRESSION_COMPLETE

};


A3State currentState = STATE_STANDBY;


// ============================================================
// TARGET INFORMATION
// ============================================================

int16_t targetX = CAMERA_CENTER_X;
int16_t targetY = CAMERA_CENTER_Y;


// Current servo positions
int servoXPosition = SERVO_X_CENTER;
int servoYPosition = SERVO_Y_CENTER;


// FINAL positions we want the servos to reach
int desiredServoX = SERVO_X_CENTER;
int desiredServoY = SERVO_Y_CENTER;


// Timing
unsigned long targetStartTime = 0;
unsigned long lastServoUpdate = 0;


// ============================================================
// I2C VARIABLES
// ============================================================

volatile byte currentStatus = A3_PARKED;

volatile byte pendingCommand = 0;

volatile int16_t pendingX = 0;
volatile int16_t pendingY = 0;

volatile bool commandWaiting = false;


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(9600);


  // --------------------------
  // Attach servos
  // --------------------------

  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);


  // Start centred
  servoXPosition = SERVO_X_CENTER;
  servoYPosition = SERVO_Y_CENTER;

  servoX.write(servoXPosition);
  servoY.write(servoYPosition);


  // --------------------------
  // Suppression LED
  // --------------------------

  pinMode(PIN_SUPPRESSION_LED, OUTPUT);

  digitalWrite(PIN_SUPPRESSION_LED, LOW);


  // --------------------------
  // I2C
  // --------------------------

  Wire.begin(A3_ADDR);

  Wire.onReceive(receiveEvent);

  Wire.onRequest(requestEvent);


  // --------------------------
  // Startup
  // --------------------------

  Serial.println(F("--------------------------------"));
  Serial.println(F(" FIREWATCH A3 ROBOTICS"));
  Serial.println(F("--------------------------------"));

  Serial.print(F("I2C address: "));
  Serial.println(A3_ADDR);

  Serial.print(F("Camera: "));
  Serial.print(CAMERA_WIDTH);
  Serial.print(F(" x "));
  Serial.println(CAMERA_HEIGHT);

  Serial.print(F("Camera centre: X="));
  Serial.print(CAMERA_CENTER_X);

  Serial.print(F(" Y="));
  Serial.println(CAMERA_CENTER_Y);

  Serial.println(F(""));
  Serial.println(F("A3: PARKED"));

  Serial.println(F(""));
  Serial.println(F("MANUAL TEST:"));
  Serial.println(F("T X Y"));
  Serial.println(F("Example: T 500 240"));
  Serial.println(F("0 = standby"));
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  handleSerialInput();

  processPendingCommand();

  runStateMachine();
}


// ============================================================
// RECEIVE FROM A1
// ============================================================

void receiveEvent(int howMany) {

  if (howMany <= 0) {
    return;
  }


  byte command = Wire.read();


  // ----------------------------------------------------------
  // STANDBY
  // ----------------------------------------------------------

  if (command == CMD_A3_STANDBY) {

    pendingCommand = CMD_A3_STANDBY;

    commandWaiting = true;
  }


  // ----------------------------------------------------------
  // TARGET
  // ----------------------------------------------------------

  else if (command == CMD_A3_TARGET) {

    if (Wire.available() >= 4) {

      int16_t newX;
      int16_t newY;

      Wire.readBytes(
        (byte*)&newX,
        sizeof(newX)
      );

      Wire.readBytes(
        (byte*)&newY,
        sizeof(newY)
      );


      pendingX = newX;
      pendingY = newY;

      pendingCommand = CMD_A3_TARGET;

      commandWaiting = true;
    }
  }


  // ----------------------------------------------------------
  // SUPPRESSION START
  // ----------------------------------------------------------

  else if (command == CMD_A3_SUPPRESS_START) {

    pendingCommand = CMD_A3_SUPPRESS_START;

    commandWaiting = true;
  }


  // ----------------------------------------------------------
  // SUPPRESSION STOP
  // ----------------------------------------------------------

  else if (command == CMD_A3_SUPPRESS_STOP) {

    pendingCommand = CMD_A3_SUPPRESS_STOP;

    commandWaiting = true;
  }


  // Clear remaining bytes
  while (Wire.available()) {

    Wire.read();
  }
}


// ============================================================
// A1 REQUESTS A3 STATUS
// ============================================================

void requestEvent() {

  Wire.write(currentStatus);
}


// ============================================================
// PROCESS COMMAND
// ============================================================

void processPendingCommand() {

  if (!commandWaiting) {

    return;
  }


  noInterrupts();


  byte command = pendingCommand;

  int16_t newX = pendingX;

  int16_t newY = pendingY;

  commandWaiting = false;


  interrupts();


  // ----------------------------------------------------------
  // STANDBY
  // ----------------------------------------------------------

  if (command == CMD_A3_STANDBY) {

    Serial.println(F(""));
    Serial.println(F("A1 -> A3: STANDBY"));

    changeState(STATE_STANDBY);
  }


  // ----------------------------------------------------------
  // TARGET
  // ----------------------------------------------------------

  else if (command == CMD_A3_TARGET) {

    targetX = newX;

    targetY = newY;


    Serial.println(F(""));
    Serial.println(F("A1 -> A3: TARGET"));

    Serial.print(F("Target X = "));
    Serial.println(targetX);

    Serial.print(F("Target Y = "));
    Serial.println(targetY);


    startTargeting();
  }


  // ----------------------------------------------------------
  // SUPPRESSION START
  // ----------------------------------------------------------

  else if (command == CMD_A3_SUPPRESS_START) {

    Serial.println(F(""));
    Serial.println(F("A1 -> A3: SUPPRESS START"));

    changeState(STATE_SUPPRESSING);
  }


  // ----------------------------------------------------------
  // SUPPRESSION STOP
  // ----------------------------------------------------------

  else if (command == CMD_A3_SUPPRESS_STOP) {

    Serial.println(F(""));
    Serial.println(F("A1 -> A3: SUPPRESS STOP"));

    digitalWrite(
      PIN_SUPPRESSION_LED,
      LOW
    );

    currentState = STATE_SUPPRESSION_COMPLETE;

    currentStatus = A3_SUPPRESSION_COMPLETE;
  }
}


// ============================================================
// START TARGETING
//
// IMPORTANT:
//
// Calculate the FINAL servo position ONCE.
//
// We DO NOT repeatedly use the same camera error.
// ============================================================

void startTargeting() {


  // ----------------------------------------------------------
  // Calculate pixel error
  // ----------------------------------------------------------

  int16_t errorX =
    targetX - CAMERA_CENTER_X;

  int16_t errorY =
    targetY - CAMERA_CENTER_Y;


  Serial.print(F("Pixel error X = "));
  Serial.println(errorX);

  Serial.print(F("Pixel error Y = "));
  Serial.println(errorY);


  // ----------------------------------------------------------
  // Convert pixels to degrees
  // ----------------------------------------------------------

  int movementX = round(

    ((float)errorX / X_PIXELS_PER_DEGREE)

    * X_DIRECTION
  );


  int movementY = round(

    ((float)errorY / Y_PIXELS_PER_DEGREE)

    * Y_DIRECTION
  );


  Serial.print(F("Calculated X movement = "));
  Serial.println(movementX);

  Serial.print(F("Calculated Y movement = "));
  Serial.println(movementY);


  // ----------------------------------------------------------
  // Calculate FINAL servo positions
  //
  // This happens ONCE.
  // ----------------------------------------------------------

  desiredServoX =
    servoXPosition + movementX;

  desiredServoY =
    servoYPosition + movementY;


  // ----------------------------------------------------------
  // Protect physical limits
  // ----------------------------------------------------------

  desiredServoX = constrain(

    desiredServoX,

    SERVO_X_MIN,

    SERVO_X_MAX
  );


  desiredServoY = constrain(

    desiredServoY,

    SERVO_Y_MIN,

    SERVO_Y_MAX
  );


  Serial.print(F("Desired servo X = "));
  Serial.println(desiredServoX);

  Serial.print(F("Desired servo Y = "));
  Serial.println(desiredServoY);


  // ----------------------------------------------------------
  // Start movement
  // ----------------------------------------------------------

  targetStartTime = millis();

  lastServoUpdate = millis();

  currentState = STATE_TARGETING;

  currentStatus = A3_PARKED;


  Serial.println(F("A3: TARGETING"));
}


// ============================================================
// STATE MACHINE
// ============================================================

void runStateMachine() {

  switch (currentState) {


    // --------------------------------------------------------
    // STANDBY
    // --------------------------------------------------------

    case STATE_STANDBY:

      break;


    // --------------------------------------------------------
    // TARGETING
    // --------------------------------------------------------

    case STATE_TARGETING:


      // Safety timeout
      if (
        millis() - targetStartTime
        >= TARGET_TIMEOUT
      ) {

        Serial.println(F("A3: TARGET TIMEOUT"));

        currentState = STATE_READY;

        currentStatus = A3_READY;

        break;
      }


      // Only move every 30 ms
      if (
        millis() - lastServoUpdate
        >= SERVO_UPDATE_INTERVAL
      ) {

        lastServoUpdate = millis();

        updateTargeting();
      }


      break;


    // --------------------------------------------------------
    // READY
    // --------------------------------------------------------

    case STATE_READY:

      // STOP MOVING.
      break;


    // --------------------------------------------------------
    // SUPPRESSING
    // --------------------------------------------------------

    case STATE_SUPPRESSING:

      digitalWrite(
        PIN_SUPPRESSION_LED,
        HIGH
      );

      break;


    // --------------------------------------------------------
    // COMPLETE
    // --------------------------------------------------------

    case STATE_SUPPRESSION_COMPLETE:

      break;
  }
}


// ============================================================
// CONTROLLED SERVO MOVEMENT
// ============================================================

void updateTargeting() {


  // ==========================================================
  // X SERVO
  // ==========================================================

  if (servoXPosition < desiredServoX) {

    servoXPosition += MAX_SERVO_STEP;


    // Do NOT overshoot
    if (servoXPosition > desiredServoX) {

      servoXPosition = desiredServoX;
    }


    servoX.write(servoXPosition);
  }


  else if (servoXPosition > desiredServoX) {

    servoXPosition -= MAX_SERVO_STEP;


    // Do NOT overshoot
    if (servoXPosition < desiredServoX) {

      servoXPosition = desiredServoX;
    }


    servoX.write(servoXPosition);
  }



  // ==========================================================
  // Y SERVO
  // ==========================================================

  if (servoYPosition < desiredServoY) {

    servoYPosition += MAX_SERVO_STEP;


    if (servoYPosition > desiredServoY) {

      servoYPosition = desiredServoY;
    }


    servoY.write(servoYPosition);
  }


  else if (servoYPosition > desiredServoY) {

    servoYPosition -= MAX_SERVO_STEP;


    if (servoYPosition < desiredServoY) {

      servoYPosition = desiredServoY;
    }


    servoY.write(servoYPosition);
  }



  // ==========================================================
  // PRINT MOVEMENT
  // ==========================================================

  Serial.print(F("Servo X = "));
  Serial.print(servoXPosition);

  Serial.print(F(" / "));
  Serial.print(desiredServoX);

  Serial.print(F("    Servo Y = "));
  Serial.print(servoYPosition);

  Serial.print(F(" / "));
  Serial.println(desiredServoY);



  // ==========================================================
  // HAVE BOTH SERVOS ARRIVED?
  // ==========================================================

  bool xFinished =
    servoXPosition == desiredServoX;

  bool yFinished =
    servoYPosition == desiredServoY;


  if (xFinished && yFinished) {

    Serial.println(F(""));
    Serial.println(F("A3: TARGET POSITION REACHED"));

    Serial.print(F("Final X = "));
    Serial.println(servoXPosition);

    Serial.print(F("Final Y = "));
    Serial.println(servoYPosition);


    // STOP targeting
    currentState = STATE_READY;

    // Tell A1 we're ready
    currentStatus = A3_READY;

    Serial.println(F("A3: READY"));
  }
}


// ============================================================
// CHANGE STATE
// ============================================================

void changeState(A3State newState) {

  currentState = newState;


  switch (newState) {


    // --------------------------------------------------------
    // STANDBY
    // --------------------------------------------------------

    case STATE_STANDBY:


      digitalWrite(
        PIN_SUPPRESSION_LED,
        LOW
      );


      // Return webcam to starting position
      servoXPosition = SERVO_X_CENTER;

      servoYPosition = SERVO_Y_CENTER;


      servoX.write(
        servoXPosition
      );

      servoY.write(
        servoYPosition
      );


      desiredServoX = SERVO_X_CENTER;

      desiredServoY = SERVO_Y_CENTER;


      currentStatus = A3_PARKED;


      Serial.println(F("A3: PARKED"));


      break;


    // --------------------------------------------------------
    // TARGETING
    // --------------------------------------------------------

    case STATE_TARGETING:

      currentStatus = A3_PARKED;

      break;


    // --------------------------------------------------------
    // READY
    // --------------------------------------------------------

    case STATE_READY:

      currentStatus = A3_READY;

      break;


    // --------------------------------------------------------
    // SUPPRESSING
    // --------------------------------------------------------

    case STATE_SUPPRESSING:


      digitalWrite(
        PIN_SUPPRESSION_LED,
        HIGH
      );


      currentStatus = A3_SUPPRESSING;


      Serial.println(F("A3: SUPPRESSING"));


      break;


    // --------------------------------------------------------
    // COMPLETE
    // --------------------------------------------------------

    case STATE_SUPPRESSION_COMPLETE:

      currentStatus =
        A3_SUPPRESSION_COMPLETE;

      break;
  }
}


// ============================================================
// MANUAL TEST
// ============================================================

void handleSerialInput() {

  if (!Serial.available()) {

    return;
  }


  char command = Serial.read();


  // ----------------------------------------------------------
  // TARGET TEST
  //
  // Example:
  //
  // T 500 240
  // ----------------------------------------------------------

  if (
    command == 'T'
    ||
    command == 't'
  ) {


    int x = Serial.parseInt();

    int y = Serial.parseInt();


    if (
      x < 0
      ||
      x > CAMERA_WIDTH
      ||
      y < 0
      ||
      y > CAMERA_HEIGHT
    ) {

      Serial.println(
        F("ERROR: target outside camera")
      );

      return;
    }


    targetX = (int16_t)x;

    targetY = (int16_t)y;


    Serial.println(F(""));
    Serial.println(F("MANUAL TARGET"));

    Serial.print(F("X = "));
    Serial.println(targetX);

    Serial.print(F("Y = "));
    Serial.println(targetY);


    startTargeting();


    return;
  }


  // ----------------------------------------------------------
  // STANDBY
  // ----------------------------------------------------------

  if (command == '0') {

    Serial.println(F(""));
    Serial.println(F("MANUAL STANDBY"));

    changeState(STATE_STANDBY);

    return;
  }


  // Ignore newlines
  if (
    command == '\n'
    ||
    command == '\r'
  ) {

    return;
  }
}