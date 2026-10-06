#include <Wire.h>
#include <Servo.h>

const byte A3_ADDR = 9;


// Commands received FROM A1
const byte CMD_A3_STANDBY       = 10;
const byte CMD_A3_TARGET        = 11;
const byte CMD_A3_SUPPRESS_START = 12;
const byte CMD_A3_SUPPRESS_STOP  = 13;


const byte A3_PARKED              = 20;
const byte A3_READY               = 21;
const byte A3_SUPPRESSING         = 22;
const byte A3_SUPPRESSION_COMPLETE = 23;

const byte PIN_SERVO_X = 5;
const byte PIN_SERVO_Y = 6;


const byte PIN_SUPPRESSION_LED = 7;


Servo servoX;
Servo servoY;

const int16_t CAMERA_WIDTH  = 1920;
const int16_t CAMERA_HEIGHT = 1080;

const int16_t CAMERA_CENTER_X = CAMERA_WIDTH / 2;
const int16_t CAMERA_CENTER_Y = CAMERA_HEIGHT / 2;


const int SERVO_X_CENTER = 90;
const int SERVO_Y_CENTER = 45;

const int SERVO_X_MIN = 10;
const int SERVO_X_MAX = 170;

const int SERVO_Y_MIN = 5;
const int SERVO_Y_MAX = 90;

const float X_PIXELS_PER_DEGREE = 10.6;
const float Y_PIXELS_PER_DEGREE = 12.0;

const int X_DIRECTION = -1;
const int Y_DIRECTION = -1;

const int16_t TARGET_TOLERANCE_X = 5;
const int16_t TARGET_TOLERANCE_Y = 5;

const int MAX_SERVO_STEP = 3;
const unsigned long SERVO_UPDATE_INTERVAL = 30;
const unsigned long TARGET_TIMEOUT = 10000;

enum A3State {
  STATE_STANDBY,
  STATE_TARGETING,
  STATE_READY,
  STATE_SUPPRESSING,
  STATE_SUPRESSION_COMPLETE
};

A3State currentState = STATE_STANDBY;

volatile int16_t receivedTargetX = CAMERA_CENTER_X;
volatile int16_t receivedTargetY = CAMERA_CENTER_Y;

volatile bool newTargetReceived = false;

int16_t targetX = CAMERA_CENTER_X;
int16_t targetY = CAMERA_CENTER_Y;

int servoXPosition = SERVO_X_CENTER;
int servoYPosition = SERVO_Y_CENTER;

unsigned long targetStartTime = 0;
unsigned long lastServoUpdate = 0;

volatile byte currentStatus = A3_PARKED;

volatile byte pendingCommand = 0;

volatile int16_t pendingX = 0;
volatile int16_t pendingY = 0;

volatile bool commandWaiting = false;

void setup() {

  Serial.begin(9600);

  // Attach servos
  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);

  // Suppression/demo LED
  pinMode(PIN_SUPPRESSION_LED, OUTPUT);
  digitalWrite(PIN_SUPPRESSION_LED, LOW);


  // Put servos into starting position
  servoXPosition = SERVO_X_CENTER;
  servoYPosition = SERVO_Y_CENTER;

  servoX.write(servoXPosition);
  servoY.write(servoYPosition);

  Wire.begin(A3_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.println(F("-----------------------------------------"));
  Serial.println(F(" FIREWATCH A3 ROBOTICS CONTROLLER"));
  Serial.println(F("-----------------------------------------"));

  Serial.print(F(" I2C address: "));
  Serial.println(A3_ADDR);

  Serial.print(F(" Camera resolution: "));
  Serial.print(CAMERA_WIDTH);
  Serial.print(F(" x "));
  Serial.println(CAMERA_HEIGHT);

  Serial.print(F(" Camera centre: X="));
  Serial.print(CAMERA_CENTER_X);
  Serial.print(F(" Y="));
  Serial.println(CAMERA_CENTER_Y);

  Serial.println(F(" Starting in STANDBY"));

  changeState(STATE_STANDBY);
  Serial.println(F(""));
  Serial.println(F("MANUAL TEST COMMANDS"));
  Serial.println(F("--------------------"));
  Serial.println(F("T X Y = enter target coordinates"));
  Serial.println(F("0     = standby"));
  Serial.println(F("S     = suppression start"));
  Serial.println(F("X     = suppression stop"));
  Serial.println(F("P     = print current status"));
  Serial.println(F(""));
  Serial.println(F("Example: T 500 240"));
  Serial.println(F(""));
}

void loop() {
  handleSerialInput();
  processPendingCommand();
  runStateMachine();
}

void receiveEvent(int howMany) {

  if (howMany <= 0) {
    return;
  }


  byte command = Wire.read();
  if (command == CMD_A3_STANDBY) {

    pendingCommand = CMD_A3_STANDBY;
    commandWaiting = true;

  }
  else if (command == CMD_A3_TARGET) {

    // Need:
    //
    // command = 1 byte
    // X       = 2 bytes
    // Y       = 2 bytes
    //
    // Total = 5 bytes

    if (Wire.available() >= 4) {

      int16_t newX;
      int16_t newY;

      Wire.readBytes((byte*)&newX, sizeof(newX));
      Wire.readBytes((byte*)&newY, sizeof(newY));

      pendingX = newX;
      pendingY = newY;

      pendingCommand = CMD_A3_TARGET;
      commandWaiting = true;
    }

  }
  else if (command == CMD_A3_SUPPRESS_START) {

    pendingCommand = CMD_A3_SUPPRESS_START;
    commandWaiting = true;

  }
  else if (command == CMD_A3_SUPPRESS_STOP) {

    pendingCommand = CMD_A3_SUPPRESS_STOP;
    commandWaiting = true;
  }
  else {

    // Consume remaining bytes so the bus transaction is clean.
    while (Wire.available()) {
      Wire.read();
    }
  }
}

void requestEvent() {

  Wire.write(currentStatus);
}

void processPendingCommand() {

  if (!commandWaiting) {
    return;
  }


  // Copy the command into a local variable.

  noInterrupts();

  byte command = pendingCommand;

  int16_t newX = pendingX;
  int16_t newY = pendingY;

  commandWaiting = false;

  interrupts();
  if (command == CMD_A3_STANDBY) {

    Serial.println(F(""));
    Serial.println(F("A1 -> A3: STANDBY"));

    changeState(STATE_STANDBY);
  }
  else if (command == CMD_A3_TARGET) {

    targetX = newX;
    targetY = newY;

    Serial.println(F(""));
    Serial.println(F("A1 -> A3: TARGET"));

    Serial.print(F(" Target X = "));
    Serial.println(targetX);

    Serial.print(F(" Target Y = "));
    Serial.println(targetY);

    changeState(STATE_TARGETING);
  }
  else if (command == CMD_A3_SUPPRESS_START) {

    Serial.println(F(""));
    Serial.println(F("A1 -> A3: SUPPRESS_START"));

    changeState(STATE_SUPPRESSING);
  }
  else if (command == CMD_A3_SUPPRESS_STOP) {

    Serial.println(F(""));
    Serial.println(F("A1 -> A3: SUPPRESS_STOP"));

    stopSuppression();

    changeState(STATE_SUPRESSION_COMPLETE);
  }
}
void changeState(A3State newState) {

  Serial.print(F("A3 STATE: "));
  Serial.print(stateName(currentState));
  Serial.print(F(" -> "));
  Serial.println(stateName(newState));


  currentState = newState;


  switch (newState) {
    case STATE_STANDBY:

      /*
        Turn demo output OFF.
      */

      digitalWrite(PIN_SUPPRESSION_LED, LOW);
      servoXPosition = SERVO_X_CENTER;
      servoYPosition = SERVO_Y_CENTER;

      servoX.write(servoXPosition);
      servoY.write(servoYPosition);


      currentStatus = A3_PARKED;

      Serial.println(F("A3: PARKED"));
      Serial.println(F("A3: Suppression LED OFF"));

      break;
    case STATE_TARGETING:

      targetStartTime = millis();
      lastServoUpdate = millis();
      digitalWrite(PIN_SUPPRESSION_LED, LOW);


      currentStatus = A3_PARKED;

      Serial.println(F("A3: TARGETING started"));

      break;
    case STATE_READY:
      currentStatus = A3_READY;

      Serial.println(F("A3: READY"));
      Serial.println(F("A3: Target position reached"));

      break;
    case STATE_SUPPRESSING:
      digitalWrite(PIN_SUPPRESSION_LED, HIGH);

      currentStatus = A3_SUPPRESSING;

      Serial.println(F("A3: SUPPRESSING"));
      Serial.println(F("A3: Water Pump activated"));

      break;
  }
}

void runStateMachine() {

  switch (currentState) {
    case STATE_STANDBY:
      break;
    case STATE_TARGETING:
      if (millis() - targetStartTime >= TARGET_TIMEOUT) {

        Serial.println(F("A3: TARGET TIMEOUT"));
        servoXPosition = SERVO_X_CENTER;
        servoYPosition = SERVO_Y_CENTER;

        servoX.write(servoXPosition);
        servoY.write(servoYPosition);

        currentStatus = A3_PARKED;

        currentState = STATE_STANDBY;

        break;
      }
      if (millis() - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {

        lastServoUpdate = millis();

        updateTargeting();
      }

      break;
    case STATE_READY:
      break;
    case STATE_SUPPRESSING:
      digitalWrite(PIN_SUPPRESSION_LED, HIGH);

      break;
  }
}

void updateTargeting() {
  int16_t errorX = targetX - CAMERA_CENTER_X;
  int16_t errorY = targetY - CAMERA_CENTER_Y;


  Serial.print(F("Target error X="));
  Serial.print(errorX);

  Serial.print(F(" Y="));
  Serial.println(errorY);
  bool xCentred =
    abs(errorX) <= TARGET_TOLERANCE_X;

  bool yCentred =
    abs(errorY) <= TARGET_TOLERANCE_Y;


  if (xCentred && yCentred) {

    Serial.println(F("A3: TARGET CENTRED"));

    currentState = STATE_READY;
    currentStatus = A3_READY;

    Serial.print(F("Final servo X="));
    Serial.println(servoXPosition);

    Serial.print(F("Final servo Y="));
    Serial.println(servoYPosition);

    return;
  }
  if (!xCentred) {

    float degreesX =
      ((float)errorX / X_PIXELS_PER_DEGREE) * X_DIRECTION;

    int stepX = (int)round(degreesX);


    /*
      Make sure a non-zero error produces at least one degree
      of movement.
    */

    if (stepX == 0) {

      if (errorX > 0) stepX = X_DIRECTION;
      else             stepX = -X_DIRECTION;
    }


    // Limit movement per update
    if (stepX > MAX_SERVO_STEP)
      stepX = MAX_SERVO_STEP;

    if (stepX < -MAX_SERVO_STEP)
      stepX = -MAX_SERVO_STEP;


    servoXPosition += stepX;


    // Mechanical limits
    servoXPosition =
      constrain(
        servoXPosition,
        SERVO_X_MIN,
        SERVO_X_MAX
      );


    servoX.write(servoXPosition);


    Serial.print(F(" X servo -> "));
    Serial.println(servoXPosition);
  }

  if (!yCentred) {

    float degreesY =
      ((float)errorY / Y_PIXELS_PER_DEGREE) * Y_DIRECTION;

    int stepY = (int)round(degreesY);


    if (stepY == 0) {

      if (errorY > 0) stepY = Y_DIRECTION;
      else             stepY = -Y_DIRECTION;
    }


    // Limit movement per update
    if (stepY > MAX_SERVO_STEP)
      stepY = MAX_SERVO_STEP;

    if (stepY < -MAX_SERVO_STEP)
      stepY = -MAX_SERVO_STEP;


    servoYPosition += stepY;


    // Mechanical limits
    servoYPosition =
      constrain(
        servoYPosition,
        SERVO_Y_MIN,
        SERVO_Y_MAX
      );


    servoY.write(servoYPosition);


    Serial.print(F(" Y servo -> "));
    Serial.println(servoYPosition);
  }
}

void stopSuppression() {

  digitalWrite(PIN_SUPPRESSION_LED, LOW);

  Serial.println(F("A3: Suppression LED OFF"));
}

void handleSerialInput(){
  if (!Serial.available()){
    return;
  }
  char command = Serial.read();

  if (command == 'T' || command == 't'){
    int x = Serial.parseInt();
    int y = Serial.parseInt();

  
    if (x < 0 || x > CAMERA_WIDTH || y < 0 || y > CAMERA_HEIGHT) {

      Serial.println(F("ERROR: Target outside camera dimensions"));

      Serial.print(F("Valid X = 0 to "));
      Serial.println(CAMERA_WIDTH);

      Serial.print(F("Valid Y = 0 to "));
      Serial.println(CAMERA_HEIGHT);

      return;
    }
    targetX = (int16_t)x;
    targetY = (int16_t)y;


    // Print what was received

    Serial.println(F(""));
    Serial.println(F("MANUAL TARGET COMMAND"));

    Serial.print(F("Target X = "));
    Serial.println(targetX);

    Serial.print(F("Target Y = "));
    Serial.println(targetY);


    // Start exactly the same targeting state used by A1

    changeState(STATE_TARGETING);

    return;
  }
  if (command == '0') {

    Serial.println(F(""));
    Serial.println(F("MANUAL COMMAND: STANDBY"));

    changeState(STATE_STANDBY);

    return;
  }
  if (command == 'S' || command == 's') {

    Serial.println(F(""));
    Serial.println(F("MANUAL COMMAND: SUPPRESSION START"));

    changeState(STATE_SUPPRESSING);

    return;
  }
  if (command == 'X' || command == 'x') {

    Serial.println(F(""));
    Serial.println(F("MANUAL COMMAND: SUPPRESSION STOP"));

    stopSuppression();

    changeState(STATE_READY);

    return;
  }
  if (command == 'P' || command == 'p') {

    Serial.println(F(""));
    Serial.println(F("--------------------------------"));
    Serial.println(F("A3 CURRENT STATUS"));
    Serial.println(F("--------------------------------"));

    Serial.print(F("State: "));
    Serial.println(stateName(currentState));

    Serial.print(F("Status code: "));
    Serial.println(currentStatus);

    Serial.print(F("Status: "));
    Serial.println(statusName(currentStatus));

    Serial.print(F("Target X: "));
    Serial.println(targetX);

    Serial.print(F("Target Y: "));
    Serial.println(targetY);

    Serial.print(F("Servo X: "));
    Serial.println(servoXPosition);

    Serial.print(F("Servo Y: "));
    Serial.println(servoYPosition);

    Serial.print(F("Suppression LED: "));

    if (digitalRead(PIN_SUPPRESSION_LED) == HIGH) {
      Serial.println(F("ON"));
    } else {
      Serial.println(F("OFF"));
    }

    Serial.println(F("--------------------------------"));

    return;
  }
  if (command == '\n' || command == '\r') {
    return;
  }
  Serial.print(F("Unknown command: "));
  Serial.println(command);

  Serial.println(F(""));
  Serial.println(F("Available commands:"));
  Serial.println(F("  T X Y = target coordinates"));
  Serial.println(F("  0     = standby"));
  Serial.println(F("  S     = suppression start"));
  Serial.println(F("  X     = suppression stop"));
  Serial.println(F("  P     = print status"));
  
}

const __FlashStringHelper* stateName(A3State state) {

  switch (state) {

    case STATE_STANDBY:
      return F("STANDBY");

    case STATE_TARGETING:
      return F("TARGETING");

    case STATE_READY:
      return F("READY");

    case STATE_SUPPRESSING:
      return F("SUPPRESSING");
  }

  return F("UNKNOWN");
}

const __FlashStringHelper* commandName(byte command) {

  switch (command) {

    case CMD_A3_STANDBY:
      return F("STANDBY");

    case CMD_A3_TARGET:
      return F("TARGET");

    case CMD_A3_SUPPRESS_START:
      return F("SUPPRESS_START");

    case CMD_A3_SUPPRESS_STOP:
      return F("SUPPRESS_STOP");
  }

  return F("UNKNOWN");
}

const __FlashStringHelper* statusName(byte status) {

  switch (status) {

    case A3_PARKED:
      return F("PARKED");

    case A3_READY:
      return F("READY");

    case A3_SUPPRESSING:
      return F("SUPPRESSING");

    case A3_SUPPRESSION_COMPLETE:
      return F("SUPPRESSION_COMPLETE");
  }

  return F("UNKNOWN");
}