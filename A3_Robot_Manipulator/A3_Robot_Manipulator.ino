/* Firewatch Arduino 3 Robot manipulator - Devlin
role of A3:
- controls the servos
- interpret the fire coordinates tuple sent from A1 to Servo angles
- center the webcam onto the fire
- activate the fire supression system (mock water pump) light turns on

Setup and Wiring:

-SDA to A4, SCL to A5 
-GND to connect to other boards

- SERVO_X direction is PIN 9
- SERVO_Y direction is PIN 10
- LASER is PIN 7
- (WATER PUMP) LED is PIN 8

I2C comms
Home position = PARKED
TARGET position = READY
Supression system on = SUPRESSING
Supression system off = SUPRESSION COMPLETE
*/

#include <Wire.h>
#include <Servo.h>


//I2C address between A3 and A1
const byte A3_ADDRESS = 9;

//Pin declerations
const int PIN_SERVO_X  = 9;   // pan
const int PIN_SERVO_Y  = 10;  // tilt
const int PIN_LASER    = 7;   // targeting motion indicator
const int WATER_PUMP = 8;   // mock "water" being ejected

//Home position of the Servo motors
const int PARK_X_ANGLE = 90;
const int PARK_Y_ANGLE = 100;

// Limitations of the Servos 
const int SERVO_X_MIN = 0;
const int SERVO_X_MAX = 180;

const int SERVO_Y_MIN = 80;
const int SERVO_Y_MAX = 180;
 //CAMERA MAPPING 
const float IMG_WIDTH  = 1920.0;
const float IMG_HEIGHT = 1080.0;
const float IMG_CX     = IMG_WIDTH  / 2.0;   // 960
const float IMG_CY     = IMG_HEIGHT / 2.0;   // 540
const float H_FOV      = 60.0;               // deg horizontal
const float V_FOV      = 60.0;               // deg vertical (approx)

// to prevent overshooting and speed control
const unsigned long STEP_INTERVAL_MS = 40;
const int           STEP_DEGREES     = 1;


Servo servoX;
Servo servoY;


 //I2C communication between A1 and A3
 

// Incoming commands from A1 
enum IncomingCommand : byte {
  CMD_NONE            = 0,
  CMD_STANDBY         = 10,
  CMD_TARGET          = 11,
  CMD_SUPPRESS_START  = 12,
  CMD_SUPPRESS_STOP   = 13
};

// Status codes sent back to A1 
enum OutgoingStatus : byte {
  STATUS_PARKED               = 20,
  STATUS_READY                = 21,
  STATUS_SUPPRESSING          = 22,
  STATUS_SUPPRESSION_COMPLETE = 23
};

// These four are written inside the I2C ISR and read in loop()
volatile IncomingCommand pendingCommand = CMD_NONE;  // what A1 last asked for
volatile int16_t         pendingX       = 0;         // target X (only for TARGET)
volatile int16_t         pendingY       = 0;         // target Y (only for TARGET)
volatile bool            commandReady   = false;     // true = new command waiting

// STANDBY state
byte currentStatus = STATUS_PARKED;

int targetXAngle  = PARK_X_ANGLE;
int targetYAngle  = PARK_Y_ANGLE;
int currentXAngle = PARK_X_ANGLE;
int currentYAngle = PARK_Y_ANGLE;

bool isMoving = false;
unsigned long lastStepTime = 0;

// Setup

void setup() {
  Serial.begin(9600);
  Serial.println(F("FIREWATCH A3 - ROBOTICS + SUPPRESSION"));

  pinMode(PIN_LASER, OUTPUT);
  pinMode(WATER_PUMP, OUTPUT);
  digitalWrite(PIN_LASER, LOW);
  digitalWrite(WATER_PUMP, LOW);

  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);
  servoX.write(180 - currentXAngle);
  servoY.write(currentYAngle);

  Wire.begin(A3_ADDRESS);
  Wire.onReceive(receiveFromA1);
  Wire.onRequest(sendToA1);

  currentStatus = STATUS_PARKED;
  Serial.println(F("A3 online - status PARKED"));
}

// Main Loop

void loop() {
  ProcessA1Command();   // act on whatever A1 sent us
  updateServos();            // keep stepping servos if moving
}

// Interpret Commands from A1

void receiveFromA1(int numBytes) {
  if (numBytes < 1) return; //commands sent from A1 are at least 1 byte in size

  byte rawCmd = Wire.read(); // read commands from A1

  if (rawCmd == CMD_TARGET) {
    // Format: [cmd][X lo][X hi][Y lo][Y hi] = 5 bytes total
    if (numBytes >= 5 && Wire.available() >= 4) {
      int16_t x, y; // data size of x and y tuple values
      Wire.readBytes((byte*)&x, sizeof(x));   // 2 bytes
      Wire.readBytes((byte*)&y, sizeof(y));   // 2 bytes
      pendingX = x; // storing x coordinates
      pendingY = y; // storing y coordinates
      pendingCommand = CMD_TARGET;
      commandReady   = true;
    } else {
    
      while (Wire.available()) Wire.read();
    }
  }
  else if (rawCmd == CMD_STANDBY) { // home position robot stays set but ready 
    pendingCommand = CMD_STANDBY;
    commandReady   = true;
  }
  else if (rawCmd == CMD_SUPPRESS_START) { // Supression command begins activate WATER PUMP (LED)
    pendingCommand = CMD_SUPPRESS_START;
    commandReady   = true;
  }
  else if (rawCmd == CMD_SUPPRESS_STOP) { // Supression command ends deactivate WATER PUMP (LED)
    pendingCommand = CMD_SUPPRESS_STOP;
    commandReady   = true;
  }
  else {
  // If unknown command wait for previous commands to be sent
    while (Wire.available()) Wire.read();
  }
}

// Send back current status to A1 i.e. one byte
void sendToA1() {
  Wire.write(currentStatus);
}

// Commands sent from A1 handling process

void ProcessA1Command() {
  if (!commandReady) return;

  
  IncomingCommand cmd;
  int16_t x, y;

  noInterrupts();
  cmd            = pendingCommand;
  x              = pendingX;
  y              = pendingY;
  pendingCommand = CMD_NONE;
  commandReady   = false;
  interrupts();

  // Standby command keep servos at PARKED home position or send servos to PARKED position
  if (cmd == CMD_STANDBY) {
    PARKED();
  } // Target coordinates sent move servos to TARGET position
  else if (cmd == CMD_TARGET) {
    TARGET(x, y);
  } // activate supression system LED ON
  else if (cmd == CMD_SUPPRESS_START) {
    SUPPRESS_START();
  } // deactivate Supression system LED OFF
  else if (cmd == CMD_SUPPRESS_STOP) {
    SUPPRESS_STOP();
  }
  // CMD_NONE: nothing to do
}



// STANDBY : park everything, all outputs off
void PARKED() {
  digitalWrite(PIN_LASER, LOW);
  digitalWrite(WATER_PUMP, LOW);

  targetXAngle = PARK_X_ANGLE;
  targetYAngle = PARK_Y_ANGLE;

  isMoving = (currentXAngle != targetXAngle) ||
             (currentYAngle != targetYAngle);

  currentStatus = STATUS_PARKED;

  Serial.print(F("STANDBY -> parking to X="));
  Serial.print(PARK_X_ANGLE);
  Serial.print(F(" Y="));
  Serial.println(PARK_Y_ANGLE);
}

// TARGET : map pixel coords to servo angles then start moving 
void TARGET(int16_t px, int16_t py) {
  // Pixel offset from image centre
  float xOffset = (float)px - IMG_CX;     // + right, - left
  float yOffset = IMG_CY - (float)py;     // + up,   - down

  // Linear FOV map to degrees
  float xAngle = PARK_X_ANGLE + (xOffset / IMG_WIDTH)  * H_FOV;
  float yAngle = PARK_Y_ANGLE + (yOffset / IMG_HEIGHT) * V_FOV;

  // Clamp to servo-safe range if coordinates beyond safe range
  if (xAngle < SERVO_X_MIN) xAngle = SERVO_X_MIN;
  if (xAngle > SERVO_X_MAX) xAngle = SERVO_X_MAX;
  if (yAngle < SERVO_Y_MIN) yAngle = SERVO_Y_MIN;
  if (yAngle > SERVO_Y_MAX) yAngle = SERVO_Y_MAX;

  targetXAngle = (int)round(xAngle);
  targetYAngle = (int)round(yAngle);

  // Laser ON while moving towards target
  digitalWrite(PIN_LASER, HIGH);

  // Status stays PARKED while moving; becomes READY when finished
  currentStatus = STATUS_PARKED;
  isMoving      = true;

  Serial.print(F("TARGET px=("));
  Serial.print(px);
  Serial.print(',');
  Serial.print(py);
  Serial.print(F(") -> servo X="));
  Serial.print(targetXAngle);
  Serial.print(F(" Y="));
  Serial.println(targetYAngle);
}

// SUPPRESS_START : "water pump" LED ON 
void SUPPRESS_START() {
  digitalWrite(WATER_PUMP, HIGH);
  currentStatus = STATUS_SUPPRESSING;
  Serial.println(F("SUPPRESS_START -> (water ejecting)"));
}

// SUPPRESS_STOP : LED OFF 
void SUPPRESS_STOP() {
  digitalWrite(WATER_PUMP, LOW);
  currentStatus = STATUS_SUPPRESSION_COMPLETE;
  Serial.println(F("SUPPRESS_STOP -> blue LED OFF"));
}

// Servo Motion control 

void updateServos() {
  if (!isMoving) return; // if not moving end 
  if (millis() - lastStepTime < STEP_INTERVAL_MS) return; // continue on previous step 
  lastStepTime = millis();

  if (currentXAngle < targetXAngle)      currentXAngle += STEP_DEGREES; // increase in degrees (to the right)
  else if (currentXAngle > targetXAngle) currentXAngle -= STEP_DEGREES; // decrease in degrees (to the left)
  servoX.write(180 - currentXAngle);

  if (currentYAngle < targetYAngle)      currentYAngle += STEP_DEGREES; // increase in degrees (upward)
  else if (currentYAngle > targetYAngle) currentYAngle -= STEP_DEGREES; // decrease in degrees (downward)
  servoY.write(currentYAngle);

  // Arrived at target coords?
  if (currentXAngle == targetXAngle && currentYAngle == targetYAngle) {
    isMoving = false; //stop moving reached target

    // Laser goes OFF the moment we reach the target
    digitalWrite(PIN_LASER, LOW);

    // Report READY, unless we are mid-suppression
    if (currentStatus != STATUS_SUPPRESSING &&
        currentStatus != STATUS_SUPPRESSION_COMPLETE) {
      currentStatus = STATUS_READY;
    }

    Serial.print(F("READY - aimed at X="));
    Serial.print(currentXAngle);
    Serial.print(F(" Y="));
    Serial.println(currentYAngle);
  }
}