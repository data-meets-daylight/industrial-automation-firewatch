/*
=================================================================
INDUSTRIAL AUTOMATED FIREWATCH (IAF)
ICTE4005 Robotics Project - Group 5
Curtin University, 2026

Team:
Saf Flatters, Annie (Annabelle) Lewkowski, Devlin MacGlip

MODULE:
Arduino 2 (A2) - Perception

PURPOSE:
A2 manages the perception system for the IAF.
- Communicates with A1 (Mission Control) via I2C
- Receives fire detection information from Python CV via USB Serial
- Reads the MLX90614 IR temperature sensor
- Performs CV + thermal sensor fusion to verify possible fires
- Sends target coordinates and fire status to A1

WIRING:

I2C Communication:
SDA (A4) -> A1 SDA
SCL (A5) -> A1 SCL
GND      -> Common GND between all Arduino boards

MLX90614 IR Temperature Sensor:
SDA -> A4
SCL -> A5
VCC -> 5V
GND -> GND

Computer Vision:
A2 -> Laptop via USB
Serial baud rate -> 115200
Python script -> CV-fire_detection.py


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

// TEST MODE
// Allows A2 to be tested without A1 connected.
const bool TEST_MODE = false;

// LIBRARIES
#include <Wire.h>
#include <Adafruit_MLX90614.h>

// A2 I2C ADDRESS
const byte A2_ADDRESS = 8; // I2C address for Arduino 2

// FIRE DETECTION THRESHOLDS
const float CV_TRIGGER_THRESHOLD = 0.70; // CV confidence required before investigating target
const float TEMP_DIFFERENCE_THRESHOLD = 20.0; // in degC. How hot is potential fire

// STATUS SENT TO A1
const byte STATUS_NO_FIRE = 0;          // A2 has not detected a fire
const byte STATUS_POSSIBLE_FIRE = 1;    // CV has detected a possible fire
const byte STATUS_FIRE_CONFIRMED = 2;   // CV + thermal sensing have confirmed fire
const byte STATUS_VERIFYING = 3;        // A2 is currently verifying the target
const byte STATUS_STANDBY = 4;          // A2 is in standby mode

// COMPUTER VISION DATA
// Received from CV-fire_detection.py
float cvConfidence = 0.0; // confidence reported by CV-fire_detection.py
int centreX = 0; // x-coord of centre of fire bounding box reported by CV-fire_detection.py
int centreY = 0; // y-coord of centre of fire bounding box reported by CV-fire_detection.py
long firePixels = 0; // area of fire bounding box (proxy for how large fire is) reported by CV-fire_detection.py
bool cvFireDetected = false; // saves whether CV-fire_detection.py currently detects fire

// THERMAL SENSOR DATA
Adafruit_MLX90614 mlx = Adafruit_MLX90614(); // create MLX90614 infrared temperature sensor object
bool thermalSensorOK = false; // stores whether infrared temperature sensor connected successfully
float ambientTemp = 0.0; // temperature of the surrounding environment measured by IR sensor
float objectTemp = 0.0; // temperature of object in front of IR sensor
float tempDifference = 0.0; // difference between object temperature and ambient temperature

// PERCEPTION STATES
enum PerceptionState {
  STANDBY,        // SEQ 01 - System startup / STANDBY
  MONITORING,     // SEQ 03 - Monitor for fire
  POSSIBLE_FIRE,  // SEQ 04 - Possible fire detected
  VERIFYING,      // SEQ 06 - Verify fire with CV + thermal
  FIRE_CONFIRMED  // SEQ 07B - Fire CONFIRMED
};

PerceptionState state = STANDBY;

// SENSOR FUSION
float thermalConfidence = 0.0;
float fireConfidence = 0.0;

// COMMANDS RECEIVED FROM A1
volatile byte commandFromA1 = 0; // Command most recently received from Arduino 1
const byte CMD_HOT_WORK = 1; // HOT_WORK
const byte CMD_VERIFY_TARGET = 2; // VERIFY_TARGET
const byte CMD_STANDBY = 3; // STAND_BY



// SETUP
// SEQ 01 - SYSTEM STARTUP / STANDBY
// Runs once when A2 is powered on or reset.
void setup() {

  // Start USB Serial communication with Python CV (and Webcam)
  Serial.begin(115200); // must be same baud rate as CV-fire_detection.py
  Serial.println("PERCEPTION_READY"); // tell CV-fire_detection.py arduino is here

  // Start A2 with I2C comms (with A1)
  Wire.begin(A2_ADDRESS);
  Wire.onReceive(receiveFromA1);
  Wire.onRequest(sendToA1);

  // Start MLX90614 thermal sensor
  if (!mlx.begin()) {  // exception handling for infrared temp sensor
    Serial.println("WARNING: MLX90614 sensor not found - continuing without thermal");  // exception for when cant find IR temp sensor
      thermalSensorOK = false;
    }
    else {
        thermalSensorOK = true;
        Serial.println("IR temperature sensor connected");
  }
}

// MAIN LOOP
// Runs continuously and manages A1 commands, CV data and fire detection.
void loop() {
  // 1. Process command received from A1
  processA1Command(); 

  // 2. Read incoming Computer Vision data
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n'); // read everything up to newline. 
    // Example of message: FIRE,0.91,360,250,24000 (FIRE | Confidence: 0.91 | Pixels: 24000 | Centre: (360, 250))
    message.trim();

    // TEST MODE - fake commands normally sent by Arduino 1
    if (TEST_MODE && message == "HOT_WORK") {commandFromA1 = CMD_HOT_WORK;}
    else if (TEST_MODE && message == "VERIFY_TARGET") {commandFromA1 = CMD_VERIFY_TARGET;}
    else if (TEST_MODE && message == "STANDBY") {commandFromA1 = CMD_STANDBY;}
    // Otherwise it is a message from Python CV
    else {
      readCVMessage(message); // send message to parsing function
    }
  }


// SEQ 03 - MONITOR FOR FIRE
  // Check whether CV has detected a fire above the trigger threshold.
  if (state == MONITORING) {

// SEQ 04 - POSSIBLE FIRE DETECTED
    if (cvFireDetected && cvConfidence >= CV_TRIGGER_THRESHOLD) {
      state = POSSIBLE_FIRE; // this sends coordinates to A1 to get A3 to move robot to centre over fire
      Serial.println("POSSIBLE_FIRE_DETECTED");
    }
  }

// SEQ 06 - VERIFY FIRE WITH CV + THERMAL
  // A1 enters this state after A3 has aimed at the possible fire.
  if (state == VERIFYING) {
    verifyTarget();
  }
}

// COMMANDS FROM A1
// Processes the most recent command received from Mission Control.
void processA1Command() {
  byte command;
  // Safely copy command received by I2C interrupt
  noInterrupts();
  command = commandFromA1;
  commandFromA1 = 0;
  interrupts();

// SEQ 02 + 03 - START HOT WORK / MONITOR FOR FIRE
    if (command == CMD_HOT_WORK) {
    state = MONITORING;
    cvFireDetected = false;
    cvConfidence = 0.0;
    
    Serial.println("START_CV");  // Tell Python to start CV monitoring
  }

// SEQ 06 - VERIFY FIRE
  // A3 has finished aiming at the possible fire target.
    else if (command == CMD_VERIFY_TARGET) {
    state = VERIFYING;

    // Tell Python we need a fresh CV result
    Serial.println("VERIFY_CV");
  }

// SEQ 01 / 10 - STANDBY
  // Stop CV when the system is inactive or A3 is returning to park.
  else if (command == CMD_STANDBY) {
    state = STANDBY;
    cvFireDetected = false;
    cvConfidence = 0.0;
    Serial.println("STOP_CV");
  }
}


// RECEIVE COMMAND FROM A1
// Called automatically when A1 sends data to A2 over I2C.
void receiveFromA1(int numberOfBytes) { // to receive from A1
  if (Wire.available()) {
    commandFromA1 = Wire.read();
  }
}


// SEND DATA TO A1
// Called when A1 requests A2's current status over I2C.
void sendToA1() {
// SEQ 04 - POSSIBLE FIRE DETECTED
  // Send target coordinates and CV confidence to A1.
  if (state == POSSIBLE_FIRE) {
    Wire.write(STATUS_POSSIBLE_FIRE);
    Wire.write((byte*)&centreX, sizeof(centreX));
    Wire.write((byte*)&centreY, sizeof(centreY));
    Wire.write((byte*)&cvConfidence, sizeof(cvConfidence));
  }
// SEQ 07B - FIRE CONFIRMED
  // Send final sensor-fusion fire confidence to A1.
  else if (state == FIRE_CONFIRMED) {
    Wire.write(STATUS_FIRE_CONFIRMED);
    Wire.write((byte*)&fireConfidence, sizeof(fireConfidence));
  }
// SEQ 06 - VERIFYING
  // Tell A1 that A2 is still verifying the target.
  else if (state == VERIFYING) {
    Wire.write(STATUS_VERIFYING);
  }
// SEQ 03 - MONITORING
  // No possible fire currently detected.
  else if (state == MONITORING) {
    Wire.write(STATUS_NO_FIRE);
  }
// SEQ 01 / 10 - STANDBY
  else {
    Wire.write(STATUS_STANDBY);
  }
}


// READ COMPUTER VISION MESSAGE
// Parses fire detection data received from CV-fire_detection.py.
void readCVMessage(String message) {
  // No fire currently detected by CV
  if (message == "NO_FIRE") {
    cvFireDetected = false;
    cvConfidence = 0.0;
    return;
  }

  // Fire detected by CV
  // Expected format: FIRE,0.91,360,250,24000
  if (message.startsWith("FIRE,")) {
    cvFireDetected = true;
    //split message by commas
    int comma1 = message.indexOf(',');
    int comma2 = message.indexOf(',', comma1 + 1);
    int comma3 = message.indexOf(',', comma2 + 1);
    int comma4 = message.indexOf(',', comma3 + 1);

    // CV confidence
    cvConfidence =
      message.substring(comma1 + 1,comma2
      ).toFloat();

    // X coordinate
    centreX =
      message.substring(
        comma2 + 1,
        comma3
      ).toInt();

    // Y coordinate
    centreY =
      message.substring(
        comma3 + 1,
        comma4
      ).toInt();

    // Bounding box size
    firePixels =
      message.substring(
        comma4 + 1
      ).toInt();
  }
}


// THERMAL VERIFICATION + SENSOR FUSION
// SEQ 06 - Verify fire with CV + thermal
// Combines fresh CV confidence with thermal sensor data
// to determine whether the possible fire is genuine.
void verifyTarget() {
  // Serial.println("DEBUG 1: Entered verifyTarget()");
  if (!thermalSensorOK) {    // Cannot perform sensor fusion without IR sensor
    fireConfidence = 0.0;   // if IR is 0
    state = MONITORING;
    cvFireDetected = false;
    cvConfidence = 0.0;   
    Serial.println("FIRE_NOT_CONFIRMED");
    return;
  }

  // Read ambient and target temperatures
  ambientTemp = mlx.readAmbientTempC();  // temperature of the MLX90614 sensor chip itself
  objectTemp = mlx.readObjectTempC(); // the temperature of the surface/object it is pointing at
  tempDifference = objectTemp - ambientTemp; // how hot the possible fire is
  Serial.print("THERMAL | Ambient: ");
  Serial.print(ambientTemp);
  Serial.print(" C | Object: ");
  Serial.print(objectTemp);
  Serial.println(" C");

  // Convert thermal result into a simple confidence value
  if (tempDifference >= TEMP_DIFFERENCE_THRESHOLD) { // if > threshold deg C then 100% chance of fire
    thermalConfidence = 1.0;
  }
  else {
    thermalConfidence = 0.0;
  }

  // TEMPORARY sensor fusion-  Equal weighting: 50% CV  50% thermal
  fireConfidence = (cvConfidence * 0.5) + (thermalConfidence * 0.5);

  // Debugging 
  Serial.print("CV Confidence: ");
  Serial.println(cvConfidence);
  Serial.print("Object Temp: ");
  Serial.println(objectTemp);
  Serial.print("Ambient Temp: ");
  Serial.println(ambientTemp);
  Serial.print("Object Temp - Ambient Temp = ");
  Serial.println(tempDifference);
  Serial.print("Thermal Confidence: ");
  Serial.println(thermalConfidence);
  Serial.print("Fire Confidence: ");
  Serial.println(fireConfidence);


// SEQ 07B - FIRE CONFIRMED
  if (fireConfidence >= 0.70) {
    state = FIRE_CONFIRMED;
    Serial.println("FIRE_CONFIRMED");
  }
// SEQ 07A - FIRE NOT CONFIRMED
  else {
    state = MONITORING;
    cvFireDetected = false;
    cvConfidence = 0.0;
    Serial.println("FIRE_NOT_CONFIRMED");
    delay(1000); // Prevent CV detecting the same target before A1 polls the result
  }
}