// Perception Arduino (Arduino 2) Sketch for Automatic FIREWATCH
// ICTE4005 Robotics Assignment - Group 5
// Saf, Annie, Devlin - Curtin University 2026

// Purpose: 
// 1. Receives/sends commands to Mission Controller (Arduino 1) via I2C
// 2. Receives fire detection information from Python CV via USB Serial
// 3. Reads MLX90614 IR temperature sensor
// 4. Performs CV + thermal sensor fusion
// 5. Sends target/fire information to Mission Controller


// test mode switch for when Mission Control Arduino 1 is not connected
const bool TEST_MODE = false;


// add MLX90614 infrared temp sensor
#include <Wire.h>
#include <Adafruit_MLX90614.h>

// Constants
const byte A2_ADDRESS = 8; // I2C address for Arduino 2
const float CV_TRIGGER_THRESHOLD = 0.70; // CV confidence required before investigating target
const float TEMP_DIFFERENCE_THRESHOLD = 50.0; // in degC. How hot is potential fire

// numeric codes that Arduino 2 uses to tell Arduino 1 what is happening
const byte STATUS_NO_FIRE = 0;          // A2 has not detected a fire
const byte STATUS_POSSIBLE_FIRE = 1;    // CV has detected a possible fire
const byte STATUS_FIRE_CONFIRMED = 2;   // CV + thermal sensing have confirmed fire
const byte STATUS_VERIFYING = 3;        // A2 is currently verifying the target
const byte STATUS_STANDBY = 4;          // A2 is in standby mode


// VARIABLES

// from CV-fire_detection.py:
float cvConfidence = 0.0; // confidence reported by CV-fire_detection.py
int centreX = 0; // x-coord of centre of fire bounding box reported by CV-fire_detection.py
int centreY = 0; // y-coord of centre of fire bounding box reported by CV-fire_detection.py
long firePixels = 0; // area of fire bounding box (proxy for how large fire is) reported by CV-fire_detection.py
bool cvFireDetected = false; // saves whether CV-fire_detection.py currently detects fire

// from infrared temp sensor
Adafruit_MLX90614 mlx = Adafruit_MLX90614(); // create MLX90614 infrared temperature sensor object
bool thermalSensorOK = false; // stores whether infrared temperature sensor connected successfully
float ambientTemp = 0.0; // temperature of the surrounding environment measured by IR sensor
float objectTemp = 0.0; // temperature of object in front of IR sensor
float tempDifference = 0.0; // difference between object temperature and ambient temperature

// Firewatch states
enum PerceptionState {
  STANDBY,
  MONITORING,
  POSSIBLE_FIRE,
  VERIFYING,
  FIRE_CONFIRMED
};

PerceptionState state = STANDBY;

// Sensor fusion
float thermalConfidence = 0.0;
float fireConfidence = 0.0;

// I2C Communications - Commands from A1
volatile byte commandFromA1 = 0; // Command most recently received from Arduino 1
const byte CMD_HOT_WORK = 1; // HOT_WORK
const byte CMD_VERIFY_TARGET = 2; // VERIFY_TARGET
const byte CMD_STANDBY = 3; // STAND_BY



/////////////////////SET UP////////////////////////////////

void setup() {
  //USB SERIAL COMMS WITH CV PYTHON (AND WEBCAM)
  Serial.begin(115200); // must be same baud rate as CV-fire_detection.py
  Serial.println("PERCEPTION_READY"); // tell CV-fire_detection.py arduino is here

  // I2C COMMS WITH ARDUINO 1
  Wire.begin(A2_ADDRESS);
  Wire.onReceive(receiveFromA1);
  Wire.onRequest(sendToA1);

  // COMMS WITH IR TEMP SENSOR
  if (!mlx.begin()) {  // exception handling for infrared temp sensor
    Serial.println("WARNING: MLX90614 sensor not found - continuing without thermal");  // exception for when cant find IR temp sensor
      thermalSensorOK = false;
    }
    else {
        thermalSensorOK = true;
        Serial.println("IR temperature sensor connected");
  }
}

//////////////////////MAIN LOOP//////////////////////////////////

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


  // 3. Monitoring (CV looking for possible fire)
  if (state == MONITORING) {

    if (
      cvFireDetected && cvConfidence >= CV_TRIGGER_THRESHOLD) {
      state = POSSIBLE_FIRE; // this sends coordinates to A1 to get A3 to move robot to centre over fire
      Serial.println("POSSIBLE_FIRE_DETECTED");
    }
  }

  // 4. Verify target - A1 puts A2 into this state after A3 finishes aiming robot at target
  if (state == VERIFYING) {
    verifyTarget();
  }
}

/////////////COMMANDS FROM MISSION CONTROLLER - ARDUINO 1 ////////////

void processA1Command() {
  byte command;
  noInterrupts();
  command = commandFromA1;
  commandFromA1 = 0;
  interrupts();

  // HOT_WORK command (When Operator selects Hot Work mode)
    if (command == CMD_HOT_WORK) {
    state = MONITORING;
    cvFireDetected = false;
    cvConfidence = 0.0;
    
    Serial.println("START_CV");  // Tell Python to start CV monitoring
  }

  // VERIFY_TARGET command (When A3 Robotics finished centring on possible fire target)
    else if (command == CMD_VERIFY_TARGET) {
    state = VERIFYING;
    // Tell Python we need a fresh CV result
    Serial.println("VERIFY_CV");
  }

  // STANDBY command (before and after work)
  else if (command == CMD_STANDBY) {
    state = STANDBY;
    cvFireDetected = false;
    cvConfidence = 0.0;
    Serial.println("STOP_CV");
  }

}
void receiveFromA1(int numberOfBytes) { // to receive from A1
  if (Wire.available()) {
    commandFromA1 = Wire.read();
  }
}


///////////////////////SEND DATA TO MISSION CONTROLLER - ARDUINO 1//////////////

void sendToA1() {

// Possible fire detected - send CV target information to A1
  if (state == POSSIBLE_FIRE) {
    Wire.write(STATUS_POSSIBLE_FIRE); 

    Wire.write((byte*)&centreX, sizeof(centreX));   // Send X coordinate as 2 bytes
    Wire.write((byte*)&centreY, sizeof(centreY));     // Send Y coordinate as 2 bytes
    Wire.write((byte*)&cvConfidence, sizeof(cvConfidence));     // Send CV confidence as 4 bytes
  }


// Fire confirmed after robotics move and thermal sensor with CV sensor fusion
  else if (state == FIRE_CONFIRMED) {
    Wire.write(STATUS_FIRE_CONFIRMED);
    Wire.write((byte*)&fireConfidence, sizeof(fireConfidence)); // Send final fire confidence
  }

// Not sure if needed but for time it takes to calculate CV + Thermal confidence 
  else if (state == VERIFYING) {
    Wire.write(STATUS_VERIFYING);
  }

// Normal monitoring
  else if (state == MONITORING) {
    Wire.write(STATUS_NO_FIRE);
  }

// Standby - pretty much off
  else {
    Wire.write(STATUS_STANDBY);
  }
}



/////////// READ COMPUTER VISION MESSAGE/////////////
void readCVMessage(String message) {

  // NO FIRE
  if (message == "NO_FIRE") {
    cvFireDetected = false;
    cvConfidence = 0.0;
    return;
  }

  // FIRE DETECTED  // FIRE,0.91,360,250,24000
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


////////////// THERMAL VERIFICATION + SENSOR FUSION/////////////
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

  // Read thermal sensor
  ambientTemp = mlx.readAmbientTempC();  // temperature of the MLX90614 sensor chip itself
  objectTemp = mlx.readObjectTempC(); // the temperature of the surface/object it is pointing at
  tempDifference = objectTemp - ambientTemp; // how hot the possible fire is
  Serial.print("THERMAL | Ambient: ");
  Serial.print(ambientTemp);
  Serial.print(" C | Object: ");
  Serial.print(objectTemp);
  Serial.println(" C");


///// to change later to better calculation////
  if (tempDifference >= TEMP_DIFFERENCE_THRESHOLD) { // if > 50 deg C then 100% chance of fire
    thermalConfidence = 1.0;
  }
  else {
    thermalConfidence = 0.0;
  }

// TEMPORARY sensor fusion-  Equal weighting: 50% CV  50% thermal
  fireConfidence = (cvConfidence * 0.5) + (thermalConfidence * 0.5);


  // Debugging - temp
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



// Final fire decision
  if (fireConfidence >= 0.70) {
    state = FIRE_CONFIRMED;
    Serial.println("FIRE_CONFIRMED");
  }
  else {
    state = MONITORING;
    cvFireDetected = false;
    cvConfidence = 0.0;
    Serial.println("FIRE_NOT_CONFIRMED");

    delay(1000); // adding delay because camera detecting fire faster than A1 polling
  }
}



    // TESTING ARDUINO THROUGH PYTHON
    // Tell Python that the CV data was successfully received and parsed by the Arduino
    // Serial.println("CV_DATA_OK");


    // // TESTING ARDUINO ONLY - print parsed values back to terminal
    // Serial.println("--- CV DATA RECEIVED ---");
    // Serial.print("Fire detected: ");
    // Serial.println(cvFireDetected);

    // Serial.print("CV Confidence: ");
    // Serial.println(cvConfidence);

    // Serial.print("Centre X: ");
    // Serial.println(centreX);

    // Serial.print("Centre Y: ");
    // Serial.println(centreY);

    // Serial.print("Fire Pixels: ");
    // Serial.println(firePixels);



