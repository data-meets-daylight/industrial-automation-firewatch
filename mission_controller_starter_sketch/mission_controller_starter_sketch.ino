// Starter sketch for Annie to look at to work with Safs Sketch

// PURPOSE:
// Arduino 1 is the Mission Controller.
// It tells Arduino 2 (Perception) when to monitor/verify.
// It receives fire status, coordinates and confidence from Arduino 2.
// It will also communicate with Arduino 3 (Robotics).

#include <Wire.h>  // Allows Arduino 1 to communicate with A2/A3 using I2C


// To talk to ARDUINO 2 - PERCEPTION
const byte A2_ADDRESS = 8;  // I2C address of Saf's Perception Arduino


// Commands A1 can SEND to A2.
// These numbers MUST match the numbers in Saf's A2 sketch.

const byte CMD_HOT_WORK = 1;       // Tell A2 to start CV fire monitoring
const byte CMD_VERIFY_TARGET = 2;  // Tell A2 robot is aimed and to verify fire
const byte CMD_STANDBY = 3;        // Tell A2 to stop monitoring


// Status codes A1 can RECEIVE from A2.
// These numbers MUST also match Saf's A2 sketch.
const byte STATUS_NO_FIRE = 0;          // A2 is monitoring but no fire detected
const byte STATUS_POSSIBLE_FIRE = 1;    // CV has detected a possible fire
const byte STATUS_FIRE_CONFIRMED = 2;   // CV + thermal verification confirmed fire
const byte STATUS_VERIFYING = 3;        // A2 is currently verifying target
const byte STATUS_STANDBY = 4;          // A2 is currently in standby


// INFORMATION RECEIVED FROM A2
byte perceptionStatus = STATUS_STANDBY;// Stores the latest status received from A2
int targetX = 0;// X coordinate of centre of fire in camera image
int targetY = 0;// Y coordinate of centre of fire in camera image
float cvConfidence = 0.0;// Confidence from YOLO computer vision, e.g. 0.86 = 86%
float fireConfidence = 0.0;// Final confidence calculated by A2 using CV + thermal sensor

// MISSION CONTROLLER STATES
enum MissionState {
  STANDBY,  // System waiting for operator to select hot work
  MONITORING,  // A2/Python are monitoring for fire
  TARGETING,  // Possible fire found and A3 is moving towards coordinates
  VERIFYING,  // A3 is aimed and A2 is verifying the suspected fire
  FIRE_CONFIRMED,  // Fire has been confirmed
  SUPPRESSING  // Suppression system operating
};

MissionState state = STANDBY;



// SETUP
void setup() {
  Serial.begin(115200);  // Serial Monitor used so we can see what Mission Controller is doing
  Wire.begin();  // Start A1 as the I2C controller/master.  // A2 uses Wire.begin(8), because A2 has address 8.
  Serial.println("MISSION CONTROL READY");  // Lets us know A1 has successfully started
}


// MAIN LOOP
void loop() {
  // STANDBY
 if (state == STANDBY) {

    // TODO ANNIE:    // Put whatever causes HOT WORK mode to start here.

    // When HOT WORK is selected, do:
      // sendCommandToA2(CMD_HOT_WORK);
    // state = MONITORING;
  }

// MONITORING
  else if (state == MONITORING) {  
    checkPerceptionArduino();    // Ask A2 whether CV has detected anything
    if (perceptionStatus == STATUS_POSSIBLE_FIRE) {
      Serial.println("MISSION CONTROL: POSSIBLE FIRE");
      Serial.print("Target X: ");
      Serial.println(targetX);
      Serial.print("Target Y: ");
      Serial.println(targetY);
      Serial.print("CV Confidence: ");
      Serial.println(cvConfidence);

      // At this point targetX and targetY contain the coordinates
      // that came from Python → A2 → A1.

      // TODO ANNIE + DEVLIN:
      // Send targetX and targetY from A1 to A3 here.
      // maybe sendTargetToA3(targetX, targetY);

      state = TARGETING;
      // Mission Controller now waits for A3 to aim at the fire
    }
  }


  // TARGETING
  else if (state == TARGETING) {

    // TODO ANNIE + DEVLIN:
    // Check whether A3 has finished moving to targetX,targetY.

    // Eventually this will be something like:
    //
    // if (A3 is ready) {
    // sendCommandToA2(CMD_VERIFY_TARGET);
    //  state = VERIFYING;
    // }


    // IMPORTANT:
    // Do NOT tell A2 to verify until A3 has finished aiming AS the thermal sensor needs to be pointing towards the suspected fire when verification happens.
  }


 // VERIFYING
  else if (state == VERIFYING) {
    checkPerceptionArduino();    // Keep asking A2 for its verification result
    if (perceptionStatus == STATUS_VERIFYING) {

      // A2 is still doing its CV + thermal verification.
      // Nothing else needs to happen yet.
    }

    else if (perceptionStatus == STATUS_FIRE_CONFIRMED) {
      Serial.println("MISSION CONTROL: FIRE CONFIRMED");
      Serial.print("Final Fire Confidence: ");
      Serial.println(fireConfidence);
      state = FIRE_CONFIRMED;      // Mission Controller can now start alarm/suppression
    }


    else if (perceptionStatus == STATUS_NO_FIRE) {
      Serial.println("MISSION CONTROL: FIRE NOT CONFIRMED");
      state = MONITORING;      // Go back to normal fire monitoring
    }
  }

  // FIRE CONFIRMED
  else if (state == FIRE_CONFIRMED) {

    // TODO ANNIE:
    // Alarm/beacon logic goes here.


    // TODO ANNIE + DEVLIN:
    // Send suppression command to A3 here.
    state = SUPPRESSING;
  }

  // SUPPRESSING
  else if (state == SUPPRESSING) {

    // TODO ANNIE + DEVLIN:
    // Suppression sequence goes here.

    // what A1 should do when suppression is finished:
    // re-check fire?
    // return to monitoring?
    // return to standby?
  }
}

// SEND COMMAND TO PERCEPTION ARDUINO
void sendCommandToA2(byte command) {
  Wire.beginTransmission(A2_ADDRESS);  // Start an I2C message addressed specifically to Arduino 2
  Wire.write(command);
  // Send one command byte either:
     // 1 = HOT_WORK or  2 = VERIFY_TARGET or  3 = STANDBY

  Wire.endTransmission();
  // Finish and actually send the I2C message
}


// CHECK PERCEPTION ARDUINO
void checkPerceptionArduino() {

  Wire.requestFrom(A2_ADDRESS, 9);
  // Ask Arduino 2 for information.
  //
  // 9 bytes is enough for the largest current A2 message:
  //
  // status       = 1 byte
  // targetX      = 2 bytes
  // targetY      = 2 bytes
  // cvConfidence = 4 bytes
  //
  // TOTAL = 9 bytes


  if (Wire.available() < 1) {
    return;    // If A2 did not send anything, leave the function
  }

  perceptionStatus = Wire.read();
  // The FIRST byte from A2 is ALWAYS its status.
  //
  // 0 = NO_FIRE
  // 1 = POSSIBLE_FIRE
  // 2 = FIRE_CONFIRMED
  // 3 = VERIFYING
  // 4 = STANDBY



  // POSSIBLE FIRE MESSAGE
  if (perceptionStatus == STATUS_POSSIBLE_FIRE) {

    // If A2 says POSSIBLE_FIRE, it sends another 8 bytes:
    //
    // targetX      = 2 bytes
    // targetY      = 2 bytes
    // cvConfidence = 4 bytes


    if (Wire.available() >= 8) {      // Make sure all 8 remaining bytes actually arrived
      Wire.readBytes(
        (byte*)&targetX,
        sizeof(targetX)
      );      // Read the next 2 bytes and rebuild targetX

      Wire.readBytes(
        (byte*)&targetY,
        sizeof(targetY)
      );      // Read the next 2 bytes and rebuild targetY

      Wire.readBytes(
        (byte*)&cvConfidence,
        sizeof(cvConfidence)
      );      // Read the next 4 bytes and rebuild the float containing YOLO's confidence


      Serial.println("A2: POSSIBLE FIRE");

      Serial.print("X = ");
      Serial.println(targetX);

      Serial.print("Y = ");
      Serial.println(targetY);

      Serial.print("CV Confidence = ");
      Serial.println(cvConfidence);
    }
  }


  // FIRE CONFIRMED MESSAGE
  else if (perceptionStatus == STATUS_FIRE_CONFIRMED) {
    // When fire is confirmed A2 sends: status = already read above
    // fireConfidence = 4 bytes

    if (Wire.available() >= 4) {

      Wire.readBytes(
        (byte*)&fireConfidence,
        sizeof(fireConfidence)
      );      // Rebuild the final confidence float sent by A2

      Serial.println("A2: FIRE CONFIRMED");
      Serial.print("Fire Confidence = ");
      Serial.println(fireConfidence);
    }
  }


  // VERIFYING
  else if (perceptionStatus == STATUS_VERIFYING) {

    Serial.println("A2: VERIFYING");
    // A2 has received VERIFY_TARGET and is doing its
    // second CV + thermal check
  }

  // NO FIRE
  else if (perceptionStatus == STATUS_NO_FIRE) {

    // A2 is actively monitoring but hasn't found a fire.
    //
    // We don't continuously Serial.print here,
    // otherwise the Serial Monitor will get flooded.
  }


  // STANDBY
  else if (perceptionStatus == STATUS_STANDBY) {

    // A2 is currently waiting in standby.
  }
}