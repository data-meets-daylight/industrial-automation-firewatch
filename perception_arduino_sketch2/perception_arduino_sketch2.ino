// Perception Arduino Sketch for Automatic FIREWATCH
// ICTE4005 Robotics Assignment - Group 5
// Saf, Annie, Devlin - Curtin University 2026

// Purpose: 
// 1. Recieves and sends comms to mission controller arduino via I2C
// 2. Recieves fire detection information from python computer vision program running on laptop 
// 3. Recieves fire detection information from thermal IR sensor
// 4. Calculates fire confidence and coordinates to send to mission controller 

// add MLX90614 infrared temp sensor
#include <Wire.h>
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// VARIABLES

// from CV-fire_detection.py:
float cvConfidence = 0.0; // confidence reported by CV-fire_detection.py
int centreX = 0; // x-coord of centre of fire bounding box reported by CV-fire_detection.py
int centreY = 0; // y-coord of centre of fire bounding box reported by CV-fire_detection.py
long firePixels = 0; // area of fire bounding box (proxy for how large fire is) reported by CV-fire_detection.py
bool cvFireDetected = false; // saves whether CV-fire_detection.py currently detects fire

// from infrared temp sensor
float ambientTemp = 0.0;
float objectTemp = 0.0;


void setup() {
  Serial.begin(115200); // must be same baud rate as CV-fire_detection.py
  Serial.println("PERCEPTION_READY"); // tell CV-fire_detection.py arduino is here

  if (!mlx.begin()) {  // exception handling for infrared temp sensor
    Serial.println("ERROR: MLX90614 sensor not found");
    while (1);
  }
  Serial.println("IR temperature sensor connected");
}

void loop() {
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n'); // read everything up to newline. 
    // Example of message: FIRE,0.91,360,250,24000 (FIRE | Confidence: 0.91 | Pixels: 24000 | Centre: (360, 250))
    message.trim();
    readCVMessage(message); // send message to parsing function
  }
  // IR Temp Sensor 
  ambientTemp = mlx.readAmbientTempC();
  objectTemp = mlx.readObjectTempC();
  float tempDifference = objectTemp - ambientTemp;
  // Serial.print("Ambient: ");
  // Serial.print(ambientTemp);
  // Serial.print(" C | Object: ");
  // Serial.print(objectTemp);
  // Serial.println(" C");'
  Serial.print("Object Temp - Ambient Temp = ");
  Serial.print(tempDifference);
  Serial.println(" C");
}

void readCVMessage(String message) {  // parsing function
  // Case 1: No fire detected
  if (message == "NO_FIRE") {
    cvFireDetected = false;
    cvConfidence = 0.0;

    //TESTING ARDUINO ONLY - print parsed values back to terminal
    // Serial.println("CV reports NO FIRE");   

    // TESTING ARDUINO THROUGH PYTHON
    // Tell Python that the CV data was successfully received and parsed by the Arduino
    Serial.println("NO_FIRE_OK");
    return;
  }

  // Case 2: CV-fire_detection.py detects fire
  if (message.startsWith("FIRE,")) {
    cvFireDetected = true;
    //split message by commas
    int comma1 = message.indexOf(',');
    int comma2 = message.indexOf(',', comma1 + 1);
    int comma3 = message.indexOf(',', comma2 + 1);
    int comma4 = message.indexOf(',', comma3 + 1);

    // extract CV confidence
    cvConfidence =
      message.substring(
        comma1 + 1,
        comma2
      ).toFloat();
      
    // Extract X coord
    centreX =
      message.substring(
        comma2 + 1,
        comma3
      ).toInt();

    // Extract Y coord
    centreY =
      message.substring(
        comma3 + 1,
        comma4
      ).toInt();

    // Extract bounding box size
    firePixels =
      message.substring(
        comma4 + 1
      ).toInt();

    // TESTING ARDUINO THROUGH PYTHON
    // Tell Python that the CV data was successfully received and parsed by the Arduino
    Serial.println("CV_DATA_OK");


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


  }
}
