# Industrial Automated Firewatch (IAF)

**ICTE4005 Robotics Project - Group 5**  
Curtin University, 2026

**Team:** Saf Flatters, Annie (Annabelle) Lewkowski, Devlin MacGlip

## Project Overview

The Industrial Automated Firewatch (IAF) is a prototype robotic system designed to automate parts of the fire-watch process used during hot work.  
The system uses three Arduino modules and a Python computer vision program to detect a possible fire, aim a camera and suppression system at the target, verify the fire using computer vision and thermal sensing, and activate a mock fire suppression system.

### System Modules

**Arduino 1 (A1) - Mission Control**
- Main system controller
- Runs the system state machine
- Communicates with A2 and A3 using I2C
- Controls the Hot Work button, status lights and alarm
- Coordinates detection, verification and suppression

**Arduino 2 (A2) - Perception**
- Communicates with the Python computer vision program
- Reads the MLX90614 infrared temperature sensor
- Combines computer vision and thermal information
- Sends fire status and target coordinates to A1

**Arduino 3 (A3) - Robotics**
- Controls the pan and tilt servos
- Aims the webcam and suppression system at the detected fire
- Controls the laser and mock water pump
- Reports robot status to A1

**Python Computer Vision**
- Captures the webcam feed using OpenCV
- Uses a trained YOLO model to detect fire
- Calculates detection confidence, target coordinates and bounding-box area
- Sends computer vision data to A2 through USB Serial


## System Communication

```text
Webcam
   |
   v
Python / YOLO
   |
   | USB Serial
   v
A2 - Perception
   |
   | I2C
   v
A1 - Mission Control
   |
   | I2C
   v
A3 - Robot Manipulator
   |
   +--> Pan/Tilt Servos
   +--> Laser
   +--> Mock Water Pump
```


# Starting the Firewatch System

## 1. Connect the hardware

Connect the **Perception Arduino (A2)** to the computer using USB.  
Connect the **webcam** to the computer.  
Connect the **Mission Controller Arduino (A1)** to the computer using USB.  
Ensure A1, A2 and A3 are connected through I2C and share a common ground.  
A3 does not require its own USB connection during normal operation.  

## 2. Check the Python Serial port

Check which COM port has been assigned to the Perception Arduino.  
In `CV-fire_detection.py`, make sure the following line uses the correct COM port:  
```python
port="COM4"
```

The Python baud rate must remain:

```python
baudrate=115200
```


## 3. Start the computer vision program
Open a terminal in the project folder.  
If required, activate the Python virtual environment:  
```text
.\.venv\Scripts\Activate.ps1
```

Start computer vision:  
```text
python CV-fire_detection.py
```

Wait for:  
```text
Connected to Perception Arduino
Arduino: PERCEPTION_READY
Arduino: IR temperature sensor connected
Arduino: START_CV
```
The webcam window should now be open and the computer vision system should be ready.

## 4. Open the Mission Controller Serial Monitor

Open the Arduino Serial Monitor for **A1 - Mission Control**.  
Set the Serial Monitor to:  
```text
9600 baud
```

Follow the instructions displayed in the Serial Monitor.  
The system can be started using the physical **red Hot Work button**.  
Pressing the button from STANDBY starts HOT WORK monitoring.  

## 5. Present a fire target
The demonstration target must:  
- Visually resemble fire strongly enough for the YOLO model to detect it.
- Produce enough heat for the infrared temperature sensor to detect a thermal difference.  
Both computer vision and thermal sensing are used during fire verification.  
The system should initially detect the target using the webcam. A3 will then aim the camera and thermal sensor toward the target before A2 performs verification.  

# System Sequence

## SEQ 01 - System startup / STANDBY
A1, A2 and A3 initialise.  
A3 moves to its tested PARKED position.  
A2 establishes communication with the Python computer vision program and initialises the infrared temperature sensor.  
The system waits for the operator to begin HOT WORK.  

## SEQ 02 - Operator starts HOT WORK
The operator presses the physical Hot Work button on A1.  
A1 enters HOT_WORK and commands A2 to begin monitoring.  

## SEQ 03 - Monitor for fire
A2 tells Python to start computer vision monitoring.  
The webcam continuously provides images to the YOLO fire detection model.  
Python sends fire detection information to A2, including:  
- CV confidence
- Target X coordinate
- Target Y coordinate
- Bounding-box area of fire
If no fire is detected, the system remains in HOT_WORK.  

## SEQ 04 - Possible fire detected  
When the computer vision confidence exceeds the required threshold, A2 reports a possible fire to A1.  
A2 sends the target coordinates and CV confidence to Mission Control.  

## SEQ 05 - Aim robot at target
A1 sends the target X/Y coordinates to A3.  
A3 converts the image coordinates into pan and tilt servo angles.  
The robot moves gradually toward the target.  
Once the servos reach the target position, A3 reports `READY` to A1.  

## SEQ 06 - Verify fire with CV + thermal
Once A3 is aimed at the possible fire, A1 asks A2 to verify the target.  
A2 obtains fresh computer vision information and reads the MLX90614 infrared temperature sensor.  
The CV and thermal results are combined using sensor fusion.  

## SEQ 07A - Fire NOT confirmed
If the verification does not confirm a fire:  
1. A2 reports `NO_FIRE` to A1.
2. A1 stops computer vision monitoring temporarily.
3. A1 commands A3 to return to PARKED.
4. Once A3 is physically parked, monitoring can resume.

## SEQ 07B - Fire CONFIRMED
If computer vision and thermal sensing confirm the fire:  
1. A2 reports `FIRE_CONFIRMED` to A1.
2. A1 activates the fire alarm.
3. A1 commands A3 to begin suppression.

## SEQ 08 - Suppress fire
A3 activates the mock water pump.  
The robot remains aimed at the confirmed fire while suppression is active.  
The current prototype performs a **20-second suppression cycle**.  

## SEQ 09 - Stop suppression and recheck
After the suppression period:
1. A1 commands A3 to stop the mock water pump.
2. A3 reports `SUPPRESSION_COMPLETE`.
3. A1 asks A2 to recheck the same target.
4. A2 performs another CV and thermal verification.
If fire is still confirmed, another suppression cycle begins.  
If no fire is detected, the fire is considered extinguished. 

## SEQ 10 - Return robot to PARKED
Once the fire is no longer detected:  
1. A1 stops CV monitoring temporarily.
2. A1 commands A3 to return to PARKED.
3. A3 moves to its tested home position.
4. A1 waits until A3 reports that it is physically `PARKED`.

## SEQ 11 - Resume HOT WORK monitoring
Once A3 is parked, A1 returns the system to HOT_WORK.  
A2 and Python resume computer vision monitoring.  
The system is ready to detect another possible fire.  

# Expected Successful Sequence
A normal successful demonstration should follow:  
```text
STANDBY
   |
   v
HOT_WORK
   |
   v
Possible fire detected
   |
   v
Robot aims at target
   |
   v
CV + thermal verification
   |
   v
FIRE CONFIRMED
   |
   v
Suppression
   |
   v
Recheck
   |
   +---- Fire still detected ----> Suppression again
   |
   +---- Fire out
             |
             v
        Return to PARKED
             |
             v
          HOT_WORK
```

# Stopping the System
The Hot Work button can be used to return the system to STANDBY when normal HOT_WORK monitoring is active.  
To stop the Python computer vision program, select the webcam window and press:  
```text
Q
```

# Python Setup

## Create a virtual environment
From the project folder:  
```text
python -m venv .venv
```
Activate it in Windows PowerShell:  
```text
.\.venv\Scripts\Activate.ps1
```
## Install required libraries  
```text
python -m pip install ultralytics opencv-python pyserial
```
If a NumPy/OpenCV compatibility error occurs:
```text
python -m pip install "numpy<2"
```

# Computer Vision Model
The computer vision system uses a pretrained Fire & Smoke YOLOv8n model.  
- Architecture: YOLOv8 Nano
- Classes: Fire and Smoke
- Training dataset: D-Fire
- Model file: `best.pt`
The `best.pt` file must be located in the same project folder as:
```text
CV-fire_detection.py
```
The Python program uses the webcam image to determine the approximate centre of a detected fire and sends the coordinates to A2.  
`fire_pixels` represents the area of the YOLO bounding box (`width × height`), rather than the exact number of individual flame pixels.  

# Troubleshooting
**Python cannot connect to A2**  
Check that the Perception Arduino is connected by USB and that the correct COM port is configured in `CV-fire_detection.py`.  
If the COM port is unavailable, close any Arduino Serial Monitor connected to the Perception Arduino before running Python.  

**IR temperature sensor not detected**  
Check the MLX90614 power, ground, SDA and SCL connections.  
The system requires the thermal sensor for normal fire verification.  

**Fire is visually detected but not confirmed**  
A YOLO detection alone does not confirm a fire.  
The target must also produce a sufficient thermal reading during verification.  

**A3 does not return to PARKED**  
Check that the pan and tilt servos can physically reach their tested parked positions:  
```text
PAN  = 100 degrees
TILT = 155 degrees
```
A1 waits for A3 to report `PARKED` before HOT_WORK monitoring resumes.

# References

**Fire & Smoke YOLOv8n pretrained model**  
https://huggingface.co/rabahdev/fire-smoke-yolov8n

**D-Fire Dataset**  
https://github.com/gaiasd/DFireDataset

**Ultralytics YOLO**  
https://docs.ultralytics.com/

**OpenCV**  
https://opencv.org/

**Connecting Python to Arduino**  
https://projecthub.arduino.cc/ansh2919/serial-communication-between-python-and-arduino-663756

**XC4490 Laser Diode Module**  
https://media.jaycar.com.au/product/resources/XC4490_manualMain_77417.pdf

**KS0019 keyestudio Passive Buzzer module**  
https://docs.keyestudio.com/projects/KS0019/en/latest/docs/KS0019.html

**Duinotech Pan and Tilt Action Camera Bracket Mount for 9G Servos**  
https://www.jaycar.com.au/pan-tilt-camera

**XC3704 Non-contact IR Sensor Module**  
https://media.jaycar.com.au/product/resources/XC3704_manualMain_81988.pdf
