# FIREWATCH Computer Vision

This program uses a webcam and a **pretrained YOLOv8 computer vision model** to detect fire in real time. This then sends results to PERCEPTION arduino.  

Rather than detecting fire based only on colour, the program uses a neural network that has already been trained on images containing fire and smoke.

Arduino 2 → USB Serial → Python → YOLO → Camera 

## Output

When fire is detected, the program outputs:

- Detection confidence
- Approximate size of the detected fire region
- Centre `(x, y)` coordinates of the detected fire
- Live webcam feed with a bounding box around the fire
- Sends information over serial to Perception Arduino

Example:

   Printed to screen: `FIRE | Confidence: 0.91 | Pixels: 24000 | Centre: (360, 250)`
   Send over serial to Perception Arduino: `FIRE,0.91,360,250,24000`

The centre coordinates can later be sent to the FIREWATCH control system to determine where the detected fire is located in the camera image.

**Note:** `Pixels` currently represents the area of the YOLO bounding box (`width × height`), not the exact number of individual flame pixels.

---

## Model

This program uses a pretrained **Fire & Smoke YOLOv8n** model.

- Architecture: YOLOv8 Nano
- Classes: Fire and Smoke
- Training dataset: D-Fire
- Model source: https://huggingface.co/rabahdev/fire-smoke-yolov8n

Download the trained `best.pt` model from:

https://huggingface.co/rabahdev/fire-smoke-yolov8n/blob/main/best.pt

Place `best.pt` in the same folder as the Python script:

    FIREWATCH_CV/
    ├── CV-fire_detection.py
    ├── best.pt
    └── CV-README.md

---

## Setup

### 1. Create a virtual environment

From the FIREWATCH_CV folder:
    python -m venv .venv

Activate it in Windows PowerShell:
    .\.venv\Scripts\Activate.ps1

You should now see `(.venv)` at the beginning of the terminal.

### 2. Install required libraries

    python -m pip install ultralytics opencv-python pyserial

If there is a NumPy/OpenCV compatibility error, run:
    python -m pip install "numpy<2"

### 3. Run FIREWATCH
1. Connect to Perception Arduino through USB serial. CHANGE "COM3" in Python code to whatever COM port Arduino uses. The baud rate MUST match Serial.begin(115200) on Arduino. 

2. Connect the webcam and run:
    python CV-fire_detection.py

Press **Q** while the video window is selected to stop the program.

---
To run in test mode without other Arduinos connected (just Arduino 2 with perception_arduino_sketch2 installed on it, IR Temp sensor connected, web cam connected and python CV code):
1. make sure in sketch `const bool TEST_MODE = true;`
2. make sure CV feedback is comments out
3. type in bash: `python CV-fire_detection.py`
4. wait for 
```Connected to Perception Arduino
> Arduino: PERCEPTION_READY
Arduino: IR temperature sensor connected
```
(if takes too long Com4 may be blocked)
5. Type: HOT_WORK to go into monitoring mode
6. Point camera at fire -> POSSIBLE FIRE DETECTED
7. Robot to be moved to those coordinates (A1, A3)
8. Type: VERIFY_TARGET and point IR Temp sensor at heat and camera at fire
9. Either NO FIRE DETECTED or FIRE DETECTED

---

## Understanding the coordinates

OpenCV image coordinates start in the top-left corner:

    (0,0) --------------------> X
      |
      |
      |
      v
      Y

For example:

    Centre: (360, 250)

means the centre of the detected fire is approximately **360 pixels from the left** and **250 pixels from the top** of the camera image.

---

## References

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