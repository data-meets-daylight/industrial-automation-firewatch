# FIREWATCH Computer Vision

This program uses a webcam and a **pretrained YOLOv8 computer vision model** to detect fire in real time.

Rather than detecting fire based only on colour, the program uses a neural network that has already been trained on images containing fire and smoke.

## Output

When fire is detected, the program outputs:

- Detection confidence
- Approximate size of the detected fire region
- Centre `(x, y)` coordinates of the detected fire
- Live webcam feed with a bounding box around the fire

Example:

    `FIRE | Confidence: 0.91 | Pixels: 24000 | Centre: (360, 250)`

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

    python -m pip install ultralytics opencv-python

If there is a NumPy/OpenCV compatibility error, run:
    python -m pip install "numpy<2"

### 3. Run FIREWATCH

Connect the webcam and run:
    python CV-fire_detection.py

Press **Q** while the video window is selected to stop the program.

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