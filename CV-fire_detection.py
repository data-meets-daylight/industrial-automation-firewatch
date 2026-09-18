# Computer Vision Script for Automatic FIREWATCH 
# ICTE4005 Robotics Assignment - Group 5
# Saf, Annie, Devlin - Curtin University 2026
# Read CV-README.md for set up advice

# Camera → YOLO → Python → USB Serial → Arduino 2

import cv2
from ultralytics import YOLO
import serial  # to be able to send to Arduino through USB
import time

#### ARDUNIO
# Connect to Perception Arduino through USB serial.
# CHANGE "COM3" to whatever COM port Arduino uses.
# The baud rate MUST match Serial.begin(115200) on Arduino.
try:
    arduino = serial.Serial(
        port="COM4",
        baudrate=115200,
        timeout=1
    )

    # Arduino normally resets when the serial connection opens, so we giving it time
    time.sleep(2)
    print("Connected to Perception Arduino")

except serial.SerialException:
    # if Arduino can ot be found, continue to run
    arduino = None
    print("Arduino NOT connected - running CV only")



#### COMPUTER VISION

# 1. Load trained fire detection model
model = YOLO("best.pt")


# 2. Open webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Could not open webcam")
    exit()


# 3. Read webcam continuously
while True:
    ret, frame = cap.read()
    if not ret:
        print("Could not read frame")
        break


    # 4. Run trained model
    results = model(
        frame,
        conf=0.50,
        verbose=False
    )
    fire_detected = False


    # 5. Read detections
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            class_name = model.names[class_id]
            confidence = float(box.conf[0])

            # Only process FIRE detections
            if class_name.lower() == "fire":
                fire_detected = True

                # Get bounding box
                x1, y1, x2, y2 = map(
                    int,
                    box.xyxy[0]
                )

                # Calculate centre coordinates
                centre_x = int((x1 + x2) / 2)
                centre_y = int((y1 + y2) / 2)

                # Approximate fire pixel area
                width = x2 - x1
                height = y2 - y1

                fire_pixels = width * height

                # Outputs
                # Laptop printed output
                print(
                    f"FIRE | "
                    f"Confidence: {confidence:.2f} | "
                    f"Pixels: {fire_pixels} | "
                    f"Centre: ({centre_x}, {centre_y})"
                )
                # Send to Arduino output
                message = (
                    f"FIRE,"
                    f"{confidence:.2f},"
                    f"{centre_x},"
                    f"{centre_y},"
                    f"{fire_pixels}\n"  # newline is end of message
                )
                if arduino is not None:    
                    arduino.write(message.encode())

                # Draw detection box
                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 0, 255),
                    2
                )

                # Draw centre point
                cv2.circle(
                    frame,
                    (centre_x, centre_y),
                    6,
                    (0, 0, 255),
                    -1
                )

                # Detection label
                cv2.putText(
                    frame,
                    f"FIRE {confidence:.0%}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

    # 6. No fire
    if not fire_detected:
        # Laptop printed output
        print("No fire")

        if arduino is not None:    
            # Send to Arduino output
            arduino.write(b"NO_FIRE\n")

    # 7. CHECK FOR RESPONSE FROM ARDUINO - If Arduino has sent something back, read it and print it.
    if arduino is not None and arduino.in_waiting > 0:
        response = arduino.readline().decode().strip()
        print(f"Arduino: {response}")

    # 8. Show webcam
    cv2.imshow(
        "FireWatch CV",
        frame
    )

    # Press Q to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


cap.release()
cv2.destroyAllWindows()
if arduino is not None:    
    arduino.close()  # closes serial connection