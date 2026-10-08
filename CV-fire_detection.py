# =================================================================
# INDUSTRIAL AUTOMATED FIREWATCH (IAF)
# ICTE4005 Robotics Project - Group 5
# Curtin University, 2026
#
# Team:
# Saf Flatters, Annie (Annabelle) Lewkowski, Devlin MacGlip
#
# MODULE:
# Python Computer Vision (CV) - Perception
#
# PURPOSE:
# The Python CV module provides RGB fire detection for A2 (Perception).
# - Captures live video from the webcam
# - Runs the trained YOLO fire detection model
# - Detects possible fires and calculates CV confidence
# - Calculates target X/Y coordinates and fire bounding-box area
# - Sends fire detection data to A2 via USB Serial
# - Starts and stops CV monitoring in response to commands from A2
#
# SETUP:
# 1. Connect the webcam and Arduino 2 (A2) to the laptop.
# 2. Open a terminal in the project folder.
# 3. Activate the Python virtual environment.
# 4. Ensure OpenCV, Ultralytics and pySerial are installed.
# 5. Ensure the trained YOLO model "best.pt" is in the project folder.
# 6. Check the Arduino COM port and update port="COM4" below if required.
# 7. Upload/start the A2 Perception Arduino code.
# 8. Run this script: python CV-fire_detection.py
# 9. Press Q in the webcam window to stop the program.
#
# COMMUNICATION:
#
# USB Serial:
# Laptop/Python <-> Arduino 2 (A2)
# Serial baud rate -> 115200
# Current port -> COM4
#
# Computer Vision:
# Webcam -> OpenCV -> YOLO -> Python -> A2
# YOLO model -> best.pt
# CV detection threshold -> 0.50
#
#
# SYSTEM SEQUENCE:
#
# SEQ 01 - System startup / STANDBY
# SEQ 02 - Operator starts HOT WORK
# SEQ 03 - Monitor for fire
# SEQ 04 - Possible fire detected
# SEQ 05 - Aim robot at target
# SEQ 06 - Verify fire with CV + thermal
# SEQ 07A - Fire NOT confirmed
# SEQ 07B - Fire CONFIRMED
# SEQ 08 - Suppress fire
# SEQ 09 - Stop suppression and recheck
# SEQ 10 - Return robot to PARKED
# SEQ 11 - Resume HOT WORK monitoring
#
# =================================================================

import cv2
from ultralytics import YOLO
import serial  # to be able to send to Arduino through USB
import time
# import threading # allows typing test commands while CV runs ()



# ARDUINO 2 SERIAL CONNECTION
# SEQ 01 - SYSTEM STARTUP / STANDBY
# Connect Python to the Perception Arduino through USB Serial.
# The baud rate must match Serial.begin(115200) on A2.
try:
    arduino = serial.Serial(
        port="COM4",  ## This must match Arduino USB Serial
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

# COMMENT OUT WHEN A1 IS UP
# TEST MODE (when not connected to A1) - manually send commands that would normally come from Arduino 1
# def manual_commands():

#     while True:
#         command = input("> ").strip().upper()
#         if command in ["HOT_WORK", "VERIFY_TARGET", "STANDBY"]:
#             if arduino is not None:
#                 arduino.write((command + "\n").encode())
#                 print(f"TEST COMMAND SENT: {command}")
#         else:
#             print("Unknown command. Use HOT_WORK, VERIFY_TARGET or STANDBY")



# COMPUTER VISION SETUP
# SEQ 01 - SYSTEM STARTUP / STANDBY

# Load trained YOLO fire detection model
model = YOLO("best.pt")

# Open webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Could not open webcam")
    exit()

# CV starts inactive and waits for HOT_WORK command from Arduino 2
cv_active = False

# Start separate thread so Bash can accept test commands while CV keeps running
# if arduino is not None:
#     command_thread = threading.Thread(
#         target=manual_commands,
#         daemon=True
#     )
#     command_thread.start()



# MAIN COMPUTER VISION LOOP
# Continuously checks A2 commands, reads the webcam and runs fire detection.
while True:

    # Check for commands from Arduino 2
    if arduino is not None and arduino.in_waiting > 0:
        response = arduino.readline().decode().strip()
        print(f"Arduino: {response}")

# SEQ 02 + 03 - START HOT WORK / MONITOR FOR FIRE
        # A2 has entered MONITORING and tells Python to start CV.
        if response == "START_CV":
            cv_active = True
            print("\n==============================")
            print("STATE: MONITORING")
            print("CV ACTIVE - looking for fire")
            print("==============================\n")

# SEQ 04 - POSSIBLE FIRE DETECTED
        # A2 has accepted the CV detection and sent the target to A1.
        elif response == "POSSIBLE_FIRE_DETECTED":
            print("\n========================================")
            print("CV: POSSIBLE FIRE DETECTED")
            print(f"  Confidence: {confidence:.0%}")
            print(f"  Centre:     ({centre_x}, {centre_y})")
            print(f"  Area:       {fire_pixels} px")
            print()
            print("A2 -> A1: POSSIBLE FIRE")
            print("  Target coordinates sent to Mission Control")
            print("  Waiting for robot to aim")
            print("========================================\n")

# SEQ 06 - VERIFY FIRE WITH CV + THERMAL
        # A3 has aimed at the target and A2 requests fresh CV data.
        elif response == "VERIFY_CV":
            print("\n==============================")
            print("STATE: VERIFYING")
            print("Getting fresh CV + thermal reading")
            print("==============================\n")

# SEQ 07A - FIRE NOT CONFIRMED
        elif response == "FIRE_NOT_CONFIRMED":
            print("\n==============================")
            print("STATE: FIRE NOT CONFIRMED")
            print("Returning to monitoring")
            print("==============================\n")

# SEQ 01 / 10 - STOP CV
        # CV is stopped while the system is in STANDBY or A3 returns to PARKED.
        elif response == "STOP_CV":
            cv_active = False
            print("CV monitoring stopped")


    # READ WEBCAM FRAME
    ret, frame = cap.read()
    if not ret:
        print("Could not read frame")
        break
    
    # height, width = frame.shape[:2]
    # print(f"CAMERA FRAME: {width} x {height}")

# SEQ 03 - MONITOR FOR FIRE
    # Only run YOLO fire detection while HOT WORK mode is active.
    if cv_active:
        results = model(
            frame,
            conf=0.50,
            verbose=False
        )
        fire_detected = False


        # READ YOLO DETECTIONS
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = model.names[class_id]
                confidence = float(box.conf[0])

# SEQ 04 - POSSIBLE FIRE DETECTION
                # Only process detections classified as fire.
                if class_name.lower() == "fire":
                    fire_detected = True

                    # Get fire bounding box
                    x1, y1, x2, y2 = map(
                        int,
                        box.xyxy[0]
                    )

                    # Calculate centre coordinates
                    centre_x = int((x1 + x2) / 2)
                    centre_y = int((y1 + y2) / 2)

                    # Flip coordinates because the physical pan/tilt mount
                    # moves opposite to the camera image coordinates
                    centre_x = 640 - centre_x
                    centre_y = 480 - centre_y

                    # Approximate fire pixel area
                    width = x2 - x1
                    height = y2 - y1

                    fire_pixels = width * height

# SEQ 04 - SEND POSSIBLE FIRE DATA TO A2
                    # Send CV confidence, target coordinates and area.
                    # print(
                    #     f"FIRE | "
                    #     f"Confidence: {confidence:.2f} | "
                    #     f"Pixels: {fire_pixels} | "
                    #     f"Centre: ({centre_x}, {centre_y})"
                    # )
                    message = (
                        f"FIRE,"
                        f"{confidence:.2f},"
                        f"{centre_x},"
                        f"{centre_y},"
                        f"{fire_pixels}\n"  # newline is end of message
                    )
                    if arduino is not None:    
                        arduino.write(message.encode())

                    # Draw fire detection box
                    cv2.rectangle(
                        frame,
                        (x1, y1),
                        (x2, y2),
                        (0, 0, 255),
                        2
                    )

                    # Draw fire centre point
                    cv2.circle(
                        frame,
                        (centre_x, centre_y),
                        6,
                        (0, 0, 255),
                        -1
                    )

                    # Draw detection label
                    cv2.putText(
                        frame,
                        f"FIRE {confidence:.0%}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2
                    )



# SEQ 03 - NO FIRE DETECTED
        # Tell A2 when YOLO does not detect fire in the current frame.
        if not fire_detected:
            # Laptop printed output
            # print("No fire")

            if arduino is not None:    
                # Send to Arduino output
                arduino.write(b"NO_FIRE\n")

            
    # DISPLAY WEBCAM
    cv2.imshow(
        "FireWatch CV",
        frame
    )

    # Press Q to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# SHUTDOWN
# Release the webcam and close the Serial connection.
cap.release()
cv2.destroyAllWindows()
if arduino is not None:    
    arduino.close()  # closes serial connection