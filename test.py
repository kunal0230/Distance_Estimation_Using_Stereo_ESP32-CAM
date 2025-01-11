import cv2
import numpy as np
import time
from matplotlib import pyplot as plt

# Functions for stereo vision and depth estimation
import triangulation as tri
import calibration

# Mediapipe for face detection
import mediapipe as mp

# ESP32-CAM URLs
url1 = 'http://192.168.0.159:81/stream'  # Camera 1 stream URL
url2 = 'http://192.168.0.178:81/stream'  # Camera 2 stream URL

# Stereo vision setup parameters
B = 6.5  # Distance between the cameras [cm]
f = 3.6  # Camera lens's focal length [mm]
alpha = 67  # Camera field of view in the horizontal plane [degrees]

# Mediapipe face detection setup
mp_facedetector = mp.solutions.face_detection
mp_draw = mp.solutions.drawing_utils

# Open video streams
cap_right = cv2.VideoCapture(url1)
cap_left = cv2.VideoCapture(url2)

# Verify stream availability
if not cap_right.isOpened():
    print("Error: Could not open stream for Camera 1")
if not cap_left.isOpened():
    print("Error: Could not open stream for Camera 2")

# Main loop for stereo vision and depth estimation
with mp_facedetector.FaceDetection(min_detection_confidence=0.7) as face_detection:
    while cap_right.isOpened() and cap_left.isOpened():
        success_right, frame_right = cap_right.read()
        success_left, frame_left = cap_left.read()

        # Check if frames are successfully captured
        if not success_right or not success_left:
            print("Error: Unable to fetch frames from one or both cameras.")
            break

        ################## CALIBRATION ###########################
        frame_right, frame_left = calibration.undistortRectify(frame_right, frame_left)
        ##########################################################

        # Start time for FPS calculation
        start = time.time()

        # Convert frames to RGB for Mediapipe
        frame_right_rgb = cv2.cvtColor(frame_right, cv2.COLOR_BGR2RGB)
        frame_left_rgb = cv2.cvtColor(frame_left, cv2.COLOR_BGR2RGB)

        # Detect faces in both frames
        results_right = face_detection.process(frame_right_rgb)
        results_left = face_detection.process(frame_left_rgb)

        # Convert back to BGR for display
        frame_right = cv2.cvtColor(frame_right_rgb, cv2.COLOR_RGB2BGR)
        frame_left = cv2.cvtColor(frame_left_rgb, cv2.COLOR_RGB2BGR)

        # Initialize center points
        center_point_right = None
        center_point_left = None

        # Process detections for Camera 1
        if results_right.detections:
            for detection in results_right.detections:
                mp_draw.draw_detection(frame_right, detection)
                bbox = detection.location_data.relative_bounding_box
                h, w, _ = frame_right.shape
                center_point_right = (
                    int(bbox.xmin * w + bbox.width * w / 2),
                    int(bbox.ymin * h + bbox.height * h / 2),
                )

        # Process detections for Camera 2
        if results_left.detections:
            for detection in results_left.detections:
                mp_draw.draw_detection(frame_left, detection)
                bbox = detection.location_data.relative_bounding_box
                h, w, _ = frame_left.shape
                center_point_left = (
                    int(bbox.xmin * w + bbox.width * w / 2),
                    int(bbox.ymin * h + bbox.height * h / 2),
                )

        # Depth Calculation
        if center_point_right and center_point_left:
            depth = tri.find_depth(center_point_right, center_point_left, frame_right, frame_left, B, f, alpha)
            cv2.putText(frame_right, f"Distance: {round(depth, 1)} cm", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame_left, f"Distance: {round(depth, 1)} cm", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            print(f"Depth: {round(depth, 1)} cm")
        else:
            # Tracking Lost
            cv2.putText(frame_right, "TRACKING LOST", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame_left, "TRACKING LOST", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # FPS Calculation
        end = time.time()
        fps = 1 / (end - start)
        cv2.putText(frame_right, f"FPS: {int(fps)}", (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame_left, f"FPS: {int(fps)}", (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Display frames
        cv2.imshow("Frame Right", frame_right)
        cv2.imshow("Frame Left", frame_left)

        # Break on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release resources
cap_right.release()
cap_left.release()
cv2.destroyAllWindows()
