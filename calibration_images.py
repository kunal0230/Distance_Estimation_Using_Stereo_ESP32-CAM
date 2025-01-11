# import cv2

# cap = cv2.VideoCapture(0)
# cap2 = cv2.VideoCapture(2)

# num = 0

# while cap.isOpened():

#     succes1, img = cap.read()
#     succes2, img2 = cap2.read()

#     k = cv2.waitKey(5)

#     if k == 27:
#         break
#     elif k == ord('s'): # wait for 's' key to save and exit
#         cv2.imwrite('images/stereoLeft/imageL' + str(num) + '.png', img)
#         cv2.imwrite('images/stereoright/imageR' + str(num) + '.png', img2)
#         print("images saved!")
#         num += 1

#     cv2.imshow('Img 1',img)
#     cv2.imshow('Img 2',img2)

# # Release and destroy all windows before termination
# cap.release()
# cap2.release()

# cv2.destroyAllWindows()

import cv2
import numpy as np

# Update the URLs of the ESP32-CAM streams
url1 = 'http://192.168.0.159:81/stream'  # Camera 1 stream URL
url2 = 'http://192.168.0.178:81/stream'  # Camera 2 stream URL

# Initialize video capture for both cameras
cap1 = cv2.VideoCapture(url1)
cap2 = cv2.VideoCapture(url2)

# Check if streams are accessible
if not cap1.isOpened():
    print("Error: Could not open stream for Camera 1")
if not cap2.isOpened():
    print("Error: Could not open stream for Camera 2")

num = 0  # Image counter

while True:
    # Read frames from both cameras
    ret1, img1 = cap1.read()
    ret2, img2 = cap2.read()

    # Check if frames are successfully captured
    if ret1 and ret2:
        # Combine frames side by side for easier visualization
        combined_frame = np.hstack((img1, img2))
        cv2.imshow('ESP32-CAM Feeds (Left | Right)', combined_frame)
    elif ret1:
        # Show only Camera 1 if Camera 2 fails
        cv2.imshow('ESP32-CAM 1', img1)
    elif ret2:
        # Show only Camera 2 if Camera 1 fails
        cv2.imshow('ESP32-CAM 2', img2)

    # Keyboard controls
    k = cv2.waitKey(5)
    if k == 27:  # Press 'Esc' to exit
        break
    elif k == ord('s'):  # Press 's' to save images
        if ret1:
            cv2.imwrite(f'images/stereoLeft/imageL{num}.png', img1)
        if ret2:
            cv2.imwrite(f'images/stereoRight/imageR{num}.png', img2)
        print(f"Images saved! Image number: {num}")
        num += 1

# Release resources and close all OpenCV windows
cap1.release()
cap2.release()
cv2.destroyAllWindows()
