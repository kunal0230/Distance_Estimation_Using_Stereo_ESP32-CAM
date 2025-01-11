# Distance Estimation Using Stereo ESP32-CAM

This project implements a stereo vision setup using two ESP32-CAM modules to estimate the depth and distance of objects. By capturing images from two cameras, it calculates the disparity map and performs depth estimation.

---

## Features
- **Stereo Vision Setup**: Two ESP32-CAM modules are used as stereo cameras.
- **Depth Estimation**: Calculates the depth of objects using disparity.
- **Real-Time Camera Feeds**: Displays both camera feeds for alignment and monitoring.
- **Image Capture**: Save stereo image pairs for calibration or processing.

---


---

## Setup Instructions

### Hardware Requirements
- 2 x ESP32-CAM modules
- Mount for aligning the cameras
- Stable Wi-Fi network

Calibration Instructions
Use a printed or digital chessboard for calibration.
Capture multiple stereo image pairs from different angles and distances.
Perform camera calibration using OpenCV to compute intrinsic and extrinsic parameters.

# Parameters for Stereo Vision

Frame Rate ~30 fps Maximum frame rate supported by ESP32-CAM.

Baseline (B) ~8 cm Distance between the two camera lenses.

Focal Length (f) ~3.6 mm Approximate focal length of the OV2640 lens.

Field of View (α) ~67° Horizontal field of view for the default lens..

# How It Works
Stereo Image Capture:

Captures synchronized images from two ESP32-CAMs.

Calibration:

Intrinsic calibration to compute lens distortion and focal length.

Stereo calibration to calculate the relative position of cameras.

Depth Estimation:

Uses disparity (difference in pixel positions) to calculate depth.

