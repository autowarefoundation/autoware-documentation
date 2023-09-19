# Intrinsic camera calibration

## Overview

Intrinsic camera calibration is the process
of determining the internal parameters of a camera that affect how it captures images.
These parameters include focal length, optical center, and lens distortion coefficients.
In order to perform camera Intrinsic calibration,
we will use TIER IV's [Intrinsic Camera Calibrator](https://github.com/tier4/CalibrationTools/blob/tier4/universe/sensor/docs/how_to_intrinsic_camera.md) tool.
First of all, we need a calibration board which can be dot, chess or apriltag grid board.
In this tutorial, we will use a 7x7 chess board consisting of 7 cm squares.

Here are some calibration board samples from [Intrinsic Camera Calibrator](https://github.com/tier4/CalibrationTools/blob/tier4/universe/sensor/docs/how_to_intrinsic_camera.md) page:

- Chess boards ([6x8 example](https://github.com/tier4/CalibrationTools/blob/tier4/universe/sensor/docs/resource/checkerboard_8x6.pdf))
- Circle dot boards ([6x8 example](https://github.com/tier4/CalibrationTools/blob/tier4/universe/sensor/docs/resource/circle_8x6.pdf))
- Apriltag grid board ([3x4 example](https://github.com/tier4/CalibrationTools/blob/tier4/universe/sensor/docs/resource/apriltag_grid_3x4.pdf))
