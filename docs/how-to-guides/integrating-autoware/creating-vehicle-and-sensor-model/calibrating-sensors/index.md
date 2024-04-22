# Calibrating your sensors

## Overview

Autoware expects to have multiple sensors attached to the vehicle as input to perception, localization, and planning stack.
Autoware uses fusion techniques to combine information from multiple sensors.
For this to work effectively,
all sensors must be calibrated properly to align their coordinate systems, and their positions must be defined using either urdf files
(as in [sample_sensor_kit](https://github.com/autowarefoundation/sample_sensor_kit_launch/tree/main/sample_sensor_kit_description))
or as tf launch files.
In this documentation,
we will explain TIER IV's [CalibrationTools](https://github.com/tier4/CalibrationTools) repository for the calibration process.
Please look
at [Starting with TIER IV's CalibrationTools page](./calibration-tools/index.md) for installation and usage of this tool.

If you want to look at other calibration packages and methods, you can check out the following packages.

## Other packages you can check out

### Camera calibration

#### Intrinsic Calibration

- Navigation2 provides a [good tutorial for camera internal calibration](https://navigation.ros.org/tutorials/docs/camera_calibration.html).
- [AutoCore](https://autocore.ai/) provides a [light-weight tool](https://github.com/autocore-ai/calibration_tools/tree/main/camera_intrinsic_calib).

### Lidar-lidar calibration

#### Lidar-Lidar Calibration tool from Autocore

[LL-Calib on GitHub](https://github.com/autocore-ai/calibration_tools/tree/main/lidar-lidar-calib), provided by [AutoCore](https://autocore.ai/), is a lightweight toolkit for online/offline 3D LiDAR to LiDAR calibration. It's based on local mapping and "GICP" method to derive the relation between main and sub lidar. Information on how to use the tool, troubleshooting tips and example rosbags can be found at the above link.

### Lidar-camera calibration

Developed by MathWorks, The Lidar Camera Calibrator app enables you to interactively estimate the rigid transformation between a lidar sensor and a camera.

<https://ww2.mathworks.cn/help/lidar/ug/get-started-lidar-camera-calibrator.html>

SensorsCalibration toolbox v0.1: One more open source method for Lidar-camera calibration.
This is a project for LiDAR to camera calibration,including automatic calibration and manual calibration

<https://github.com/PJLab-ADG/SensorsCalibration/blob/master/lidar2camera/README.md>

Developed by [AutoCore](https://autocore.ai/), an easy-to-use lightweight toolkit for Lidar-camera-calibration is proposed. Only in three steps, a fully automatic calibration will be done.

<https://github.com/autocore-ai/calibration_tools/tree/main/lidar-cam-calib-related>

### Lidar-IMU calibration

Developed by [APRIL Lab](https://github.com/APRIL-ZJU) at Zhejiang University in China, the LI-Calib calibration tool is a toolkit for calibrating the 6DoF rigid transformation and the time offset between a 3D LiDAR and an IMU, based on continuous-time batch optimization.
IMU-based cost and LiDAR point-to-surfel (surfel = surface element) distance are minimized jointly, which renders the calibration problem well-constrained in general scenarios.

[AutoCore](https://autocore.ai/) has forked the original LI-Calib tool and overwritten the Lidar input for more general usage. Information on how to use the tool, troubleshooting tips and example rosbags can be found at the [LI-Calib fork on GitHub](https://github.com/autocore-ai/calibration_tools/tree/main/li_calib).
