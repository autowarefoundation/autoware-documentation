# How to calibrate your sensors

## Overview

Autoware expects to have multiple sensors attached to the vehicle as input to perception, localization, and planning stack. These sensors must be calibrated correctly and their positions must be defined using either urdf files (as in [sample_sensor_kit](https://github.com/autowarefoundation/sample_sensor_kit_launch/tree/main/sample_sensor_kit_description)) or as tf launch files.

## Camera calibration

Navigation2 provides a [good tutorial for camera internal calibration](https://navigation.ros.org/tutorials/docs/camera_calibration.html).

## Lidar-lidar calibration

TBD

## Lidar-camera calibration

TBD

## Lidar-IMU calibration

Developed by [APRIL Lab](https://github.com/APRIL-ZJU) at Zhejiang University in China, the LI-Calib calibration tool is a toolkit for calibrating the 6DoF rigid transformation and the time offset between a 3D LiDAR and an IMU, based on continuous-time batch optimization.
IMU-based cost and LiDAR point-to-surfel (surfel = surface element) distance are minimized jointly, which renders the calibration problem well-constrained in general scenarios.

[AutoCore](https://autocore.ai/) has forked the original LI-Calib tool and overwritten the Lidar input for more general usage. Information on how to use the tool, troubleshooting tips and example rosbags can be found at the [LI-Calib fork on Github](https://github.com/autocore-ai/calibration_tools/tree/main/li_calib).
