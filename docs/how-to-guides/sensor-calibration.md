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

This calibration tool, developed by [APRIL Lab](https://github.com/APRIL-ZJU) in Zhejiang University in China, is a toolkit for calibrating the 6DoF rigid transformation and the time offset between a 3D LiDAR and an IMU. It's based on continuous-time batch optimization. IMU-based cost and LiDAR point-to-surfel distance are minimized jointly, which renders the calibration problem well-constrained in general scenarios.

[Autocore](https://autocore.ai/) has overwritten the Lidar input for a more general usage. For more details including instructions, trouble shooting or rosbag etc., please refer to the link [here](https://gitlab.com/JianKang_Egon/li_calib)
