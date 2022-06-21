# How to calibrate your sensors

## Overview

Autoware expects to have multiple sensors attached to the vehicle as input to perception, localization, and planning stack. These sensors must be calibrated correctly and their positions must be defined using either urdf files (as in [sample_sensor_kit](https://github.com/autowarefoundation/sample_sensor_kit_launch/tree/main/sample_sensor_kit_description)) or as tf launch files.

## Camera calibration

Navigation2 provides a [good tutorial for camera internal calibration](https://navigation.ros.org/tutorials/docs/camera_calibration.html).

## Lidar-lidar calibration

TBD

## Lidar-camera calibration

TBD
