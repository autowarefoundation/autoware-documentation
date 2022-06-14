# How to calibrate your sensors

## Overview
Autoware expects to have multiple sensors attached to the vehicle as input to perception, localization, and planning stack. These sensors must be calibrated correctly and the position must be defined as urdf (or as tf launch files) as in [sample_sensor_kit](https://github.com/autowarefoundation/sample_sensor_kit_launch/tree/main/sample_sensor_kit_description).


## Camera Calibration
Navigation2 provides a good tutorial for camera internal calibraiton:
https://navigation.ros.org/tutorials/docs/camera_calibration.html


## Lidar-Lidar Calibration
TBD

## Lidar-Camera Calibration
TBD
