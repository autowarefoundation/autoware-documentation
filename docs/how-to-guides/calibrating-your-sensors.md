# Calibrating your sensors

## Overview

Autoware expects to have multiple sensors attached to the vehicle as input to perception, localization, and planning stack. These sensors must be calibrated correctly and their positions must be defined using either urdf files (as in [sample_sensor_kit](https://github.com/autowarefoundation/sample_sensor_kit_launch/tree/main/sample_sensor_kit_description)) or as tf launch files.

## Camera calibration

Navigation2 provides a [good tutorial for camera internal calibration](https://navigation.ros.org/tutorials/docs/camera_calibration.html).


### Camera calibration tool from TIER IV
TIER IVではCameraのキャリブレーションを行うため以下の二種類のキャリブレーションツールを提供しています
- [Camera Calibration based on ROS](https://github.com/tier4/CalibrationTools/tree/tier4/universe/sensor#intrinsic-camera-calibration)
  - パラメータ値を直接変更しrvizを見ながら調整をするツールです。
- [Camera Calibration via camera-lidar calibraton](https://github.com/tier4/CalibrationTools/tree/tier4/universe/sensor#intrinsic-camera-calibration)
  - camera-lidarキャリブレーションと同時にcameraの内パラ推定ができます

## Lidar-lidar calibration

### Lidar-lidar calibration tool from TIER IV

TIER IVではLiDAR - LiDARのキャリブレーションを行うため以下の二種類のキャリブレーションツールを提供しています
- [Extrinsic Manual Calibration](https://github.com/tier4/CalibrationTools/tree/tier4/universe/sensor#generic-calibration)
  - パラメータ値を直接変更しrvizを見ながら調整をするツールです。
- [Extrinsic Map-Based Calibration](https://github.com/tier4/CalibrationTools/tree/tier4/universe/sensor#lidar-lidar-calibration)
  - 点群地図を使って自動的にキャリブレーションを行うツールです。

## Lidar-camera calibration

### Lidar-camera calibratio from TIER IV

TIER IVではLiDAR - Cameraのキャリブレーションを行うため以下の三種類のキャリブレーションツールを提供しています。
- [Extrinsic Manual Calibration](https://github.com/tier4/CalibrationTools/tree/tier4/universe/sensor#generic-calibration)
  - パラメータ値を直接変更しrvizを見ながら調整をするツールです。
- [Extrinsic Interactive Calibration](https://github.com/tier4/CalibrationTools/tree/tier4/universe/sensor#camera-lidar-calibration)
  - 画像と点群の関係をクリックすることでキャリブレーションを行うツールです。
- [Extrinsic tag-based calibration](https://github.com/tier4/CalibrationTools/tree/tier4/universe/sensor#camera-lidar-calibration)
  - キャリブレーションボードを使って自動的にキャリブレーションを行うツールです。

## Lidar-IMU calibration

Developed by [APRIL Lab](https://github.com/APRIL-ZJU) at Zhejiang University in China, the LI-Calib calibration tool is a toolkit for calibrating the 6DoF rigid transformation and the time offset between a 3D LiDAR and an IMU, based on continuous-time batch optimization.
IMU-based cost and LiDAR point-to-surfel (surfel = surface element) distance are minimized jointly, which renders the calibration problem well-constrained in general scenarios.

[AutoCore](https://autocore.ai/) has forked the original LI-Calib tool and overwritten the Lidar input for more general usage. Information on how to use the tool, troubleshooting tips and example rosbags can be found at the [LI-Calib fork on Github](https://github.com/autocore-ai/calibration_tools/tree/main/li_calib).

## Base-lidar calibration

### Base-lidar calibration from TIER IV

TIER IVではBase - LiDARのキャリブレーションを行うため以下の二種類のキャリブレーションツールを提供しています
- [Extrinsic Manual Calibration](https://github.com/tier4/CalibrationTools/tree/tier4/universe/sensor#generic-calibration)
  - パラメータ値を直接変更しrvizを見ながら調整をするツールです。
- [Extrinsic ground-plane Calibration](https://github.com/tier4/CalibrationTools/tree/tier4/universe/sensor#base-lidar-calibration)
  - roll, pitch, zを自動でキャリブレーションするツールです。


## Other Calibration Tools from TIER IV

TIER IVではsensorの他にLocalization, controlのキャリブレーションツールを開発しOSSとして公開しています。他のキャリブレーションツールは[こちら](https://github.com/tier4/CalibrationTools)を参照してください
