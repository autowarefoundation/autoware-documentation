<!-- cspell: ignore Zhejiang surfel -->

# Calibrating your sensors

## Overview

Autoware expects to have multiple sensors attached to the vehicle as input to perception, localization, and planning stack.
Autoware uses fusion techniques to combine information from multiple sensors.
For this to work effectively,
all sensors must be calibrated properly to align their coordinate systems, and their positions must be defined using either urdf files
(as in [sample_sensor_kit_description](https://github.com/autowarefoundation/autoware_launch/tree/main/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description))
or as tf launch files.
In this documentation,
we will explain TIER IV's [CalibrationTools](https://github.com/tier4/CalibrationTools) repository for the calibration process.
Please look
at [Starting with TIER IV's CalibrationTools page](./calibration-tools/index.md) for installation and usage of this tool.

If you want to look at other calibration packages and methods, you can check out the following packages.

## Other packages you can check out

### Camera calibration

#### Intrinsic Calibration

- The ROS `camera_calibration` package has a [tutorial for monocular camera calibration](https://docs.ros.org/en/jazzy/p/camera_calibration/doc/tutorial_mono.html).
- [AutoCore](https://autocore.ai/) provides a [light-weight tool](https://github.com/autocore-ai/calibration_tools/tree/main/camera_intrinsic_calib).

#### Camera calibration tools provided by TIER IV

At TIER IV, we provide two types of calibration tools for camera calibration:

<div class="grid cards" markdown>

- [:octicons-arrow-right-24: **Camera Calibration based on ROS**](https://github.com/tier4/CalibrationTools/blob/tier4/universe/docs/tutorials/intrinsic_camera_calibrator.md)

  ***

  This tool detects a calibration board in the images and calculates the intrinsic parameters. It can read the images from a ROS topic, a ROS bag, or image files.

- [:octicons-arrow-right-24: **Camera Calibration via camera-lidar calibration**](https://github.com/tier4/CalibrationTools/blob/tier4/universe/docs/tutorials/tag_based_sfm_calibrator.md#external-camera-intrinsic-calibration)

  ***

  This tool estimates the camera’s intrinsic parameters simultaneously during the camera–lidar calibration process.

</div>

### Lidar-lidar calibration

#### Lidar-lidar Calibration tool provided by Autocore

[LL-Calib on GitHub](https://github.com/autocore-ai/calibration_tools/tree/main/lidar-lidar-calib), provided by [AutoCore](https://autocore.ai/), is a lightweight toolkit for online/offline 3D LiDAR to LiDAR calibration. It's based on local mapping and "GICP" method to derive the relation between main and sub lidar. Information on how to use the tool, troubleshooting tips and example rosbags can be found at the above link.

### Lidar-lidar calibration tool provided by TIER IV

TIER IV provides the following two types of calibration tools for performing LiDAR–LiDAR calibration:

- [**Mapping-based calibrator**](https://github.com/tier4/CalibrationTools/blob/tier4/universe/docs/tutorials/mapping_based_calibrator.md)
  This tool makes a pointcloud map with one LiDAR, and then registers the other LiDARs to this map. It calibrates the full pose.

- [**Lidar-lidar 2D calibrator**](https://github.com/tier4/CalibrationTools/tree/tier4/universe/calibrators/lidar_to_lidar_2d_calibrator)
  This tool uses natural features to calibrate only x, y, and yaw.

### Lidar-camera calibration

Developed by MathWorks, The Lidar Camera Calibrator app enables you to interactively estimate the rigid transformation between a lidar sensor and a camera.

<https://www.mathworks.com/help/lidar/ug/get-started-lidar-camera-calibrator.html>

SensorsCalibration toolbox v0.1: One more open source method for Lidar-camera calibration.
This is a project for LiDAR to camera calibration,including automatic calibration and manual calibration

<https://github.com/PJLab-ADG/SensorsCalibration/blob/master/lidar2camera/README.md>

Developed by [AutoCore](https://autocore.ai/), an easy-to-use lightweight toolkit for Lidar-camera-calibration is proposed. Only in three steps, a fully automatic calibration will be done.

<https://github.com/autocore-ai/calibration_tools/tree/main/lidar-cam-calib-related>

### Lidar-camera calibration with tools provided by TIER IV

TIER IV provides the following calibration tools for performing LiDAR–Camera calibration:

- [**Interactive camera-lidar calibrator**](https://github.com/tier4/CalibrationTools/tree/tier4/universe/calibrators/interactive_camera_lidar_calibrator)
  This tool performs calibration by clicking corresponding points between the image and the point cloud.

- [**Tag-based PnP calibrator**](https://github.com/tier4/CalibrationTools/blob/tier4/universe/docs/tutorials/tag_based_pnp_calibrator.md)
  This tool performs automatic calibration using a calibration board.

- [**Tag-based SfM calibrator**](https://github.com/tier4/CalibrationTools/blob/tier4/universe/docs/tutorials/tag_based_sfm_calibrator.md)
  This tool uses tags on the ground and on boards to calibrate the cameras, the LiDARs, and `base_link` together.

### Lidar-IMU calibration

Developed by [APRIL Lab](https://github.com/APRIL-ZJU) at Zhejiang University in China, the LI-Calib calibration tool is a toolkit for calibrating the 6DoF rigid transformation and the time offset between a 3D LiDAR and an IMU, based on continuous-time batch optimization.
IMU-based cost and LiDAR point-to-surfel (surfel = surface element) distance are minimized jointly, which renders the calibration problem well-constrained in general scenarios.

[AutoCore](https://autocore.ai/) has forked the original LI-Calib tool and overwritten the Lidar input for more general usage. Information on how to use the tool, troubleshooting tips and example rosbags can be found at the [LI-Calib fork on GitHub](https://github.com/autocore-ai/calibration_tools/tree/main/li_calib).

## Base-lidar calibration

### Base-lidar calibration with tools provided by TIER IV

TIER IV provides the following two types of calibration tools for performing Base–LiDAR calibration:

- [**Ground plane calibrator**](https://github.com/tier4/CalibrationTools/tree/tier4/universe/calibrators/ground_plane_calibrator)
  This tool automatically calibrates roll, pitch, and z.

- [**Mapping-based calibrator (base-lidar)**](https://github.com/tier4/CalibrationTools/tree/tier4/universe/calibrators/mapping_based_calibrator)
  This tool uses natural features and the ground to calibrate roll, pitch, and z.

## Radar-lidar calibration

TIER IV provides the [**Marker radar-lidar calibrator**](https://github.com/tier4/CalibrationTools/blob/tier4/universe/docs/tutorials/marker_radar_lidar_calibrator.md).
This tool uses radar reflectors as markers, and it calibrates x, y, and yaw.

## Vehicle calibration tools

The CalibrationTools repository has only sensor calibration tools.
To calibrate the vehicle itself, `autoware_universe` has these packages:

- [autoware_accel_brake_map_calibrator](https://github.com/autowarefoundation/autoware_universe/tree/main/vehicle/autoware_accel_brake_map_calibrator) calibrates the accelerator and brake maps of `autoware_raw_vehicle_cmd_converter`.
- [autoware_steer_offset_estimator](https://github.com/autowarefoundation/autoware_universe/tree/main/vehicle/autoware_steer_offset_estimator) estimates the offset of the steering angle.
