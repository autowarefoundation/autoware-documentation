# Starting with TIER IV's CalibrationTools

## Overview

Autoware expects to have multiple sensors attached to the vehicle as input to perception, localization, and planning stack. These sensors must be calibrated correctly, and their positions must be defined using either urdf files (as in [sample_sensor_kit](https://github.com/autowarefoundation/sample_sensor_kit_launch/tree/main/sample_sensor_kit_description)) or as tf launch files.
In order to do that, we will use TIER IV' [CalibrationTools](https://github.com/tier4/CalibrationTools) repository.

## Installing TIER IV's CalibrationTools repositories on autoware

After completing previous steps (creating your own autoware,
creating a vehicle and sensor model etc.)
we are ready to calibrate sensors which prepared their pipeline in creating the sensor model section.

Firstly, we will clone CalibrationTools repositories in own autoware.

```bash
cd <YOUR-OWN-AUTOWARE-DIRECTORY> # for example: cd autoware.tutorial_vehicle
wget https://raw.githubusercontent.com/tier4/CalibrationTools/tier4/universe/calibration_tools.repos
vcs import src < calibration_tools.repos
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

Then build the all packages
after the all necessary changes are made on sensor model and vehicle model.

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Usage of CalibrationTools

The CalibrationTools repository has several packages
for calibrating different sensor pairs such as lidar-lidar,
camera-lidar, ground-lidar etc. In order to calibrate our sensors,
we will modify `extrinsic_calibration_package` for our sensor kit.

- [Manual Calibration](./manual-calibration.md)
- [Lidar-Lidar Calibration](./lidar-lidar-calibration.md)
  - [Ground Plane-Lidar Calibration](./ground-lidar-calibration.md)
- [Intrinsic Camera Calibration](./intrinsic-camera-calibration.md)
- [Lidar-Camera Calibration](./lidar-camera-calibration.md)
