# Starting with TIER IV's CalibrationTools

## Overview

Autoware expects to have multiple sensors attached to the vehicle as input to perception,
localization,
and planning stack.
These sensors must be calibrated correctly,
and their positions must be defined in the `config/` folder of the `<YOUR-VEHICLE-NAME>_sensor_kit_description` package.
In this tutorial,
we will use TIER IV's [CalibrationTools](https://github.com/tier4/CalibrationTools) repository for the calibration.

## Setting of sensor_kit_base_link position with respect to the base_link

In previous section (creating the vehicle and sensor model),
we mentioned about `sensors_calibration.yaml`.
This file stores `sensor_kit_base_link` (child frame) position and orientation with respect to the
`base_link` (parent frame).
We need to update this relative position
(all values were initially set equal to zero when file is created)
with using CAD data of our vehicle.

{{ diagram("images/tutorial-vehicle-sensor-kit-base-link.svg", "The base_link to sensor_kit_base_link transformation of tutorial_vehicle, from its CAD dimensions.") }}

So, our `sensors_calibration.yaml` file for our tutorial_vehicle should be like this:

```yaml
base_link:
  sensor_kit_base_link:
    x: 1.600000 # meter
    y: 0.0
    z: 1.421595 # 1.151595m + 0.270m
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

You need to update this transformation value with respect to the `sensor_kit_base_link` frame.
You can also use CAD values for GNSS/INS and IMU position in `sensor_kit_calibration.yaml` file.
Both files are in the `config/` folder of your sensor kit description package.

## Installing TIER IV's CalibrationTools repositories on autoware

After completing previous steps (creating your own autoware,
creating a vehicle and sensor model etc.)
we are ready to calibrate sensors which prepared their pipeline in creating the sensor model section.
CalibrationTools supports Ubuntu 22.04 with ROS 2 Humble and Ubuntu 24.04 with ROS 2 Jazzy.

Firstly, we will clone CalibrationTools repositories in own autoware.

```bash
cd <YOUR-OWN-AUTOWARE-DIRECTORY> # for example: cd autoware.tutorial_vehicle
wget https://raw.githubusercontent.com/tier4/CalibrationTools/tier4/universe/calibration_tools_autoware.repos
vcs import src < calibration_tools_autoware.repos
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

Then build the all packages
after the all necessary changes are made on sensor model and vehicle model.

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Usage of CalibrationTools

The CalibrationTools repository has several calibrator packages
for calibrating different sensor pairs such as lidar-lidar,
camera-lidar, ground-lidar etc.
The `sensor_calibration_manager` package gives one user interface for all extrinsic calibrators.
It starts the calibrator with a launch file, makes sure that the required `tf` frames are available, and saves the result.

Start it with this command, after you source your Autoware workspace:

```bash
ros2 run sensor_calibration_manager sensor_calibration_manager
```

Then do these steps:

1. Select a project and a calibrator, and click `Continue`. A project is a set of launch files for one vehicle type. The `default_project` shows most parameters of each calibrator, so you can use it with your own vehicle without new files.
2. Set the arguments of the launch file in the `Launcher configuration` window, for example the frame names and the topic names of your sensors. Then click `Launch`.
3. Play your rosbag, or start your sensors. When the calibrator service and the required `tf` frames are available, the `Calibrate` button becomes active. If you remap the `tf` topics of the rosbag, publish the `tf` of your vehicle with `tier4_vehicle_launch` (see the [manual calibration](../extrinsic-manual-calibration/index.md#checking-the-calibration-in-rviz) page).
4. Click `Calibrate`. Some calibrators need more input, for example points that you select in RViz.
5. Check the result in the `Calibration tree` and `Final TF tree` widgets, and click `Save calibration`.
6. Copy the calibrated values to `sensor_kit_calibration.yaml` or `sensors_calibration.yaml` in your sensor kit description package, and build the package again. If the saved frames are different from the frames in these files, convert the values first. The page of each calibrator tells you which frames it saves.

The manager remembers your last configuration in `~/.config/tier4/sensor_calibration_manager.conf`.
If you calibrate your vehicles often, you can create your own project with your frame names and topic names.
See the [Integration section of the CalibrationTools README](https://github.com/tier4/CalibrationTools/tree/tier4/universe#integration) for the steps.

!!! note

    The calibration pages of tutorial_vehicle were first written for the `extrinsic_calibration_manager` package of an earlier CalibrationTools version.
    The current CalibrationTools does not have this package, so the pages now use `sensor_calibration_manager`.
    The launch files of tutorial_vehicle for the earlier version are in the [tutorial_vehicle_calibration_tools](https://github.com/leo-drive/tutorial_vehicle_calibration_tools/tree/tutorial_vehicle/sensor/extrinsic_calibration_manager/launch/tutorial_vehicle_sensor_kit) repository.

The following pages explain each calibration:

- [Manual Calibration](../extrinsic-manual-calibration/index.md)
- [Lidar-Lidar Calibration](../lidar-lidar-calibration/index.md)
  - [Ground Plane-Lidar Calibration](../ground-lidar-calibration/index.md)
- [Intrinsic Camera Calibration](../intrinsic-camera-calibration/index.md)
- [Lidar-Camera Calibration](../lidar-camera-calibration/index.md)
- [Lidar-Imu Calibration](../lidar-imu-calibration/index.md)
