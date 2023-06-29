# Localization mode

The Autoware provides multiple localization methods that work with multiple different sensor configurations.
The table below shows the supported sensor configurations and their corresponding algorithms.

| localization mode | algorithm | map type        |
| ----------------- | --------- | --------------- |
| LiDAR-based       | NDT       | point cloud map |
| Camera-based      | YabLoc    | vector map      |
| GNSS/IMU-based    | Eagleye   | -               |

The table below indicates which pose_estimator and twist_estimator are called based on the invoked launch file and provided arguments.

|                              | localization_mode<br>=lidar       | localization_mode<br>=camera | pose_estimator_mode<br>=lidar | pose_estimator_mode<br>=camera |
| ---------------------------- | --------------------------------- | ---------------------------- | ----------------------------- | ------------------------------ |
| tier4_localization_component | ndt_scan_matcher<br>gyro_odometer | yabloc<br>gyro_odometer      | -                             | -                              |
| map4_localization_component  | -                                 | -                            | ndt_scan_matcher<br>eagleye   | eagleye<br>eagleye             |

The top row of cells indicate pose_estimator and the bottom row indicates twist_estimator.

## LiDAR-based localizer (default)

By default, Autoware launches [ndt_scan_matcher](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/ndt_scan_matcher) for localization.

## Camera-based localizer

You can use YabLoc as a camera-based localization method.
For more details on YabLoc, please refer to the [README of YabLoc](https://github.com/autowarefoundation/autoware.universe/blob/main/localization/yabloc/README.md) in autoware.universe.

To use YabLoc as a pose_estimator, add `localization_mode:=camera` when launching Autoware.
By default, the `localization_mode` is set to `lidar`.
By specifying this command-line argument, YabLoc nodes will be automatically launched while the NDT nodes will not be started.

Here is an example of a launch command:

```bash
ros2 launch autoware_launch autoware.launch.xml \
  vehicle_model:=YOUR_VEHICLE \
  sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP \
  localization_mode:=camera # Add this argument
```

## GNSS/IMU-based localizer

You can use Eagleye as a GNSS/IMU-based localization method. For more details on Eagleye, please refer to the [Eagleye](eagleye-guide.md).

Eagleye has a function for position estimation and twist estimation, namely `pose_estimator` and `twist_estimator`, respectively.
When running Eagleye in twist_estimator mode, ndt_scan_matcher is used as the pose_estimator.
Eagleye will improve scan matching by providing accurate twists using GNSS doppler.

To use Eagleye, it requires both specifying the command-line arguments and modifying the launch file.

Firstly, please modify the commented-out line and change it to launch `map4_localization_component.launch.xml` instead of `tier4_localization_component.launch.xml` in [`autoware.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/autoware.launch.xml).
Please refer to the following snippet for the modification details:

```xml
  <!-- Localization -->
  <group if="$(var launch_localization)">
    <!-- <include file="$(find-pkg-share autoware_launch)/launch/components/tier4_localization_component.launch.xml"/> -->
    <include file="$(find-pkg-share autoware_launch)/launch/components/map4_localization_component.launch.xml"/>
  </group>
```

NOTE: Please refer to [`map4_localization_launch`](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/map4_localization_launch) in the `autoware.universe` package and [`map4_localization_component.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/map4_localization_component.launch.xml) in `autoware_launch` package for information on how to modify the localization launch.

Once you have modified the launch file, you can use Eagleye by specifying the `pose_estimator_mode` through command-line arguments.

**Example of using Eagleye as the pose twist estimator:**

```bash
ros2 launch autoware_launch autoware.launch.xml \
  vehicle_model:=YOUR_VEHICLE \
  sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP \
  pose_estimator_mode:=gnss # Add this argument
```

**Example of using Eagleye as the twist estimator:**

```bash
ros2 launch autoware_launch autoware.launch.xml \
  vehicle_model:=YOUR_VEHICLE \
  sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP \
  pose_estimator_mode:=lidar # Not mandatory, as it is the default.
```
