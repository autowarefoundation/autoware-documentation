# Localization mode

The Autoware provides multiple localization methods that work with multiple different sensor configurations. 
The table below shows the supported sensor configurations and their corresponding algorithms.

| localization mode | method  | map type        |
|-------------------|---------|-----------------|
| LiDAR-based       | NDT     | point cloud map |
| camera-based      | YabLoc  | vector map      |
| GNSS/IMU-based    | Eagleye | -               |

## How to launch LiDAR-based localizer (default)

By default, Autoware launches [ndt_scan_matcher](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/ndt_scan_matcher) for self-localization.
## How to launch camera-based localizer

You can use YabLoc as a camera-based localization method. For more details on YabLoc, please refer to the [YabLoc Guide](yabloc-guide.md).

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

## How to launch GNSS/IMU-based localizer

You can use Eagleye as a GNSS/IMU-based localization method. For more details on Eagleye, please refer to the [Eagleye Guide](eagleye-guide.md).

Eagleye has a function for position estimation and twist estimation, namely `pose_estimator` and `twist_estimator`, respectively.
When running Eagleye in twist_estimator mode, ndt_scan_matcher is used as the pose_estimator.
Eagleye will improve scan matching by providing accurate twists using GNSS doppler.

To use Eagleye, it requires both specifying the command-line arguments and modifying the launch file.

### 1. Modifying Autoware launch files

When using Eagleye, comment out `tier4_localization_component.launch.xml` and start `map4_localization_component.launch.xml` in [`autoware.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/autoware.launch.xml).
Please refer to the following snippet for the modification details:

```xml
  <!-- Localization -->
  <group if="$(var launch_localization)">
    <!-- <include file="$(find-pkg-share autoware_launch)/launch/components/tier4_localization_component.launch.xml"/> -->
    <include file="$(find-pkg-share autoware_launch)/launch/components/map4_localization_component.launch.xml"/>
  </group>
```

NOTE: Please refer to [`map4_localization_launch`](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/map4_localization_launch) in the `autoware.universe` package and [`map4_localization_component.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/map4_localization_component.launch.xml) in `autoware_launch` package for information on how to modify the localization launch.

### 2. Execution command

Once you have modified the launch file, you can use Eagleye by specifying the `pose_estimator_mode` through command-line arguments.

The following table shows the available arguments, along with the corresponding estimation methods.

| pose_estimator_mode | pose_estimator method           | twist_estimator method          |
|---------------------|---------------------------------|---------------------------------|
| `lidar`(default)    | ndt_scan_matcher (default)      | eagleye as twist_estiamtor      |
| `gnss`              | eagleye as pose_twist_estiamtor | eagleye as pose_twist_estiamtor |


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