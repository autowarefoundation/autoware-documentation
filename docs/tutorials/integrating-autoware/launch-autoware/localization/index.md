# Localization launch files

## Overview

The Autoware localization stack starts with `autoware.launch.xml`, as the [Launch Autoware](../index.md) page explains. If `launch_localization` is `true`, `autoware.launch.xml` includes `component_localization.launch.xml` of the `autoware_launch` package. This file includes the launch files of the `autoware_localization_launch` package in the `/localization` namespace. The following diagram shows the localization launch flow in the `autoware_launch` repository.

{{ diagram("images/localization-launch-flow.svg", "component_localization.launch.xml includes the estimators that pose_source and twist_source select, and always the util, fusion filter, and error monitor launch files.") }}

!!! note

    The Autoware project is a large project.
    Therefore, as we manage the Autoware project, we utilize specific
    arguments in the launch files.
    ROS 2 offers an argument-overriding feature for these launch files.
    Please refer to [the official ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameter-overrides) for further information.
    For instance,
    if we define an argument at the top-level launch,
    it will override the value on lower-level launches.

## component_localization.launch.xml

The [`component_localization.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_launch/launch/components/component_localization.launch.xml) launch file is the main localization launch file of the `autoware_launch` package. It selects the localization units, and it includes one launch file for each unit from the [`autoware_localization_launch`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_localization_launch) package. There is no single `localization.launch.xml` file any more. The parameter files are in the [`autoware_localization_config`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_localization_config) package. You can change the localization launch arguments in `component_localization.launch.xml`.

The localization launch files support several pose estimators and twist estimators. `component_localization.launch.xml` has two arguments that select the estimators:

- **`pose_source:`** This argument selects the pose estimator. The options are `ndt` (the default), `yabloc`, `eagleye`, `artag`, and `lidar-marker`. By default, Autoware launches [ndt_scan_matcher](https://autowarefoundation.github.io/autoware_core/main/localization/autoware_ndt_scan_matcher/) as the pose estimator. You can use YabLoc as a camera-based localization method. For more information about YabLoc, see the [YabLoc documentation](https://autowarefoundation.github.io/autoware_universe/main/localization/yabloc/). You can also use Eagleye as a localization method that uses GNSS, IMU, and wheel odometry. For more information, see the [Eagleye](./eagleye/index.md) page.

  To run several pose estimators, join their names with underscores, for example `ndt_yabloc`. Then Autoware also launches the [pose_estimator_arbiter](https://autowarefoundation.github.io/autoware_universe/main/localization/autoware_pose_estimator_arbiter/), which selects the active pose estimator.

  You can set the `pose_source` argument in `component_localization.launch.xml`. For example, to use Eagleye as the pose source, update `component_localization.launch.xml` like this:

  ```diff
  - <arg name="pose_source" default="ndt" description="A string consisting of ndt, yabloc, eagleye, artag and lidar-marker joined by underscores no matter the order. e.g. ndt_yabloc"/>
  + <arg name="pose_source" default="eagleye" description="A string consisting of ndt, yabloc, eagleye, artag and lidar-marker joined by underscores no matter the order. e.g. ndt_yabloc"/>
  ```

  You can also override the launch argument on the command line:

  ```bash
  ros2 launch autoware_launch autoware.launch.xml ... pose_source:=eagleye
  ```

- **`twist_source:`** This argument selects the twist estimator. The options are `gyro_odom` (the default) and `eagleye`. By default, Autoware launches [gyro_odometer](https://autowarefoundation.github.io/autoware_core/main/localization/autoware_gyro_odometer/) as the twist estimator. You can also use Eagleye as the twist source. For more information, see the [Eagleye](./eagleye/index.md) page. To change the twist source to Eagleye, update `component_localization.launch.xml` like this:

  ```diff
  - <arg name="twist_source" default="gyro_odom" description="select twist_estimator. gyro_odom, eagleye"/>
  + <arg name="twist_source" default="eagleye" description="select twist_estimator. gyro_odom, eagleye"/>
  ```

  Or you can override the launch argument on the command line:

  ```bash
  ros2 launch autoware_launch autoware.launch.xml ... twist_source:=eagleye
  ```

- **`input_pointcloud:`** This argument sets the input point cloud of the localization point cloud pipeline in the `util` launch file. If NDT is a pose source, this pipeline downsamples the point cloud for NDT. The default value is `/sensing/lidar/concatenated/pointcloud`, which is the output of the [point cloud preprocessing](https://autowarefoundation.github.io/autoware_universe/main/sensing/autoware_pointcloud_preprocessor/) pipeline of sensing. You can change this value to the topic of one LiDAR. In `sample_sensor_kit_launch`, each LiDAR publishes its preprocessed point cloud on `/sensing/lidar/<name>/pointcloud_before_sync`. For example:

  ```diff
  - <arg name="input_pointcloud" default="/sensing/lidar/concatenated/pointcloud" description="The topic will be used in the localization util module"/>
  + <arg name="input_pointcloud" default="/sensing/lidar/top/pointcloud_before_sync" description="The topic will be used in the localization util module"/>
  ```

- **`gnss_enabled:`** If this argument is `true` (the default), the pose initializer uses the GNSS pose for the initial position estimation.

The unit launch files of `autoware_localization_launch` give some topics as fixed values. A launch argument does not change these topics. For example, `pose_twist_estimator/gyro_odometer.launch.xml` sets the input twist topic of the gyro odometer:

```xml
<arg name="input_vehicle_twist_with_covariance_topic" value="/sensing/vehicle_velocity_converter/twist_with_covariance"/>
```

To change this topic, change the value in your fork of `autoware_launch`.

**Note:** The vehicle velocity converter package provides the input topic of the gyro odometer. The sensor kit launches this package. For more information, see the [vehicle velocity converter package](https://autowarefoundation.github.io/autoware_core/main/sensing/autoware_vehicle_velocity_converter/).

## Note when using non NDT pose estimator

!!! note

    NDT is the pose estimator that most users use. So the default diagnostic graph monitors the diagnostics of NDT.
    If you use a pose estimator other than NDT, the diagnostics of NDT do not arrive, and their state becomes stale. The diagnostic graph treats a stale state as an error. Then the localization state is an error, and Autoware cannot change to the autonomous mode.

To work around this, change the diagnostic graph. The diagnostic graph aggregator reads the graph file that the `diagnostic_graph_aggregator_graph_path` argument of `autoware.launch.xml` sets. Its default value is [`autoware-main.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_launch/config/system/diagnostics/autoware-main.yaml), which includes [`localization.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_launch/config/system/diagnostics/localization.yaml). In `localization.yaml`, the `/autoware/localization/scan_matching_status` unit reads the `scan_matching_status` diagnostics of `ndt_scan_matcher`:

```yaml
- path: /autoware/localization/scan_matching_status
  type: diag
  node: ndt_scan_matcher
  name: scan_matching_status
```

To prevent the error when NDT does not run, copy all files of the `config/system/diagnostics` directory to a new directory. `autoware-main.yaml` includes the other files from its own directory (`$(dirname)`), so keep them together. In your copy of `localization.yaml`, remove the link to `/autoware/localization/scan_matching_status` from the `/autoware/localization` unit. Then set `diagnostic_graph_aggregator_graph_path` to your copy of `autoware-main.yaml`. For more information about the graph files, see the [diagnostic_graph_aggregator](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_diagnostic_graph_aggregator/) documentation.
