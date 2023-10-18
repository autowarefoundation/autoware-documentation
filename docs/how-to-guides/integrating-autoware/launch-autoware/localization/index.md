# Localization methods

## Overview

The Autoware localization stacks start
launching at `autoware_launch.xml` as we mentioned at [Launch Autoware](../index.md) page.
The `autoware_launch` package includes `tier4_localization_component.launch.xml`
for starting localization launch files invocation from `autoware_launch.xml`.
This diagram describes some of the Autoware localization launch files flow at `autoware_launch` and `autoware.universe` packages.

<figure markdown>
  ![localization-launch-flow](images/localization_launch_flow.svg){ align=center }
  <figcaption>
    Autoware localization launch flow diagram
  </figcaption>
</figure>

The Autoware project is a large project.
Therefore, as we manage the Autoware project, we utilize specific
arguments in the launch files.
ROS 2 offers an argument-overriding feature for these launch files.
Please refer to [the official ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameter-overrides) for further information.
For instance,
if we define an argument at the top-level launch,
it will override the value on lower-level launches.

## tier4_localization_component.launch.xml

The `tier4_localization_component.launch.xml` launch file is the main localization component launch at the `autoware_launch` package.
This launch file calls `localization.launch.xml` at [tier4_localization_launch](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/tier4_localization_launch) package from `autoware.universe` repository.
We can modify localization launch arguments at tier4_localization_component.launch.xml.

The current localization launcher implemented by TIER IV supports multiple localization methods, both pose estimators and twist estimators.
`tier4_localization_component.launch.xml` has two arguments to select which estimators to launch:

- **`pose_source:`** This argument specifies the pose_estimator, currently supporting `ndt` (default), `yabloc`, `artag` and `eagleye` for localization.
  By default, Autoware launches [ndt_scan_matcher](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/ndt_scan_matcher) for pose estimator.
  You can use YabLoc as a camera-based localization method.
  For more details on YabLoc,
  please refer to the [README of YabLoc](https://github.com/autowarefoundation/autoware.universe/blob/main/localization/yabloc/README.md) in autoware.universe.
  Also, you can use Eagleye as a GNSS & IMU & wheel odometry-based localization method. For more details on Eagleye, please refer to the [Eagleye](./eagleye).

  You can set `pose_source` argument on `tier4_localization_component.launch.xml`,
  for example, if you want to use eagleye as pose_source,
  you need to update `tier4_localization_component.launch.xml` like:

  ```diff
  - <arg name="pose_source" default="ndt" description="select pose_estimator: ndt, yabloc, eagleye"/>
  + <arg name="pose_source" default="eagleye" description="select pose_estimator: ndt, yabloc, eagleye"/>
  ```

  Also, you can use command-line for overriding launch arguments:

  ```bash
  ros2 launch autoware_launch autoware.launch.xml ... pose_source:=eagleye
  ```

- **`twist_source:`** This argument specifies the twist_estimator, currently supporting `gyro_odom` (default), and `eagleye`.
  By default,
  Autoware launches [gyro_odometer](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/gyro_odometer) for twist estimator.
  Also, you can use eagleye for the twist source, please refer to the [Eagleye](./eagleye).
  If you want to change your twist source to eagleye, you can update `tier4_localization_component.launch.xml` like:

  ```diff
  - <arg name="twist_source" default="gyro_odom" description="select twist_estimator. gyro_odom, eagleye"/>
  + <arg name="twist_source" default="eagleye" description="select twist_estimator. gyro_odom, eagleye"/>
  ```

  Or you can use command-line for overriding launch arguments:

  ```bash
  ros2 launch autoware_launch autoware.launch.xml ... twist_source:=eagleye
  ```

- **`input_pointcloud:`** This argument specifies the input pointcloud of the localization pointcloud pipeline. The default value is
  `/sensing/lidar/top/outlier_filtered/pointcloud` which
  is output of the [pointcloud pre-processing](https://autowarefoundation.github.io/autoware.universe/main/sensing/pointcloud_preprocessor/) pipeline from sensing.
  You can change this value according to your LiDAR topic name,
  or you can choose to use concatenated point cloud:

  ```diff
  - <arg name="input_pointcloud" default="/sensing/lidar/top/outlier_filtered/pointcloud" description="The topic will be used in the localization util module"/>
  + <arg name="input_pointcloud" default="/sensing/lidar/concatenated/pointcloud"/>
  ```

You can add every necessary argument
to `tier4_localization_component.launch.xml` launch file like these examples.
In case, if you want to change your gyro odometer twist input topic,
you can add this argument on `tier4_localization_component.launch.xml` launch file:

```diff
+ <arg name="input_vehicle_twist_with_covariance_topic" value="<YOUR-VEHICLE-TWIST-TOPIC-NAME>"/>
```

**Note:** Gyro odometer input topic provided from velocity converter package. This package will be launched at sensor_kit. For more information,
please check [velocity converter package](https://github.com/autowarefoundation/autoware.universe/tree/main/sensing/vehicle_velocity_converter).
