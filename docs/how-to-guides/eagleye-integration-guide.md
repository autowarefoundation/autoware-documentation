# Eagleye Integration Guide

This page will show you how to set up [eagleye](https://github.com/MapIV/eagleye), which is integrated into autoware.

Discussions during the integration are as follows.
https://github.com/orgs/autowarefoundation/discussions/3257

## What is Eagleye?

Eagleye provides a cost-effective alternative to LiDAR and point cloud-based localization by using low-cost GNSS and IMU sensors to provide vehicle position and orientation.

Autoware users will be able to choose between their existing LiDAR and point cloud-based localization stacks or GNSS/IMU-based eagleye localizers, depending on their specific needs and operating environment.

There are two ways to utilize eagleye results with the autoware localization stack.

1. Feed only Twist into the EKF localizer.

![eagleye twist integration](images/eagleye-integration-guide/eagleye-twist.png)

2. Feed both Twist and Pose from eagleye into the EKF localizer.(twist can also be used with regular gyro_odometry
)

![eagleye pose integration](images/eagleye-integration-guide/eagleye-pose.png)

## Behavior when eagleye is executed by autoware

 eagleye estimates are largely based on the following

### Static Estimation
eagleye needs to be stationary for about 3~5 seconds (yaw_rate_offset_stop.estimated_interval in eagleye_config.yaml) after startup. Static estimation is performed even in environments where GNSS is not received. At this point the yaw rate offset is corrected.

### Movement Estimation
Next, it needs to travel in a straight line for about 20~30 seconds (heading.estimated_minimum_interval and velocity_scale_factor.estimated_minimum_interval) and the wheel speed scale factor and azimuth angle are estimated. At this point, the estimation of twist is complete and pose will begin to be output.

## eagleye setup

### gnss ros drivers setting

In eagleye, it is necessary to obtain velocity information from GNSS, in addition to NavSatFix.

ex)
 - [ublox_gps](https://github.com/KumarRobotics/ublox/tree/humble-devel/ublox_gps)

This ROS driver publishes sensor_msgs/msg/NavSatFix and geometry_msgs/msg/TwistWithCovarianceStamped required for eagleye with default settings

https://github.com/KumarRobotics/ublox/blob/humble-devel/ublox_msgs/msg/NavPVT.msg

 - [septentrio_gnss_driver](https://github.com/septentrio-gnss/septentrio_gnss_driver/tree/ros2)

Set `publish.navsatfix` and `publish.twist` in the parameter `yaml` file to `true`

https://github.com/septentrio-gnss/septentrio_gnss_driver/blob/ros2/config/gnss.yaml#L90

### eagleye topic setting

You must specify input topics.
Input topics include GNSS latitude/longitude height information, GNSS speed information, IMU information, and vehicle speed information

https://github.com/MapIV/autoware_launch/blob/3f04a9dd7bc4a4c49d4ec790e3f6b9958ab822da/autoware_launch/config/localization/eagleye_config.param.yaml#L7-L16


### eagleye parameter tuning

See below for parameter description.

https://github.com/MapIV/eagleye/tree/autoware-main/eagleye_rt/config

https://github.com/MapIV/eagleye/blob/autoware-main/eagleye_util/fix2pose/launch/fix2pose.xml

### Autoware Setting for Eagleye

Please refer to the following PR when introducing agleye to your autoware.

https://github.com/autowarefoundation/autoware/pull/3261

### Usage of eagleye in autoware

eagleye has a function for position estimation as pose_estimator and a function for twist correction as twist_estimator.

#### eagleye as pose_estimator

In the sample autoware, you can set pose_estimator to gnss by setting `pose_estimator_mode:=gnss` in `autoware.launch.xml`.

- Note that it does not match the map, so be careful when using maps that are out of georeference.
- In the case of a single GNSS antenna, initial position estimation takes several tens of seconds to complete after starting to run in an environment where GNSS positioning is available.

ref
https://github.com/autowarefoundation/autoware_launch/pull/200


#### eagleye as twist_estimator

In the sample autoware, you can set pose_estimator to gnss by setting `twist_estimator_mode:=gyro_odom_gnss_fusion` in `localization.launch.xml`.

- Unlike eagleye position estimation, eagleye twist estimation first outputs uncorrected raw values when activated, and then outputs corrected twists as soon as estimation is complete.


ref
https://github.com/autowarefoundation/autoware.universe/pull/2848

## Things to keep in mind about eagleye

- Note that in the case of eagleye with a single antenna, it is necessary to run about a minute outdoors  before the pose estimation starts.

If you set `use_multi_antenna_mode` in `eagleye_rt.launch.xml` to `true` and input `geometry_msgs/PoseStamped` with the attitude estimated by GNSS multi antennas in [heading_node](https://github.com/MapIV/eagleye/blob/develop-ros2/eagleye_rt/launch/eagleye_rt.launch.xml#L42-L55), you can estimate without running.

Position and attitude estimation with GNSS multi-antenna, for example, can be done with the following packages
[gnss_compass_ros](https://github.com/MapIV/gnss_compass_ros/tree/main-ros2)
