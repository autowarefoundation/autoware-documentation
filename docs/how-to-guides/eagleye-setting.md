# [WIP] Eagleye setting

This page will show you how to set up [Eagleye](https://github.com/MapIV/eagleye), which is integrated into Autoware.

## eagleye topic setting

You must specify the eagleye input topic.
Input topics include GNSS latitude/longitude height information, GNSS speed information, IMU information, and vehicle speed information

https://github.com/MapIV/autoware_launch/blob/3f04a9dd7bc4a4c49d4ec790e3f6b9958ab822da/autoware_launch/config/localization/eagleye_config.param.yaml#L7-L16

## gnss ros drivers setting

Note that speed information besides the NavSatFix is generally required to be obtained.

ex)
1. [ublox_gps](https://github.com/KumarRobotics/ublox/tree/humble-devel/ublox_gps)

This ROS driver publishes sensor_msgs/msg/NavSatFix and geometry_msgs/msg/TwistWithCovarianceStamped required for eagleye with default settings

https://github.com/KumarRobotics/ublox/blob/humble-devel/ublox_msgs/msg/NavPVT.msg


2. [septentrio_gnss_driver](https://github.com/septentrio-gnss/septentrio_gnss_driver/tree/ros2)

Set `publish.navsatfix` and `publish.twist` in the parameter `yaml` file to `true`

https://github.com/septentrio-gnss/septentrio_gnss_driver/blob/ros2/config/gnss.yaml#L90

## eagleye parameter tuning

See below for parameter descripiton.

https://github.com/MapIV/eagleye/tree/autoware-main/eagleye_rt/config

https://github.com/MapIV/eagleye/blob/autoware-main/eagleye_util/fix2pose/launch/fix2pose.xml

## Autoware Setting for eagleye

Please refer to the following PR when introducing eagleye to your autoware.

https://github.com/autowarefoundation/autoware/pull/3261

## Things to keep in mind about eagleye

- Note that in the case of eagleye with a single antenna, it is necessary to run about a minute outdoors  before the pose estimation starts.

If you are RTK, you can speed up the time to POSE estimation by
Please specify the fix output from the GNSS ROS driver instead of eagleye/fix below.
https://github.com/MapIV/eagleye/blob/autoware-main/eagleye_util/fix2pose/launch/fix2pose.xml#L7 

If you set `use_multi_antenna_mode` in `eagleye_rt`.launch to `true` and input `PoseStamped` with the attitude estimated by GNSS multi antennas in [heading_node](https://github.com/MapIV/eagleye/blob/develop-ros2/eagleye_rt/launch/eagleye_rt.launch.xml#L42-L55), you can estimate without running.

Position and attitude estimation with GNSS multi-antenna, for example, can be done with the following packages
[gnss_compass_ros](https://github.com/MapIV/gnss_compass_ros/tree/main-ros2)