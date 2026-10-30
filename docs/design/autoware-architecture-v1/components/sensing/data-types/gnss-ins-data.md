# GNSS/INS data pre-processing design

## Overview

A GNSS (Global Navigation Satellite System) receiver measures the position of its antenna as latitude, longitude, and altitude. An INS (Inertial Navigation System) also gives the orientation. The sensing component converts these measurements to a pose of `base_link` in the map frame. Localization uses this pose.

<div class="aw-pipeline" markdown>

- :fa-cl-s fa-satellite: <span>**GNSS/INS driver**<br>NavSatFix and orientation</span>
- :fa-cl-s fa-location-crosshairs: <span>**GNSS poser**<br>Pose in the map frame</span>
- :fa-cl-s fa-map-location-dot: <span>**Localization**<br>Initial pose and regularization</span>

</div>

The [`autoware_gnss_poser`](https://github.com/autowarefoundation/autoware_core/tree/main/sensing/autoware_gnss_poser) package of Autoware Core does this conversion. It is the only GNSS preprocessing node in the sensor kits of `autoware_launch`.

## Inputs

| Topic in the sample sensor kit                                              | Message type                                                                                                                                                                   | Description                                                       |
| :-------------------------------------------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------- |
| `/sensing/gnss/ublox/nav_sat_fix` or `/sensing/gnss/septentrio/nav_sat_fix` | [`sensor_msgs/msg/NavSatFix`](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/NavSatFix.msg)                                                            | Position of the antenna. `header.frame_id` is the antenna frame.  |
| `/autoware_orientation`                                                     | [`autoware_sensing_msgs/msg/GnssInsOrientationStamped`](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_sensing_msgs/msg/GnssInsOrientationStamped.msg) | Orientation from the INS, in the ENU (East, North, Up) convention |
| `/map/map_projector_info`                                                   | `autoware_map_msgs/msg/MapProjectorInfo`                                                                                                                                       | Projection of the map, from the map projection loader             |

`GnssInsOrientationStamped` holds a quaternion and the root mean square error of the rotation around each axis (`rmse_rotation_x`, `rmse_rotation_y`, and `rmse_rotation_z`).

## Outputs

| Topic                                | Message type                                   | Description                                                   |
| :----------------------------------- | :--------------------------------------------- | :------------------------------------------------------------ |
| `/sensing/gnss/pose`                 | `geometry_msgs/msg/PoseStamped`                | Pose of `base_link` in the map frame                          |
| `/sensing/gnss/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped`  | The same pose with a covariance                               |
| `/sensing/gnss/fixed`                | `autoware_internal_debug_msgs/msg/BoolStamped` | `true` if the receiver has a fix                              |
| TF `map` to `gnss_base_link`         | `tf2_msgs/msg/TFMessage`                       | The same pose as a transform, with the `gnss_base_frame` name |

## Processing in `autoware_gnss_poser`

For each `NavSatFix` message, the node does these steps:

<div class="aw-steps" markdown>

1. **Wait for the map projection.** The node needs `/map/map_projector_info`. If the map uses the `Local` projector type, the node cannot convert the GNSS position and publishes no pose.
2. **Check the fix.** The node publishes the fix status. If the `NavSatFix` status is lower than `STATUS_FIX`, the node skips the message.
3. **Project the position.** The node projects the latitude and longitude to the map frame with the map projection. It converts the altitude to the vertical datum of the map.
4. **Smooth the position (optional).** With `gnss_pose_pub_method` set to `1` (average) or `2` (median), the node uses the last `buff_epoch` positions. The default `0` uses each position as it is.
5. **Set the orientation.** If `use_gnss_ins_orientation` is `true` (default), the node uses the last `GnssInsOrientationStamped` message. If it is `false`, the node calculates the heading from the difference between two positions.
6. **Move the pose to `base_link`.** The node transforms the antenna pose to `base_link` with the static TF from the antenna frame. If this TF is not available, the node publishes the antenna pose.

</div>

The position covariance comes from the `NavSatFix` message. If the message has no covariance, the node uses `10.0` for x, y, and z. The orientation covariance is the square of the INS rotation error. Without INS orientation, the node uses `0.1` for roll and pitch and `1.0` for yaw.

## GNSS receivers in the sample sensor kit

The [`gnss.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml) file of `sample_sensor_kit_launch` selects the receiver with the `gnss_receiver` launch argument:

- `ublox` (default): the `ublox_gps` driver with the `zed_f9p.yaml` configuration
- `septentrio`: the `septentrio_gnss_driver` with the `mosaic_x5_rover.launch.xml` launch file

`autoware_sensing_launch` also has a [`gnss_postprocessor.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_sensing_launch/launch/gnss_postprocessor.launch.xml) file for Septentrio receivers. It launches `autoware_gnss_poser` and the `septentrio_heading_converter.py` script. This script converts the `AttEuler` heading of the Septentrio driver to a `GnssInsOrientationStamped` message.

## Users of the GNSS pose

- The pose initializer ([`autoware_pose_initializer`](https://github.com/autowarefoundation/autoware_core/tree/main/localization/autoware_pose_initializer)) reads `/sensing/gnss/pose_with_covariance` to find the initial pose.
- The NDT scan matcher reads the same topic for its regularization. Regularization is off in the default parameters.
- Eagleye (`pose_source` or `twist_source` set to `eagleye`) does not use `autoware_gnss_poser`. It reads `/sensing/gnss/ublox/nav_sat_fix` and `/sensing/gnss/ublox/navpvt` directly.

## GNSS-INS velocity and acceleration

The [input types](../index.md#input-types) table also lists `geometry_msgs/msg/TwistWithCovarianceStamped` for the GNSS-INS velocity and `geometry_msgs/msg/AccelWithCovarianceStamped` for the GNSS-INS acceleration. Autoware defines no topic for them. The [Sensing interface](../../../interfaces/components/sensing.md) page gives "Not defined" as their topic.

No node of the sensing component reads these messages. Eagleye can read a GNSS velocity as `TwistWithCovarianceStamped` (`velocity_source_type: 3` of its `gnss_converter` node). The default `eagleye_config.param.yaml` of `autoware_launch` reads the u-blox `NavPVT` message (`velocity_source_type: 2`) instead. No node of Autoware 2.0.0 reads a GNSS acceleration.
