# Radar objects data pre-processing design

## Overview

### Pipeline

This diagram describes the pre-process pipeline for radar objects.

{{ diagram("image/radar-objects-sensing.svg", "Each radar has a noise filter and a message converter. The object merger merges all radars for perception.") }}

The perception launch files read the merged radar objects from `/sensing/radar/detected_objects` by default. The [`radar_tracks_postprocessor.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_sensing_launch/launch/radar_tracks_postprocessor.launch.xml) file of `autoware_sensing_launch` launches the noise filter and the message converter for one radar. None of the sensor kits of `autoware_launch` has a radar.

### Interface

- Input
  - Radar data from device
  - Odometry of the ego vehicle (`nav_msgs/msg/Odometry.msg`)
- Output
  - Merged radar objects information

### Note

- The radar pre-process package filters noise through the `ros-perception/radar_msgs/msg/RadarTrack.msg` message type with sensor coordinate.
- It is recommended to change the coordinate system from each sensor to base_link with message converter.
- If there are multiple radar objects, the object merger package concatenates these objects.

## Input

Autoware uses radar objects data type as [radar_msgs/msg/RadarTracks.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg).
In detail, please see [Data message for radars](reference-implementations/data-message.md).

## Reference implementations

### Device driver for radars

Autoware supports `ros-perception/radar_msgs/msg/RadarScan.msg` and `ros-perception/radar_msgs/msg/RadarTracks.msg` for radar drivers.

In detail, please see [Device driver for radars](reference-implementations/device-driver.md).

### Noise filter

- [radar_tracks_noise_filter](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_radar_tracks_noise_filter)

Radar can detect x-axis velocity as doppler velocity, but cannot detect y-axis velocity. Some radar can estimate y-axis velocity inside the device, but it sometimes lack precision. This package treats these objects as noise by y-axis threshold filter.

### Message converter

- [radar_tracks_msgs_converter](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_radar_tracks_msgs_converter)

This package converts from `radar_msgs/msg/RadarTracks` into `autoware_perception_msgs/msg/DetectedObjects` and `autoware_perception_msgs/msg/TrackedObjects`. It transforms the objects to the `new_frame_id` frame (`base_link` in `autoware_sensing_launch`). It can also compensate the ego vehicle motion with the odometry (`use_twist_compensation` and `use_twist_yaw_compensation`, both `false` in `autoware_sensing_launch`).

### Object merger

- [object_merger](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_object_merger)

This package can merge 2 topics of `autoware_perception_msgs/msg/DetectedObjects`.

- [simple_object_merger](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_simple_object_merger)

This package can merge simply multiple topics of `autoware_perception_msgs/msg/DetectedObjects`.
Different from [object_merger](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_object_merger), this package doesn't use association algorithm and can merge with low calculation cost.

- [topic_tools](https://github.com/ros-tooling/topic_tools)

If a vehicle has 1 radar, the topic relay tool in `topic_tools` can be used.
Example is as below.

```xml
<launch>
  <group>
    <push-ros-namespace namespace="radar"/>
    <group>
      <push-ros-namespace namespace="front_center"/>
      <include file="$(find-pkg-share example_launch)/launch/ars408.launch.xml">
        <arg name="interface" value="can0" />
      </include>
    </group>
    <node pkg="topic_tools" exec="relay" name="radar_relay" output="log" args="front_center/detected_objects detected_objects"/>
  </group>
</launch>
```

### Radar objects adapter

- [radar_objects_adapter](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_radar_objects_adapter)

Some radar drivers also publish `autoware_sensing_msgs/msg/RadarObjects`. For example, the Nebula driver of the Continental ARS548 publishes `RadarScan`, `RadarTracks`, and `RadarObjects`. The `autoware_radar_objects_adapter` package converts `RadarObjects` into `autoware_perception_msgs/msg/DetectedObjects` without filtering. It also reads `autoware_sensing_msgs/msg/RadarInfo` and can remap the classification labels. For the message definitions, see [Data message for radars](reference-implementations/data-message.md).

## Appendix

### Discussion

Radar architecture design is discussed as below.

- [Discussion 2531](https://github.com/orgs/autowarefoundation/discussions/2531)
- [Discussion 2532](https://github.com/orgs/autowarefoundation/discussions/2532).
