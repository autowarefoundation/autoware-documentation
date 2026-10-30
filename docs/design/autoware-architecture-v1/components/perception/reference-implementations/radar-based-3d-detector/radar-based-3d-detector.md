# Radar-based 3D detector

## Overview

### Features

The radar-based 3D detector has two aims.

#### Detect objects farther than the range of LiDAR-based 3D detection

Radar can get data from a longer distance than LiDAR (more than 100 m). If the range of LiDAR-based 3D detection is not sufficient, you can add the radar-based 3D detector. The detection range of radar-based 3D detection depends on the specification of the radar device.

#### Improve the velocity estimation of dynamic objects

Radar measures the velocity of objects. A fusion of the objects from LiDAR-based 3D detection with the radar information can give a more precise twist. This can improve object tracking, prediction, and planning functions such as adaptive cruise control.

### Whole pipeline

The radar-based 3D detector with radar objects consists of these parts:

- 3D object detection with the radar point cloud
- Noise filter
- Faraway dynamic 3D object detection
- Radar fusion to LiDAR-based 3D object detection
- Radar object tracking
- Merger of tracked objects

{{ diagram("image/radar-based-3d-detector.svg", "The radar pipeline from the radar driver to prediction.") }}

The parts are in these launch files:

| Part                                     | Status in Autoware 2.0.0                                                                                                                                                                                                                       |
| :--------------------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Noise filter and message converter       | `radar_tracks_postprocessor.launch.xml` of [`autoware_sensing_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_universe_launch/autoware_sensing_launch). A sensor kit launch file with radars can include it. |
| Faraway dynamic 3D object detection      | `radar_filter.launch.xml` of `autoware_perception_launch`. The `lidar_radar` and `camera_lidar_radar` object recognition launch files include it.                                                                                              |
| Radar object tracking with other sensors | The multi-object tracker reads the radar objects on the `radar` channel.                                                                                                                                                                       |
| Merger of tracked objects                | Only in `object_recognition_camera_lidar_radar_serial.launch.xml`.                                                                                                                                                                             |
| Radar fusion to LiDAR-based 3D detection | The package is available. The launch files of Autoware 2.0.0 do not start it.                                                                                                                                                                  |
| 3D object detection with radar points    | Not available yet.                                                                                                                                                                                                                             |

The sample sensor kits of `autoware_launch` have no radar. As a result, the default `autoware.launch.xml` does not start the radar pipeline.

### Interface

- Input
  - The message type of the radar point cloud is [`radar_msgs/msg/RadarScan`](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarScan.msg).
  - The message type of the radar objects is [`radar_msgs/msg/RadarTracks`](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg). [`autoware_radar_tracks_msgs_converter`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_radar_tracks_msgs_converter) converts them to [`autoware_perception_msgs/msg/DetectedObjects`](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/DetectedObjects.msg) on `/sensing/radar/detected_objects`, and to `autoware_perception_msgs/msg/TrackedObjects` on `/sensing/radar/tracked_objects`.
    - Input objects need to be concatenated.
    - The velocity of the input objects must be relative to the ground, so the ego motion must be compensated. The velocity filter of the faraway detection relies on this. The converter can do this compensation with the odometry (`use_twist_compensation: true`, the default of the package). The `radar_tracks_msgs_converter.param.yaml` of `autoware_sensing_launch` sets `use_twist_compensation: false`. With this file, the radar device or its driver must give the compensated velocity.
    - Input objects need to be transformed to `base_link`. The converter does this transform (`new_frame_id: base_link`).
- Output
  - Tracked objects

## Module

### Radar pointcloud 3D detection

No package detects objects from the radar point cloud yet. The sensing packages [`autoware_radar_scan_to_pointcloud2`](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_radar_scan_to_pointcloud2), [`autoware_radar_static_pointcloud_filter`](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_radar_static_pointcloud_filter), and [`autoware_radar_threshold_filter`](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_radar_threshold_filter) process `RadarScan` data.

### Noise filter and radar faraway dynamic 3D object detection

{{ diagram("image/faraway-object-detection.svg", "The noise filter and the faraway detection of radar objects.") }}

This function filters noise objects and detects faraway (more than 80 m with the default parameters) dynamic vehicles.
The main idea is that in the case where LiDAR is used, near range can be detected accurately using LiDAR pointcloud and the main role of radar is to detect distant objects that cannot be detected with LiDAR alone.
For details, see the [radar faraway dynamic object detection](faraway-object-detection.md) page.

### Radar fusion to LiDAR-based 3D object detection

- [`autoware_radar_fusion_to_detected_object`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_radar_fusion_to_detected_object)

This package contains a sensor fusion module for radar-detected objects and 3D detected objects. The fusion node can:

- Attach velocity to 3D detections when successfully matching radar data. The tracking modules use the velocity information to enhance the tracking results while planning modules use it to execute actions like adaptive cruise control.
- Improve the low confidence 3D detections when corresponding radar detections are found.

The launch files of Autoware 2.0.0 do not start this package.

### Radar object tracking

In the multi-channel launch files (`object_recognition_lidar_radar.launch.xml` and `object_recognition_camera_lidar_radar.launch.xml`), [`autoware_multi_object_tracker`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_multi_object_tracker) tracks the radar objects together with the other detections. The `radar` channel can start new tracks and the tracker trusts its existence probability and class. It does not trust the size and orientation of radar objects.

[`autoware_radar_object_tracker`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_radar_object_tracker) is a separate tracker for radar objects. The launch files of Autoware 2.0.0 do not start it.

### Merger of tracked objects

`object_recognition_camera_lidar_radar_serial.launch.xml` uses the radar tracks of the radar device:

1. [`autoware_object_sorter`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_object_sorter) (`tracked_object_sorter`) keeps the radar tracks by velocity and distance.
2. The tracked object lanelet filter of [`autoware_detected_object_validation`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_detected_object_validation) keeps the tracks on the lanes.
3. The `decorative_tracker_merger` of [`autoware_tracking_object_merger`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_tracking_object_merger) adds the radar tracks to the output of the multi-object tracker. The LiDAR-based tracks are the main tracks, and the radar tracks complement them.

## Appendix

### Customize your radar interface

The perception interface of Autoware uses `DetectedObjects`, `TrackedObjects`, and `PredictedObjects`. Some modules use other messages for their own cases. For example, the perception modules use [`DetectedObjectWithFeature`](https://github.com/tier4/tier4_autoware_msgs/tree/tier4/universe/tier4_perception_msgs/msg/object_recognition) as a custom message.

In the same way, you can make a new radar interface. For example, `RadarTrack` has no orientation information. See the [discussion in radar_msgs pull request 3](https://github.com/ros-perception/radar_msgs/pull/3), especially [the comment about the orientation](https://github.com/ros-perception/radar_msgs/pull/3#issuecomment-661599741). If you need orientation information, you can make the radar ROS driver publish `TrackedObject` directly.
