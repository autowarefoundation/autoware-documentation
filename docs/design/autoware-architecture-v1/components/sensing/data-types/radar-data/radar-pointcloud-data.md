# Radar pointcloud data pre-processing design

## Overview

### Pipeline

This diagram describes the pre-process pipeline for radar pointcloud.

{{ diagram("image/radar-pointcloud-sensing.svg", "The radar scan goes through a noise filter, then to a PointCloud2 converter and to a static points filter.") }}

!!! note

    The launch files of `autoware_launch` do not launch this pipeline, and none of its sensor kits has a radar.
    No perception package of Autoware Universe reads `RadarScan` today. The "Radar detection and tracking" box with a dashed border shows the intended user.

### Interface

- Input
  - Radar data from device
  - Odometry of the ego vehicle (`nav_msgs/msg/Odometry.msg`)
- Output
  - Dynamic radar pointcloud (`ros-perception/radar_msgs/msg/RadarScan.msg`)
  - Noise filtered radar pointcloud (`sensor_msgs/msg/PointCloud2.msg`)

### Note

- In the sensing layer, the radar pre-process packages filter noise through the `ros-perception/radar_msgs/msg/RadarScan.msg` message type with sensor coordinate.
- To use radar pointcloud data in LiDAR packages, a converter creates `sensor_msgs/msg/PointCloud2.msg` from `ros-perception/radar_msgs/msg/RadarScan.msg`.

## Reference implementations

### Data message for radars

Autoware uses radar pointcloud data type as [radar_msgs/msg/RadarScan.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarScan.msg).
In detail, please see [Data message for radars](reference-implementations/data-message.md).

### Device driver for radars

Autoware supports `ros-perception/radar_msgs/msg/RadarScan.msg` and `ros-perception/radar_msgs/msg/RadarTracks.msg` for radar drivers.

In detail, please see [Device driver for radars](reference-implementations/device-driver.md).

### Basic noise filter

- [radar_threshold_filter](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_radar_threshold_filter)

This package removes pointcloud noise with low amplitude, edge angle, too near pointcloud by threshold.
The noise depends on the radar devices and installation location.

### Filter to static/dynamic pointcloud

- [radar_static_pointcloud_filter](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_radar_static_pointcloud_filter)

This package extracts static/dynamic radar pointcloud by using doppler velocity and the ego vehicle odometry.
The static radar pointcloud can be used for localization like NDT scan matching, and the dynamic radar pointcloud can be used for dynamic object detection.

### Message converter from RadarScan to PointCloud2

- [radar_scan_to_pointcloud2](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_radar_scan_to_pointcloud2)

To use radar pointcloud with the existing LiDAR packages, the `autoware_radar_scan_to_pointcloud2` package converts `ros-perception/radar_msgs/msg/RadarScan.msg` to `sensor_msgs/msg/PointCloud2.msg`. It publishes two point clouds: `output/amplitude_pointcloud` has the amplitude as intensity, and `output/doppler_pointcloud` has the doppler velocity as intensity.

|            |           LiDAR package           |                 Radar package                 |
| :--------: | :-------------------------------: | :-------------------------------------------: |
|  message   | `sensor_msgs/msg/PointCloud2.msg` | `ros-perception/radar_msgs/msg/RadarScan.msg` |
| coordinate |             (x, y, z)             |                   (r, θ, φ)                   |
|   value    |             intensity             |          amplitude, doppler velocity          |

For considered use cases,

- Use [pointcloud_preprocessor](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_pointcloud_preprocessor) for radar scan.
- Apply obstacle segmentation like [ground segmentation](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_ground_segmentation) to radar points for LiDAR-less (camera + radar) systems.

## Appendix

### Discussion

Radar architecture design is discussed as below.

- [Discussion 2531](https://github.com/orgs/autowarefoundation/discussions/2531)
- [Discussion 2532](https://github.com/orgs/autowarefoundation/discussions/2532).
