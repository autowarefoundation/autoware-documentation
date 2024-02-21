# Sensing component design

## Overview

Sensing component is a collection of modules that apply some primitive pre-processing to the raw sensor data.

The sensor input formats are defined in this component.

## Role

- Abstraction of data formats to enable usage of sensors from various vendors
- Perform common/primitive sensor data processing required by each component

## High-level architecture

This diagram describes the high-level architecture of the Sensing Component.

![overall-sensing-architecture](image/overall-sensing-architecture.drawio.svg)

## Inputs

### Input types

| Sensor Data                                  | Message Type                                                                                                                                                                 |
| -------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Point cloud (LiDARs, depth cameras, etc.)    | [sensor_msgs/msg/PointCloud2.msg](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg)                                                    |
| Image (RGB, monochrome, depth, etc. cameras) | [sensor_msgs/msg/Image.msg](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg)                                                                |
| Radar scan                                   | [radar_msgs/msg/RadarScan.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarScan.msg)                                                                     |
| Radar tracks                                 | [radar_msgs/msg/RadarTracks.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg)                                                                 |
| GNSS-INS position                            | [sensor_msgs/msg/NavSatFix.msg](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/NavSatFix.msg)                                                        |
| GNSS-INS orientation                         | [autoware_sensing_msgs/GnssInsOrientationStamped.msg](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_sensing_msgs/msg/GnssInsOrientationStamped.msg) |
| GNSS-INS velocity                            | [geometry_msgs/msg/TwistWithCovarianceStamped.msg](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/TwistWithCovarianceStamped.msg)                  |
| GNSS-INS acceleration                        | [geometry_msgs/msg/AccelWithCovarianceStamped.msg](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/AccelWithCovarianceStamped.msg)                  |
| Ultrasonics                                  | [sensor_msgs/msg/Range.msg](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Range.msg)                                                                |

## Design by data-types

- [GNSS/INS data pre-processing design](data-types/gnss-ins-data.md)
- [Image pre-processing design](data-types/image.md)
- [Point cloud pre-processing design](data-types/point-cloud.md)
- [Radar pointcloud data pre-processing design](data-types/radar-data/radar-pointcloud-data.md)
- [Radar objects data pre-processing design](data-types/radar-data/radar-objects-data.md)
- [Ultrasonics data pre-processing design](data-types/ultrasonics-data.md)
