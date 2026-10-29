# Sensing component design

## Overview

The sensing component is a collection of modules that apply primitive preprocessing to the raw sensor data.

This component also defines the input formats of the sensor data.

## Role

- Abstraction of data formats to enable usage of sensors from various vendors
- Perform common/primitive sensor data processing required by each component

## High-level architecture

This diagram shows the high-level architecture of the sensing component.

{{ diagram("image/overall-sensing-architecture.svg", "Sensing preprocesses the data of each sensor type and sends it to localization and perception.") }}

The dashed line from the camera to localization is optional. Only the camera-based localizers (YabLoc and the AR tag localizer) read the camera image.

A sensor kit launch package (for example [`sample_sensor_kit_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/sensor_kit/sample_sensor_kit_launch)) launches the drivers and the preprocessing of one sensor set. `autoware_launch` selects the package with the `sensor_model` launch argument and includes its `launch/sensing.launch.xml` in the `/sensing` namespace. The sensor kits use two shared launch packages:

- [`autoware_sensor_driver_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_universe_launch/autoware_sensor_driver_launch): launch files for the LiDAR drivers of [Nebula](https://github.com/tier4/nebula)
- [`autoware_sensing_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_universe_launch/autoware_sensing_launch): launch files for the preprocessing, for example the point cloud preprocessing, the IMU corrector, and the radar track filters

## Inputs

### Input types

| Sensor data                                  | Message type                                                                                                                                                                 |
| -------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Point cloud (LiDARs, depth cameras, etc.)    | [sensor_msgs/msg/PointCloud2.msg](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg)                                                    |
| Image (RGB, monochrome, depth, etc. cameras) | [sensor_msgs/msg/Image.msg](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg)                                                                |
| Compressed image                             | [sensor_msgs/msg/CompressedImage.msg](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/CompressedImage.msg)                                            |
| Radar scan                                   | [radar_msgs/msg/RadarScan.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarScan.msg)                                                                     |
| Radar tracks                                 | [radar_msgs/msg/RadarTracks.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg)                                                                 |
| Radar objects                                | [autoware_sensing_msgs/msg/RadarObjects.msg](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_sensing_msgs/msg/RadarObjects.msg)                       |
| GNSS-INS position                            | [sensor_msgs/msg/NavSatFix.msg](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/NavSatFix.msg)                                                        |
| GNSS-INS orientation                         | [autoware_sensing_msgs/GnssInsOrientationStamped.msg](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_sensing_msgs/msg/GnssInsOrientationStamped.msg) |
| GNSS-INS velocity                            | [geometry_msgs/msg/TwistWithCovarianceStamped.msg](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/TwistWithCovarianceStamped.msg)                  |
| GNSS-INS acceleration                        | [geometry_msgs/msg/AccelWithCovarianceStamped.msg](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/AccelWithCovarianceStamped.msg)                  |
| IMU                                          | [sensor_msgs/msg/Imu.msg](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg)                                                                    |
| Ultrasonics                                  | [sensor_msgs/msg/Range.msg](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Range.msg)                                                                |
| Vehicle velocity                             | [autoware_vehicle_msgs/msg/VelocityReport.msg](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/VelocityReport.msg)                   |

## Sensing modules

This table lists the modules that the sensor kits of `autoware_launch` use, and the topics that they publish.

| Data             | Package                                                                                                                                                                                                                                                                                                                   | Processing                                                                                 | Main output topic                                                    |
| :--------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | :----------------------------------------------------------------------------------------- | :------------------------------------------------------------------- |
| Vehicle velocity | [`autoware_vehicle_velocity_converter`](https://github.com/autowarefoundation/autoware_core/tree/main/sensing/autoware_vehicle_velocity_converter) (Core)                                                                                                                                                                 | Converts `VelocityReport` to `TwistWithCovarianceStamped`                                  | `/sensing/vehicle_velocity_converter/twist_with_covariance`          |
| GNSS/INS         | [`autoware_gnss_poser`](https://github.com/autowarefoundation/autoware_core/tree/main/sensing/autoware_gnss_poser) (Core)                                                                                                                                                                                                 | Converts the position to a pose of `base_link` in the map frame                            | `/sensing/gnss/pose_with_covariance`                                 |
| IMU              | [`autoware_imu_corrector`](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_imu_corrector) (Universe)                                                                                                                                                                                   | Corrects the gyro offset, scale, and standard deviation. Estimates the gyro bias.          | `/sensing/imu/imu_data`                                              |
| LiDAR            | [`autoware_pointcloud_preprocessor`](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_pointcloud_preprocessor) or [`autoware_cuda_pointcloud_preprocessor`](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_cuda_pointcloud_preprocessor) (Universe) | Crops, corrects the motion distortion, removes outliers, and concatenates the point clouds | `/sensing/lidar/concatenated/pointcloud`                             |
| Camera           | None in the sensing component                                                                                                                                                                                                                                                                                             | The perception and localization modules read the image topics of the camera driver         | Driver topics, for example `/sensing/camera/traffic_light/image_raw` |
| Radar            | `autoware_radar_*` packages (Universe)                                                                                                                                                                                                                                                                                    | Filters the radar noise and converts the radar data to Autoware messages                   | `/sensing/radar/detected_objects`                                    |

`/sensing/radar/detected_objects` is the default radar input of the perception launch files. None of the sensor kits in `autoware_launch` has a radar.

## Design by data types

<div class="grid cards aw-cards" markdown>

- :fa-cl-s fa-satellite: **[GNSS/INS data](data-types/gnss-ins-data.md)**

  The GNSS poser converts the GNSS fix and the INS orientation to a pose in the map frame.

- :fa-cl-s fa-camera: **[Image](data-types/image.md)**

  The camera drivers publish the images. The perception modules decompress them when necessary.

- :fa-cl-s fa-braille: **[Point cloud](data-types/point-cloud.md)**

  The point cloud preprocessing pipeline and the point fields that the LiDAR drivers must publish.

- :fa-cl-s fa-satellite-dish: **[Radar pointcloud data](data-types/radar-data/radar-pointcloud-data.md)**

  The noise filter and the static/dynamic filter for `RadarScan`, and the conversion to `PointCloud2`.

- :fa-cl-s fa-tower-broadcast: **[Radar objects data](data-types/radar-data/radar-objects-data.md)**

  The noise filter for `RadarTracks`, the conversion to `DetectedObjects`, and the object mergers.

- :fa-cl-s fa-wave-square: **[Ultrasonics data](data-types/ultrasonics-data.md)**

  The message type for ultrasonic sensors. Autoware has no ultrasonic processing today.

</div>
