<!-- cspell:ignore SECONDSTRONGEST -->

# Point cloud pre-processing design

## Overview

Point cloud preprocessing is a collection of modules that apply primitive preprocessing to the raw sensor data.

This pipeline covers the flow of data from the LiDAR drivers to the perception and localization components.

## Processing pipeline

{{ diagram("image/point-cloud-pipeline.svg", "Each LiDAR has its own preprocessing chain. The concatenation node merges all LiDARs into one point cloud.") }}

The sensor kits of `autoware_launch` build this pipeline with the shared launch files of [`autoware_sensing_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_universe_launch/autoware_sensing_launch). The topics in the diagram are relative to the namespace of each LiDAR, for example `/sensing/lidar/top/`.

| Step | Node name                                              | Launch file                               | What it does                                                                                                                                                                                                                             |
| :--- | :----------------------------------------------------- | :---------------------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1    | Nebula driver, for example `velodyne_ros_wrapper_node` | `autoware_sensor_driver_launch`           | Decodes the LiDAR packets and publishes `pointcloud_raw_ex` with the `PointXYZIRCAEDT` point type.                                                                                                                                       |
| 2    | `crop_box_filter_self`                                 | `pointcloud_crop_boxes.launch.xml`        | Removes the points on the vehicle body.                                                                                                                                                                                                  |
| 3    | `crop_box_filter_mirror`                               | `pointcloud_crop_boxes.launch.xml`        | Removes the points on the side mirrors. The `secondary_crop_box` launch argument sets the name of this second crop box.                                                                                                                  |
| 4    | `distortion_corrector_node`                            | `pointcloud_preprocessor_cpu.launch.xml`  | Corrects the motion distortion of each point with the vehicle twist and the IMU angular velocity.                                                                                                                                        |
| 5    | `ring_outlier_filter`                                  | `pointcloud_preprocessor_cpu.launch.xml`  | Removes noise points, for example from rain or insects. It publishes `pointcloud_before_sync` with the `PointXYZIRC` point type, in the frame of the sensor.                                                                             |
| 6    | `concatenate_data`                                     | `pointcloud_concatenation_cpu.launch.xml` | Collects the point clouds of all LiDARs by time stamp, compensates the motion between them, and transforms them to `base_link`. It publishes `/sensing/lidar/concatenated/pointcloud` and `/sensing/lidar/concatenated/pointcloud_info`. |

The sensing pipeline does not downsample the point cloud. The users of `/sensing/lidar/concatenated/pointcloud` crop and downsample it for their own needs:

- The ground segmentation of perception crops the point cloud to the height of the vehicle and removes the ground points.
- The localization utilities crop the point cloud to the measurement range and downsample it with a voxel grid filter and a random filter. The result is the input of the NDT scan matcher.

## List of modules

The modules used here are from the [`autoware_pointcloud_preprocessor`](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_pointcloud_preprocessor) package. The [filter table of the package README](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_pointcloud_preprocessor#inner-workings--algorithms) describes each module. The package also has modules that the default pipeline does not use, for example the polygon remover, the dual return outlier filter, and the blockage diagnostics.

[`autoware_cuda_pointcloud_preprocessor`](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_cuda_pointcloud_preprocessor) has CUDA versions of the same steps. Its `cuda_pointcloud_preprocessor_node` does steps 2 to 5 in one node. Use `pointcloud_preprocessor_cuda.launch.xml` and `pointcloud_concatenation_cuda.launch.xml` instead of the CPU launch files to use it. The sensor kits of `autoware_launch` use the CPU launch files.

Autoware Core also has two point cloud filter packages: [`autoware_crop_box_filter`](https://github.com/autowarefoundation/autoware_core/tree/main/sensing/autoware_crop_box_filter) and [`autoware_downsample_filters`](https://github.com/autowarefoundation/autoware_core/tree/main/sensing/autoware_downsample_filters). The launch files of `autoware_launch` use the filters of `autoware_pointcloud_preprocessor`.

The sensor kits load the driver and the steps 2 to 5 of each LiDAR into one container, `/sensing/lidar/<LiDAR name>/pointcloud_preprocessor/pointcloud_container`. They load the concatenation node into `pointcloud_container`. The sample sensor kit enables intra-process communication for these nodes. For details, see [ROS 2 Composition](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html).

## Point cloud fields

The LiDAR driver must output a point cloud with the `PointXYZIRCAEDT` point type. [Nebula](https://github.com/tier4/nebula), the LiDAR driver of the sensor kits, publishes this point type. Autoware defines the point types in [`autoware_point_types`](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_point_types) of Autoware Core.

| Name              | Field name    | Datatype  | Derived | Description                                                                  |
| ----------------- | ------------- | --------- | ------- | ---------------------------------------------------------------------------- |
| `X`               | `x`           | `FLOAT32` | `false` | X position                                                                   |
| `Y`               | `y`           | `FLOAT32` | `false` | Y position                                                                   |
| `Z`               | `z`           | `FLOAT32` | `false` | Z position                                                                   |
| `I` (intensity)   | `intensity`   | `UINT8`   | `false` | Measured reflectivity, intensity of the point                                |
| `R` (return type) | `return_type` | `UINT8`   | `false` | Laser return type for dual return lidars                                     |
| `C` (channel)     | `channel`     | `UINT16`  | `false` | Channel ID of the laser that measured the point                              |
| `A` (azimuth)     | `azimuth`     | `FLOAT32` | `true`  | `atan2(Y, X)`, Horizontal angle from the lidar origin to the point           |
| `E` (elevation)   | `elevation`   | `FLOAT32` | `true`  | `atan2(Z, D)`, Vertical angle from the lidar origin to the point             |
| `D` (distance)    | `distance`    | `FLOAT32` | `true`  | `hypot(X, Y, Z)`, Euclidean distance from the lidar origin to the point      |
| `T` (time)        | `time_stamp`  | `UINT32`  | `false` | Nanoseconds passed since the time of the header when this point was measured |

The ring outlier filter keeps only the first six fields. Its output and the concatenated point cloud use the `PointXYZIRC` point type.

!!! note

    `A (azimuth)`, `E (elevation)`, and `D (distance)` fields are derived fields.
    They are provided by the driver to reduce the computational load on some parts of the perception stack.

!!! warning

    The concatenation node converts each input point cloud to `PointXYZIRC`.
    If an input does not have a `UINT8` `intensity`, a `UINT8` `return_type`, and a `UINT16` `channel` field, the node keeps only `x`, `y`, and `z` and sets the other fields to 0.
    For example, a `PointXYZI` input loses its intensity.
    Use this only for prototyping. The distortion corrector and the ring outlier filter require the `PointXYZIRCAEDT` layout.

### Intensity

Autoware uses the following ranges for intensity, compatible with [the VLP16 User Manual](https://usermanual.wiki/Pdf/VLP16Manual.1719942037/view):

Quoting from the VLP-16 User Manual:

> For each laser measurement, a reflectivity byte is returned in addition to distance.
> Reflectivity byte values are segmented into two ranges, allowing software to distinguish diffuse reflectors
> (e.g. tree trunks, clothing) in the low range from retroreflectors (e.g. road signs, license plates) in the high range.
> A retroreflector reflects light back to its source with a minimum of scattering.
> The VLP-16 provides its own light, with negligible separation between transmitting laser and receiving detector, so
> retroreflecting surfaces pop with reflected IR light compared to diffuse reflectors that tend to scatter reflected energy.
>
> - Diffuse reflectors report values from 0 to 100 for reflectivities from 0% to 100%.
> - Retroreflectors report values from 101 to 255, where 255 represents an ideal reflection.

In a typical point cloud without retroreflectors, all intensity points will be between 0 and 100.

<img src="https://upload.wikimedia.org/wikipedia/commons/6/6d/Retroreflective_Gradient_road_sign.jpg" width="200" alt="A retroreflective road sign">

Image source: [Retroreflective Gradient road sign on Wikimedia Commons](https://commons.wikimedia.org/wiki/File:Retroreflective_Gradient_road_sign.jpg)

But in a point cloud with retroreflectors, the intensity points will be between 0 and 255.

#### Intensity mapping for other lidar brands

##### Hesai PandarXT16

[Hesai Pandar XT16 User Manual](https://www.hesaitech.com/wp-content/uploads/2025/04/PandarXT-16_User_Manual_X02-en-250410.pdf)

This lidar has 2 modes for reporting reflectivity:

- Linear mapping
- Non-linear mapping

If you use the linear mapping mode, map [0, 255] to [0, 100] when you construct the point cloud.

If you use the non-linear mapping mode, map these ranges (Hesai to Autoware)

- [0, 251] to [0, 100] and
- [252, 254] to [101, 255]

when constructing the point cloud.

##### Livox Mid-70

[Livox Mid-70 User Manual](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Download/Mid-70/new/Livox%20Mid-70%20User%20Manual_EN_v1.2.pdf)

This lidar has 2 modes for reporting reflectivity similar to Velodyne VLP-16, only the ranges are slightly different.

Map these ranges (Livox to Autoware)

- [0, 150] to [0, 100] and
- [151, 255] to [101, 255]

when constructing the point cloud.

##### RoboSense RS-LiDAR-16

[RoboSense RS-LiDAR-16 User Manual](https://cdn.robosense.cn/20200723161715_42428.pdf)

No mapping required, same as Velodyne VLP-16.

##### Ouster OS-1-64

[Software User Manual v2.0.0 for all Ouster sensors](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf)

In the manual it is stated:

> Reflectivity [16 bit unsigned int] - sensor Signal Photons measurements are scaled based on measured range and sensor sensitivity at that range, providing an indication of target reflectivity. Calibration of this measurement has not currently been rigorously implemented, but this will be updated in a future firmware release.

As a result, map the 16-bit reflectivity to the [0, 100] range.

##### Leishen CH64W

[Leishen download page](http://www.lslidar.com/en/down) (no English user manual was found)

A user manual of this lidar says:

> Byte 7 represents echo strength, and the value range is 0-255. (Echo strength can reflect
> the energy reflection characteristics of the measured object in the actual measurement
> environment. Therefore, the echo strength can be used to distinguish objects with
> different reflection characteristics.)

As a result, map [0, 255] to the [0, 100] range.

### Return type

Various lidars support multiple return modes. Velodyne lidars support **Strongest** and **Last** return modes.

In the `PointXYZIRC` and `PointXYZIRCAEDT` types, the `R` field represents the return type with a `UINT8`.
The values come from the driver. Nebula uses the values of its `ReturnType` enumeration ([`nebula_common.hpp`](https://github.com/tier4/nebula/blob/main/src/nebula_core/nebula_core_common/include/nebula_core_common/nebula_common.hpp)):

| R (return type) | Nebula `ReturnType` |
| --------------- | ------------------- |
| `0`             | `UNKNOWN`           |
| `1`             | `LAST`              |
| `2`             | `FIRST`             |
| `3`             | `STRONGEST`         |
| `4`             | `FIRST_WEAK`        |
| `5`             | `LAST_WEAK`         |
| `6`             | `IDENTICAL`         |
| `7`             | `SECOND`            |
| `8`             | `SECONDSTRONGEST`   |
| `9`             | `FIRST_STRONGEST`   |
| `10`            | `LAST_STRONGEST`    |

### Channel

The channel field is used to identify the vertical channel of the laser that measured the point.
In various lidar manuals or literature, it can also be called _laser id_, _ring_, _laser line_.

For Velodyne VLP-16, there are 16 channels. Default order of channels in drivers are generally in firing order.

In the `PointXYZIRC` and `PointXYZIRCAEDT` types, the `C` field represents the vertical channel ID with a `UINT16`.

### Azimuth

The azimuth field gives the horizontal angle between the optical origin of the lidar and the point.
Many lidar measure this with the angle of the rotary encoder when the laser was fired, and the driver typically corrects the value based on calibration data.

In the `PointXYZIRCAEDT` type, the `A` field represents the azimuth angle in radians (clockwise) with a `FLOAT32`.

### Elevation

The elevation field gives the vertical angle between the optical origin of the lidar and the point.
In the `PointXYZIRCAEDT` type, the `E` field represents the elevation angle in radians (clockwise) with a `FLOAT32`.

#### Solid state and petal pattern lidars

!!! warning

    This section is subject to change. Following are suggestions and open for discussion.

For solid state lidars that have lines, assign row number as the channel id.

For petal pattern lidars, you can keep channel 0.

### Time stamp

In lidar point clouds, each point measurement can have its individual time stamp.
This information can be used to eliminate the motion blur that is caused by the movement of the lidar during the scan.

#### Point cloud header time

The header contains a [Time field](https://github.com/ros2/rcl_interfaces/blob/rolling/builtin_interfaces/msg/Time.msg).
The time field has 2 components:

| Field     | Type     | Description                                       |
| --------- | -------- | ------------------------------------------------- |
| `sec`     | `int32`  | Unix time (seconds elapsed since January 1, 1970) |
| `nanosec` | `uint32` | Nanoseconds elapsed since the `sec` field         |

The header of the point cloud message is expected to have the time of the earliest point it has.

!!! note

    The `sec` field is `int32` in ROS 2 Humble, Jazzy, and Rolling. The largest value it can represent is 2^31 seconds.
    As a result, it has the year 2038 problem. The ROS 2 community has not fixed this yet.

    For details, see [the year 2038 issue of `rcl_interfaces`](https://github.com/ros2/rcl_interfaces/issues/85).

#### Individual point time stamp

Each `PointXYZIRCAEDT` point type has the `T` field for representing the nanoseconds passed since the first-shot point of the point cloud.

To calculate exact time each point was shot, the `T` nanoseconds are added to the header time.

!!! note

    The `T` field is `uint32` type. The largest value it can represent is 2^32 nanoseconds, which equates to roughly
    4.29 seconds. A full cycle of a usual point cloud takes 100 ms or less. As a result, this field is large enough.
