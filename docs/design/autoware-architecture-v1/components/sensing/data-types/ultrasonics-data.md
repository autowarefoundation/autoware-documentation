# Ultrasonics data pre-processing design

## Overview

An ultrasonic sensor measures the distance to the nearest object in a small cone in front of the sensor.

!!! info "No ultrasonic processing in Autoware today"

    Autoware 2.0.0 has no package that reads, filters, or uses ultrasonic sensor data. No sensor kit in `autoware_launch` launches an ultrasonic driver, and no perception or planning module reads ultrasonic topics.

## Message type

The [input types](../index.md#input-types) table of the sensing component defines the message type for ultrasonic sensors: [`sensor_msgs/msg/Range`](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Range.msg).

A `Range` message holds one distance measurement:

| Field            | Type      | Description                                                                              |
| :--------------- | :-------- | :--------------------------------------------------------------------------------------- |
| `header`         | `Header`  | The time of the measurement and the frame of the sensor                                  |
| `radiation_type` | `uint8`   | `ULTRASOUND` (0) or `INFRARED` (1)                                                       |
| `field_of_view`  | `float32` | The size of the arc that the measured distance is valid for [rad]                        |
| `min_range`      | `float32` | The minimum distance that the sensor can measure [m]                                     |
| `max_range`      | `float32` | The maximum distance that the sensor can measure [m]                                     |
| `range`          | `float32` | The measured distance [m]                                                                |
| `variance`       | `float32` | The variance of the measured distance. `0` means unknown. Only in ROS 2 Jazzy and later. |

## Integrate an ultrasonic sensor

To use ultrasonic sensors with Autoware today, you must write the driver and the processing yourself. For example, a driver can publish one `Range` topic for each sensor in the `/sensing` namespace of your sensor kit. Then your own module must convert the measurements into a form that other components read, for example a point cloud or an occupancy grid.
