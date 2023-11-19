# Coordinate system

## Overview

The commonly used coordinate systems include the world coordinate system, the vehicle coordinate system, and the sensor coordinate system.

- The world coordinate system is a fixed coordinate system that defines the physical space in the environment where the vehicle is located.

- The vehicle coordinate system is the vehicle's own coordinate system, which defines the vehicle's position and orientation in the world coordinate system.

- The sensor coordinate system is the sensor's own coordinate system, which is used to define the sensor's position and orientation in the vehicle coordinate system.

## How coordinates are used in Autoware

In Autoware, coordinate systems are typically used to represent the position and movement of vehicles and obstacles in space. Coordinate systems are commonly used for path planning, perception and control, can help the vehicle decide how to avoid obstacles and to plan a safe and efficient path of travel.

1. Transformation of sensor data

   In Autoware, each sensor has a unique coordinate system and their data is expressed in terms of the coordinates. In order to correlate the independent data between different sensors, we need to find the position relationship between each sensor and the vehicle body. Once the installation position of the sensor on the vehicle body is determined, it will remain fixed during running, so the offline calibration method can be used to determine the precise position of each sensor relative to the vehicle body.

2. ROS TF2

   The `TF2` system maintains a tree of coordinate transformations to represent the relationships between different coordinate systems. Each coordinate system is given a unique name and they are connected by coordinate transformations. How to use `TF2`, refer to the [TF2 tutorial](http://docs.ros.org/en/galactic/Concepts/About-Tf2.html).

## TF tree

In Autoware, a common coordinate system structure is shown below:

```mermaid
graph TD
    /earth --> /map
    /map --> /base_link
    /base_link --> /imu
    /base_link --> /lidar
    /base_link --> /gnss
    /base_link --> /radar
    /base_link --> /camera_link
    /camera_link --> /camera_optical_link
```

- earth: `earth` coordinate system describe the position of any point on the earth in terms of geodetic longitude, latitude, and altitude. In Autoware, the `earth` frame is only used in the `GnssInsPositionStamped` message.

- map: `map` coordinate system is used to represent the location of points on a local map. Geographical coordinate system are mapped into plane rectangular coordinate system using UTM or MGRS. The `map` frame`s axes point to the East, North, Up directions as explained in [Coordinate Axes Conventions](#coordinate-axes-conventions).

- base_link: vehicle coordinate system, the origin of the coordinate system is the center of the rear axle of the vehicle.

- imu, lidar, gnss, radar: these are sensor frames, transfer to vehicle coordinate system through mounting relationship.

- camera_link: `camera_link` is ROS standard camera coordinate system .

- camera_optical_link: `camera_optical_link` is image standard camera coordinate system.

### Estimating the `base_link` frame by using the other sensors

Generally we don't have the localization sensors physically at the `base_link` frame. So various sensors localize with respect to their own frames, let's call it `sensor` frame.

We introduce a new frame naming convention: `x_by_y`:

```yaml
x: estimated frame name
y: localization method/source
```

We cannot directly get the `sensor` frame. Because we would need the EKF module to estimate the `base_link` frame first.

Without the EKF module the best we can do is to estimate `Map[map] --> sensor_by_sensor --> base_link_by_sensor` using this sensor.

#### Example by the GNSS/INS sensor

For the integrated GNSS/INS we use the following frames:

```mermaid
flowchart LR
    earth --> Map[map] --> gnss_ins_by_gnss_ins --> base_link_by_gnss_ins
```

The `gnss_ins_by_gnss_ins` frame is obtained by the coordinates from GNSS/INS sensor. The coordinates are converted to `map` frame using the `gnss_poser` node.

Finally `gnss_ins_by_gnss_ins` frame represents the position of the `gnss_ins` estimated by the `gnss_ins` sensor in the `map`.

Then by using the static transformation between `gnss_ins` and the `base_link` frame, we can obtain the `base_link_by_gnss_ins` frame. Which represents the `base_link` estimated by the `gnss_ins` sensor.

References:

- <https://www.ros.org/reps/rep-0105.html#earth>

### Coordinate Axes Conventions

We are using East, North, Up (ENU) coordinate axes convention by default throughout the stack.

```yaml
X+: East
Y+: North
Z+: Up
```

The position, orientation, velocity, acceleration are all defined in the same axis convention.

Position by the GNSS/INS sensor is expected to be in `earth` frame.

Orientation, velocity, acceleration by the GNSS/INS sensor are expected to be in the sensor frame. Axes parallel to the `map` frame.

If roll, pitch, yaw is provided, they correspond to rotation around X, Y, Z axes respectively.

```yaml
Rotation around:
  X+: roll
  Y+: pitch
  Z+: yaw
```

References:

- <https://www.ros.org/reps/rep-0103.html#axis-orientation>

## How they can be created

1. Calibration of sensor

   The conversion relationship between every sensor coordinate system and `base_link` can be obtained through sensor calibration technology.
   Please consult the following link
   [calibrating your sensors](../../../how-to-guides/integrating-autoware/creating-vehicle-and-sensor-model/calibrating-sensors) for instructions
   on how to calibrate your sensors.

2. Localization

   The relationship between the `base_link` coordinate system and the `map` coordinate system is determined by the position and orientation of the vehicle, and can be obtained from the vehicle localization result.

3. Geo-referencing of map data

   The geo-referencing information can get the transformation relationship of `earth` coordinate system to local `map` coordinate system.
