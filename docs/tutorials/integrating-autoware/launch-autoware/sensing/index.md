# Sensing launch files

## Overview

The Autoware sensing stack starts with `autoware.launch.xml`, as the [Launch Autoware](../index.md) page explains. If `launch_sensing` is `true`, `autoware.launch.xml` includes [`component_sensing.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_launch/launch/components/component_sensing.launch.xml) of the `autoware_launch` package. This file includes `sensing.launch.xml` of your sensor kit launch package, `<sensor_model>_launch`, in the `/sensing` namespace. The following diagram shows the sensing launch flow with [`sample_sensor_kit_launch`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/sensor_kit/sample_sensor_kit_launch).

{{ diagram("images/sensing-launch-flow.svg", "component_sensing.launch.xml includes sensing.launch.xml of the sensor kit, which includes the launch files of each sensor type.") }}

!!! note

    The Autoware project is a large project.
    Therefore, as we manage the Autoware project, we utilize specific
    arguments in the launch files.
    ROS 2 offers an argument-overriding feature for these launch files.
    Please refer to [the official ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameter-overrides) for further information.
    For instance,
    if we define an argument at the top-level launch,
    it will override the value on lower-level launches.

The sensor kit launch package holds the sensing pipeline of your vehicle. It uses the shared launch files of two packages:

- [`autoware_sensor_driver_launch`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_sensor_driver_launch): launch files for the LiDAR drivers of [Nebula](https://github.com/tier4/nebula), for example `velodyne_VLS128.launch.xml` and `hesai_XT32.launch.xml`
- [`autoware_sensing_launch`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_sensing_launch): launch files for the preprocessing, for example the point cloud preprocessor and the point cloud concatenation

The sensing launch depends on your sensor kit. So, to change the sensing launch, change your `<YOUR-SENSOR-KIT>_launch` package. For more information, see the [creating sensor and vehicle model](../../creating-vehicle-and-sensor-model/index.md) pages. You can also change some things in the top-level launch files.

For example, if you do not want to launch the sensor drivers with Autoware, disable them with a command-line argument:

```bash
ros2 launch autoware_launch autoware.launch.xml ... launch_sensing_driver:=false ...
```

Or you can change it in your `autoware.launch.xml` launch file:

```diff
- <arg name="launch_sensing_driver" default="true" description="launch sensing driver"/>
+ <arg name="launch_sensing_driver" default="false" description="launch sensing driver"/>
```

`component_sensing.launch.xml` gives this value to `sensing.launch.xml` as `launch_driver`. In `sample_sensor_kit_launch`, `launch_driver:=false` stops the GNSS driver and the Velodyne monitor, and the Nebula LiDAR drivers do not connect to the sensors. The IMU driver node of the sample is commented out, so you add your own IMU driver to `imu.launch.xml`. The preprocessing still runs. You can use this, for example, to replay the sensor data of a rosbag.
