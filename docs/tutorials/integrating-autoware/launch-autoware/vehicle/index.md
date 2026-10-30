# Vehicle launch files

## Overview

The Autoware vehicle stack starts with `autoware.launch.xml`, as the [Launch Autoware](../index.md) page explains. If `launch_vehicle` is `true`, `autoware.launch.xml` includes `vehicle.launch.xml` of the [`tier4_vehicle_launch`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/tier4_universe_launch/tier4_vehicle_launch) package directly. No component launch file is between them. The following diagram shows the vehicle launch flow in the `autoware_launch` repository.

{{ diagram("images/vehicle-launch-flow.svg", "vehicle.launch.xml starts robot_state_publisher and includes the vehicle interface launch file of your vehicle.") }}

!!! note

    The Autoware project is a large project.
    Therefore, as we manage the Autoware project, we utilize specific
    arguments in the launch files.
    ROS 2 offers an argument-overriding feature for these launch files.
    Please refer to [the official ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameter-overrides) for further information.
    For instance,
    if we define an argument at the top-level launch,
    it will override the value on lower-level launches.

`vehicle.launch.xml` does these two things:

- It starts `robot_state_publisher`. This node reads the URDF that `tier4_vehicle_launch/urdf/vehicle.xacro` makes from the `<vehicle_model>_description` and `<sensor_model>_description` packages, and publishes the static transforms of the vehicle and the sensors.
- If `launch_vehicle_interface` is `true`, it includes `vehicle_interface.launch.xml` of the `<vehicle_model>_launch` package, for example `sample_vehicle_launch`. This file starts the vehicle interface of your vehicle.

`autoware.launch.xml` also gives `vehicle.launch.xml` two paths: the `config` directory of `<sensor_model>_description`, which holds the sensor calibration files, and `autoware_launch/config/vehicle/raw_vehicle_cmd_converter/raw_vehicle_cmd_converter.param.yaml`.

You do not have many configuration choices in the vehicle launch files, because the parameters are in your vehicle and sensor kit packages. But you can disable the vehicle interface. For example, to run `robot_state_publisher` without the vehicle interface, launch Autoware with this command-line argument:

```bash
ros2 launch autoware_launch autoware.launch.xml ... launch_vehicle_interface:=false ...
```

Or you can change it in your `autoware.launch.xml` launch file:

```diff
- <arg name="launch_vehicle_interface" default="true" description="launch vehicle interface"/>
+ <arg name="launch_vehicle_interface" default="false" description="launch vehicle interface"/>
```
