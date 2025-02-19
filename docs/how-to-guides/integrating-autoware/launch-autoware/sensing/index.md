# Sensing Launch Files

## Overview

The Autoware sensing stacks start
launching at `autoware_launch.xml` as we mentioned at [Launch Autoware](../index.md) page.
The `autoware_launch` package includes `tier4_sensing_component.launch.xml`
for starting sensing launch files invocation from `autoware_launch.xml`.
This diagram describes some of the Autoware sensing launch files flow at `autoware_launch` and `autoware.universe` packages.

<figure markdown>
  ![sensing-launch-flow](images/sensing_launch_flow.svg){ align=center }
  <figcaption>
    Autoware sensing launch flow diagram
  </figcaption>
</figure>

!!! note

    The Autoware project is a large project.
    Therefore, as we manage the Autoware project, we utilize specific
    arguments in the launch files.
    ROS 2 offers an argument-overriding feature for these launch files.
    Please refer to [the official ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameter-overrides) for further information.
    For instance,
    if we define an argument at the top-level launch,
    it will override the value on lower-level launches.

The sensing launch is more related to your sensor kit,
so if you want to modify your launch, we recommend applying
these modifications to the <YOUR-SENSOR-KIT> packages.
Please look
at [creating sensor and vehicle model](../../creating-vehicle-and-sensor-model/index.md) pages for more information but there
is are some modifications on which can you done at top-level launch files.

For example, if you do not want to launch the sensor driver with Autoware,
you can disable it with a command-line argument:

```bash
ros2 launch autoware_launch autoware.launch.xml ... launch_sensing_driver:=false ...
```

Or you can change it on your `autoware.launch.xml` launch file:

```diff
- <arg name="launch_sensing_driver" default="true" description="launch sensing driver"/>
+ <arg name="launch_sensing_driver" default="false" description="launch sensing driver"/>
```
