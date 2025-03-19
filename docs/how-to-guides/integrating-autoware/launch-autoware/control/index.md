# Control Launch Files

## Overview

The Autoware control stacks start
launching at `autoware_launch.xml` as mentioned on the [Launch Autoware](../index.md) page.
The `autoware_launch` package includes `tier4_control_component.launch.xml`
for initiating control launch files invocation from `autoware_launch.xml`.
The diagram below illustrates the flow of Autoware control launch files within the autoware_launch and autoware_universe packages.

<figure markdown>
  ![control-launch-flow](images/control_launch_flow.svg){ align=center }
  <figcaption>
    Autoware control launch flow diagram
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

## tier4_control_component.launch.xml

The tier4_control_component.launch.xml launch file is the main control component launch in the autoware_launch package.
This launch file calls control.launch.xml from the [tier4_control_launch](https://github.com/autowarefoundation/autoware_universe/tree/main/launch/tier4_control_launch) package
within the autoware_universe repository.
We can modify control launch arguments in tier4_control_component.launch.xml.
Additionally,
we can add any other necessary arguments
that need adjustment since tier4_control_component.launch.xml serves as the top-level launch file for other control launch files.
Here are some predefined control launch arguments:

- **`lateral_controller_mode:`** This argument determines
  the lateral controller algorithm.
  The default value is `mpc`.
  To change it to pure pursuit,
  make the following update in your `tier4_control_component.launch.xml` file:

  ```diff
  - <arg name="lateral_controller_mode" default="mpc"/>
  + <arg name="lateral_controller_mode" default="pure_pursuit"/>
  ```

- **`enable_autonomous_emergency_braking:`** This argument enables autonomous emergency
  braking under specific conditions.
  Please refer to the [Autonomous emergency braking (AEB)](https://autowarefoundation.github.io/autoware_universe/main/control/autonomous_emergency_braking/) page for
  more information.
  To enable it, update the value in the `tier4_control_component.launch.xml` file:

  ```diff
  - <arg name="enable_autonomous_emergency_braking" default="false"/>
  + <arg name="enable_autonomous_emergency_braking" default="true"/>
  ```

- **`enable_predicted_path_checker:`** This argument enables the predicted path checker module.
  Please refer to the [Predicted Path Checker](https://autowarefoundation.github.io/autoware_universe/main/control/predicted_path_checker/) page for
  more information.
  To enable it, update the value in the `tier4_control_component.launch.xml` file:

  ```diff
  - <arg name="enable_predicted_path_checker" default="false"/>
  + <arg name="enable_predicted_path_checker" default="true"/>
  ```

!!! note

    You can also use this arguments as command line arguments:
    ```bash
    ros2 launch autoware_launch autoware.launch.xml ... enable_predicted_path_checker:=true lateral_controller_mode:=pure_pursuit ...
    ```

The predefined arguments in tier4_control_component.launch.xml have been explained above.
However, numerous control arguments are included in the autoware_launch control config parameters.
