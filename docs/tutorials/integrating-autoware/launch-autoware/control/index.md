# Control launch files

## Overview

The Autoware control stack starts with `autoware.launch.xml`, as the [Launch Autoware](../index.md) page explains. If `launch_control` is `true`, `autoware.launch.xml` includes `component_control.launch.xml` of the `autoware_launch` package. This file includes `control.launch.xml` of the [`autoware_control_launch`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_control_launch) package. The following diagram shows the control launch flow in the `autoware_launch` repository.

{{ diagram("images/control-launch-flow.svg", "control.launch.xml reads a module preset and includes the trajectory follower, command gate, external command, and control checker launch files.") }}

!!! note

    The Autoware project is a large project.
    Therefore, as we manage the Autoware project, we utilize specific
    arguments in the launch files.
    ROS 2 offers an argument-overriding feature for these launch files.
    Please refer to [the official ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameter-overrides) for further information.
    For instance,
    if we define an argument at the top-level launch,
    it will override the value on lower-level launches.

## component_control.launch.xml

The [`component_control.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_launch/launch/components/component_control.launch.xml) launch file is the main control launch file of the `autoware_launch` package. It includes [`control.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_universe_launch/autoware_control_launch/launch/control.launch.xml) of `autoware_control_launch`. The parameter files are in the [`autoware_control_config`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_control_config) package. `control.launch.xml` puts the composable nodes into two containers in the `/control` namespace: `control_container` and `control_check_container`. You can change the control launch arguments in `component_control.launch.xml`. These are some of its arguments:

- **`lateral_controller_mode:`** This argument selects the lateral controller algorithm. The default value is `mpc` ([MPC lateral controller](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_mpc_lateral_controller/)). To change it to [pure pursuit](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_pure_pursuit/), update your `component_control.launch.xml` file:

  ```diff
  - <arg name="lateral_controller_mode" default="mpc"/>
  + <arg name="lateral_controller_mode" default="pure_pursuit"/>
  ```

- **`use_individual_control_param:`** If this argument is `true`, the lateral and longitudinal controllers read their parameters from `config/trajectory_follower/<vehicle_id>/` of `autoware_control_config`. The default value is `false`.

- **`module_preset:`** This argument selects the module preset. `autoware.launch.xml` gives it the value of `control_module_preset`. The default value is `default`, which selects [`default_preset.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_universe_launch/autoware_control_config/config/preset/default_preset.yaml) of `autoware_control_config`.

## Module preset

The preset file declares the launch arguments that enable or disable the control modules. These are some of its arguments:

- **`launch_autonomous_emergency_braking:`** This argument enables autonomous emergency braking under specific conditions. For more information, see the [Autonomous emergency braking (AEB)](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_autonomous_emergency_braking/) page. The default value is `true`. To disable it, update the preset file:

  ```diff
    - arg:
        name: launch_autonomous_emergency_braking
  -     default: "true"
  +     default: "false"
  ```

- **`launch_predicted_path_checker:`** This argument enables the predicted path checker module. For more information, see the [Predicted Path Checker](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_predicted_path_checker/) page. The default value is `false`. To enable it, update the preset file:

  ```diff
    - arg:
        name: launch_predicted_path_checker
  -     default: "false"
  +     default: "true"
  ```

- **`trajectory_follower_mode:`** This argument selects the trajectory follower. The options are `trajectory_follower_node` (the default), `smart_mpc_trajectory_follower`, and `none`.

The preset file also has `launch_lane_departure_checker`, `launch_control_validator`, `launch_collision_detector`, `launch_obstacle_collision_checker`, `launch_external_cmd_selector`, `launch_external_cmd_converter`, and `launch_control_evaluator`.

!!! note

    You can also use these arguments as command-line arguments:
    ```bash
    ros2 launch autoware_launch autoware.launch.xml ... launch_predicted_path_checker:=true lateral_controller_mode:=pure_pursuit ...
    ```

Autoware 2.0.0 has no `enable_autonomous_emergency_braking` or `enable_predicted_path_checker` argument. The `launch_autonomous_emergency_braking` and `launch_predicted_path_checker` arguments of the preset replace them.

The parameters of each control module are in the `autoware_control_config` package, for example in `config/trajectory_follower/lateral/mpc.param.yaml`.
