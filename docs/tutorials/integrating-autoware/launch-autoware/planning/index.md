# Planning launch files

## Overview

The Autoware planning stack starts with `autoware.launch.xml`, as the [Launch Autoware](../index.md) page explains. If `launch_planning` is `true`, `autoware.launch.xml` includes `component_planning.launch.xml` of the `autoware_launch` package. This file includes `planning.launch.xml` of the [`autoware_planning_launch`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_planning_launch) package. The following diagram shows the planning launch flow in the `autoware_launch` repository.

{{ diagram("images/planning-launch-flow.svg", "planning.launch.xml reads a module preset and includes mission planning, the trajectory source that planning_setting selects, and the planning validator.") }}

!!! note

    The Autoware project is a large project.
    Therefore, as we manage the Autoware project, we utilize specific
    arguments in the launch files.
    ROS 2 offers an argument-overriding feature for these launch files.
    Please refer to [the official ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameter-overrides) for further information.
    For instance,
    if we define an argument at the top-level launch,
    it will override the value on lower-level launches.

## component_planning.launch.xml

The [`component_planning.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_launch/launch/components/component_planning.launch.xml) launch file is the main planning launch file of the `autoware_launch` package. It includes [`planning.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_universe_launch/autoware_planning_launch/launch/planning.launch.xml) of `autoware_planning_launch`. The parameter files are in the [`autoware_planning_config`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_planning_config) package. `component_planning.launch.xml` gives these arguments of `autoware.launch.xml` to the planning launch files:

- **`planning_setting:`** This argument selects the source of the trajectory. With `rule_based` (the default), the scenario planning modules make the trajectory. With `diffusion_planner`, the learned [Diffusion Planner](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_diffusion_planner/) makes it. In both cases, the planning validator checks the trajectory and publishes `/planning/trajectory`.
- **`planning_module_preset:`** This argument selects the module preset. `planning.launch.xml` includes the file `config/preset/<planning_module_preset>_preset.yaml` of `autoware_planning_config`. The default value is `default`, which selects [`default_preset.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_universe_launch/autoware_planning_config/config/preset/default_preset.yaml).

## Module preset

The preset file declares the launch arguments that enable or disable the planning modules. For more information, see [how to enable or disable a planning module](https://autowarefoundation.github.io/autoware_universe/main/planning/#how-to-enable-or-disable-planning-module). These are some of its arguments:

- **`launch_obstacle_cruise_module:`** This argument enables the [obstacle cruise module](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/) of the motion velocity planner. The motion velocity planner also has the [obstacle stop module](https://autowarefoundation.github.io/autoware_core/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/) (`launch_obstacle_stop_module`) and the obstacle slow down module (`launch_obstacle_slow_down_module`). All three are `true` by default. To disable the obstacle cruise module, change the preset file:

  ```diff
    - arg:
        name: launch_obstacle_cruise_module
  -     default: "true"
  +     default: "false"
  ```

- **`launch_surround_obstacle_checker:`** This argument enables the [surround_obstacle_checker](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_surround_obstacle_checker/). The default value is `false`. To enable it, change the preset file:

  ```diff
    - arg:
        name: launch_surround_obstacle_checker
  -     default: "false"
  +     default: "true"
  ```

- **`velocity_smoother_type:`** This argument selects the type of the smoother of the [velocity_smoother](https://autowarefoundation.github.io/autoware_core/main/planning/autoware_velocity_smoother/) package. For the available smoother types, see its documentation. For example, to change the smoother type from `JerkFiltered` to `L2`, change the preset file:

  ```diff
    - arg:
        name: velocity_smoother_type
  -     default: JerkFiltered
  +     default: L2
  ```

!!! note

    You can also use these arguments as command-line arguments:
    ```bash
    ros2 launch autoware_launch autoware.launch.xml ... launch_surround_obstacle_checker:=true velocity_smoother_type:=L2 ...
    ```

Autoware 2.0.0 has no `use_experimental_lane_change_function`, `cruise_planner_type`, or `use_surround_obstacle_check` argument. The preset arguments above replace the last two. `use_experimental_lane_change_function` changed the collision check of the lane change module. Its options are now lane change parameters, for example `use_all_predicted_paths` in the `collision_check` section of `config/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml` of `autoware_planning_config`.

To keep your changes apart from the default preset, copy `default_preset.yaml` to a new file, for example `my_preset.yaml`, in the same directory. Build `autoware_planning_config` again, so that the install directory has the new file. Then launch Autoware with `planning_module_preset:=my`.

The parameters of each planning module are in the `autoware_planning_config` package, for example in `config/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/`.
