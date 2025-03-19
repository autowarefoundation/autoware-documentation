# Planning Launch Files

## Overview

The Autoware planning stacks start
launching at `autoware_launch.xml` as mentioned on the [Launch Autoware](../index.md) page.
The `autoware_launch` package includes `tier4_planning_component.launch.xml`
for initiating planning launch files invocation from `autoware_launch.xml`.
The diagram below illustrates the flow of Autoware planning launch files within the autoware_launch and autoware.universe packages.

<figure markdown>
  ![planning-launch-flow](images/planning_launch_flow.svg){ align=center }
  <figcaption>
    Autoware planning launch flow diagram
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

## tier4_planning_component.launch.xml

The `tier4_planning_component.launch.xml` launch file is the main planning component launch at the `autoware_launch` package.
This launch file calls `planning.launch.xml` at [tier4_planning_launch](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/tier4_planning_launch) package from `autoware.universe` repository.
We can modify planning launch arguments at tier4_planning_component.launch.xml.
Also,
we can add any other necessary arguments
that we want
to change it since `tier4_planning_component.launch.xml` is the top-level launch file of other planning launch files.
Here are some predefined planning launch arguments:

- **`use_experimental_lane_change_function:`** This argument enables
  `enable_collision_check_at_prepare_phase`, `use_predicted_path_outside_lanelet`,
  and `use_all_predicted_path` options for Autoware for experimental lane changing
  (for more information, please refer to [lane_change documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/docs/behavior_path_planner_lane_change_design/)).
  The default value is True.
  To set it to False, make the following change in the `tier4_planning_component.launch.xml` file:

  ```diff
  - <arg name="use_experimental_lane_change_function" default="true"/>
  + <arg name="use_experimental_lane_change_function" default="false"/>
  ```

- **`cruise_planner_type:`** There are two types of cruise planners in Autoware: [obstacle_stop_planner](https://autowarefoundation.github.io/autoware_universe/main/planning/obstacle_stop_planner/)
  and [obstacle_cruise_planner](https://autowarefoundation.github.io/autoware_universe/main/planning/obstacle_cruise_planner/). For specifications on these cruise planner types,
  please refer to the package documentation. The default cruise planner is `obstacle_stop_planner`.
  To change it to obstacle_cruise_planner, update the argument value in the `tier4_planning_component.launch.xml` file:

  ```diff
  - <arg name="cruise_planner_type" default="obstacle_stop_planner" description="options: obstacle_stop_planner, obstacle_cruise_planner, none"/>
  + <arg name="cruise_planner_type" default="obstacle_cruise_planner" description="options: obstacle_stop_planner, obstacle_cruise_planner, none"/>
  ```

- **`use_surround_obstacle_check:`** This argument enables the [surround_obstacle_checker](https://autowarefoundation.github.io/autoware_universe/main/planning/surround_obstacle_checker/)
  for Autoware. If you want to disable it, you can do in the
  `tier4_planning_component.launch.xml` file:

  ```diff
  - <arg name="use_surround_obstacle_check" default="true"/>
  + <arg name="use_surround_obstacle_check" default="false"/>
  ```

- **`velocity_smoother_type:`** This argument specifies the type of smoother
  for the [motion_velocity_smoother](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_smoother/) package. Please consult the documentation
  for detailed information about available smoother types. For instance, if
  you wish to change your smoother type from JerkFiltered to L2, you can do
  in the tier4_planning_component.launch.xml file.:

  ```diff
  - <arg name="velocity_smoother_type" default="JerkFiltered" description="options: JerkFiltered, L2, Analytical, Linf(Unstable)"/>
  + <arg name="velocity_smoother_type" default="L2" description="options: JerkFiltered, L2, Analytical, Linf(Unstable)"/>
  ```

!!! note

    You can also use this arguments as command line arguments:
    ```bash
    ros2 launch autoware_launch autoware.launch.xml ... use_surround_obstacle_check:=false velocity_smoother_type:=L2 ...
    ```

The predefined arguments in tier4_planning_component.launch.xml have been explained above.
However, numerous planning arguments are included in the autoware_launch planning config parameters.
