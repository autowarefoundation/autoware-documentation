# Tuning planning

In this section, we will update, evaluate, and fine-tune the Autoware planning modules,
with a specific emphasis on the lane driving modules within the YTU campus environment.
The Autoware planning side consists of the following main lane driving sections:
**Behavior planning and motion planning**. Our focus will be on fine-tuning these modules to
enhance our planning performance in the campus environment.

## Planning parameter tuning

### Behavior planning tuning

#### Behavior velocity planner

The Behavior velocity planner is a planner that adjusts velocity based on traffic rules.
It loads modules as plugins. Please refer to the package documentation for more
information about these modules.

To enable or disable Behavior velocity planner modules,
we will enable or disable the necessary plugin modules in the [`default_preset.yaml` file](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/preset/default_preset.yaml) according to our preferences.

- For example, in the YTU campus environment, there are many speed bumps, pedestrians, and specific areas where autonomous driving is not permitted.
  We will enable the speed bump module to handle these conditions.
  The out of lane module also helps in these areas.
  In Autoware 2.0, it is a module of the motion velocity planner, and `launch_out_of_lane_module` is already `true` by default.

To enable a module,
we need to set the launch\_<MODULE-NAME> argument
to `true` for this module in the `default_preset.yaml` parameter file:

!!! note "[`default_preset.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/preset/default_preset.yaml) parameter file:"

    ```diff
      - arg:
          name: launch_speed_bump_module
    -     default: "false"
    +     default: "true"
    ```

#### Speed bump module tuning

- Our vehicle's cruising speed is set to `15 km/h`.
  Therefore,
  we will decrease the default speed bump velocity limits in the `speed_bump.param.yaml`
  to allow for minimum and maximum speeds when going over speed bumps.

!!! note "[`speed_bump.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/speed_bump.param.yaml) parameter file:"

    ```diff
    - min_speed: 1.39  # [m/s] = [5 kph]
    + min_speed: 1.11  # [m/s] = [4 kph]
    - max_speed: 2.78  # [m/s] = [10 kph]
    + max_speed: 2.22  # [m/s] = [8 kph]
    ```

- Also, we will increase the slow_start_margin parameter to provide
  a greater margin for slowing down when approaching a speed bump.
  Please refer to the speed bump module page for more information
  about these parameters. It is essential to fine-tune these modules
  based on your specific vehicle and environmental conditions.

!!! note "[`speed_bump.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/speed_bump.param.yaml) parameter file:"

    ```diff
    - slow_start_margin: 1.0
    + slow_start_margin: 2.0
    ```

- The following image illustrates the virtual wall created by the slow start margin
  of the speed bump module. If you increase or decrease the `slow_start_margin` parameter,
  you will observe that the position of the virtual wall changes is relative to the speed bump.

<figure markdown>
  ![speed-bump-tuning](images/speed-bump.png){ align=center }
  <figcaption>
    The speed bump module slow start margin is demonstrated as a virtual wall in RViz.
  </figcaption>
</figure>

#### Avoidance

The Avoidance module plays a pivotal role in Autoware's behavior planning scene modules,
offering rule-based avoidance capabilities. It provides the flexibility to define behavior
based on intuitive parameters like lateral jerk and avoidance distance margin, enabling
adaptable avoidance strategies tailored to your specific environment. This module operates
within lane constraints, requiring access to lane structure information to ensure compliance
with traffic rules. For example, it triggers indicator signals when the vehicle crosses a lane.
The key distinction between motion and behavior modules in the planning stack is their consideration
of traffic rules.
Please refer to the [`Avoidance Module` page](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/) for more information about the module's capabilities.

We will modify and update certain avoidance rules and margin parameters
to handle with the specific conditions of our YTU campus environment
and the capabilities of our vehicle.

- First, we will disable the use of opposite lanelets while avoiding,
  as we do not want to utilize them in our YTU Campus environment.
  The default avoidance behavior in our environment is as depicted in the image below.

<figure markdown>
  ![avoidance-opposite-lane](images/avoidance-opposite-lane.png){ align=center }
  <figcaption>
    When `use_lane_type` is set to `opposite_direction_lane`,
the path can be shifted to the opposite lane if the ego vehicle cannot fit in the current lane.
  </figcaption>
</figure>

- To disable the use of the opposite lanelet in the avoidance, we will set `use_lane_type`
  to `same_direction_lane` in the `static_obstacle_avoidance.param.yaml` file:

!!! note "[`static_obstacle_avoidance.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml) parameter file:"

    ```diff
      avoidance:
    -   use_lane_type: "opposite_direction_lane"
    +   use_lane_type: "same_direction_lane"
    ```

Now, we expect the avoidance module not to utilize the opposite lane in our area.
If the vehicle cannot fit past an obstacle in the same lanelet, it will come to a stop,
as depicted in the image below:

<figure markdown>
  ![avoidance-current-lane](images/avoidance-current-lane.png){ align=center }
  <figcaption>
    When `use_lane_type` is set to `same_direction_lane`,
the path cannot be shifted to the opposite lane, even if the ego vehicle cannot fit in the current lane.
  </figcaption>
</figure>

Since we disabled the use of opposite lanelets for the Avoidance module,
there may be instances where we cannot avoid objects due to the shifted path
not fitting within the available lanelet.
As a small vehicle, we will reduce the `soft_margin` parameter to decrease the distance
between objects and the ego vehicle.
You can adjust these changes to any target object according to your preference.
For more information on these parameters, please refer to
the [avoidance module documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/).

!!! note "[`static_obstacle_avoidance.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml) parameter file:"

    ```diff
      avoidance:
        target_object:
          car:
            lateral_margin:
    -         soft_margin: 0.5                            # [m]
    +         soft_margin: 0.3                            # [m]
    ```

<figure markdown>
  ![avoidance-current-lane](images/avoidance-lateral-margin.png){ align=center }
  <figcaption>
    The Avoidance module's behavior changes when the `soft_margin` value is decreased,
such that the shifted path fits within the lanelet.
  </figcaption>
</figure>

- Also, you can choose which objects can be avoided along the path.
  In the YTU campus environment, as mentioned earlier, there are many
  pedestrians, and we do not want to perform avoidance maneuvers for them.
  Therefore, we will disable the avoidance maneuver for pedestrians by
  modifying the target value in the [`static_obstacle_avoidance.param.yaml` file](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml).
  You can disable any target object according to your preference.

!!! note "[`static_obstacle_avoidance.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml) parameter file:"

    ```diff
      avoidance:
        target_filtering:
          target_type:
    -       pedestrian: true
    +       pedestrian: false
    ```

### Motion Planning Tuning

#### Path optimizer

The path optimizer generates a kinematically feasible
and collision-free trajectory based on the input path and drivable area.
It updates the trajectory's position and orientation but retains the velocity from the input path.
Please refer to the [Path Optimizer](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/)
and [Model Predictive Trajectory (MPT)](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/docs/mpt/)
documentation page for more information about this package.

- The YTU Campus environment features numerous U-turns, narrow roads, and roundabouts.
  Given our vehicle's maximum steering angle is insufficient for these conditions,
  we must consider our steering angle limit (defined in the vehicle_info.param.yaml)
  to navigate these road types without exceeding road boundaries.
  Therefore, we will enable the steer_limit_constraint parameter for the path optimizer:

!!! note "[`path_optimizer.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml) parameter file:"

    ```diff
      mpt:
        option:
    -     steer_limit_constraint: false
    +     steer_limit_constraint: true
    ```

- Additionally, we will modify our mechanism for checking outside the drivable area.
  By default, the path optimizer checks the four corner points of the MPT footprint.
  However, this may lead to incorrect information in some situations, as shown in the following image.

{{ diagram("images/path-optimizer-drivable-area-check.svg", "The default check uses only the four corner points of the MPT footprint, so it does not find a footprint side that leaves the drivable area.") }}

- To address this issue,
  we will enable the `use_footprint_polygon_for_outside_drivable_area_check` parameter
  to consider the footprint as a polygon and check if it exceeds the Lanelet2 boundaries.

!!! note "[`path_optimizer.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml) parameter file:"

    ```diff
      option:
    -   use_footprint_polygon_for_outside_drivable_area_check: false # If false, only the footprint's corner points are considered.
    +   use_footprint_polygon_for_outside_drivable_area_check: true # If false, only the footprint's corner points are considered.
    ```

#### Obstacle stop module

In Autoware 2.0, the motion velocity planner has three modules for obstacles:
the obstacle stop module, the obstacle slow down module, and the obstacle cruise module.
The obstacle stop module inserts stop points into the trajectory for the detected objects.
It can also stop for the input point cloud, which adds safety, because the point cloud also includes obstacles that the perception does not detect.
By default, the point cloud check of the obstacle stop module is off.
The obstacle cruise module uses only the dynamic objects.

The `default_preset.yaml` file enables or disables each module:

!!! note "[`default_preset.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/preset/default_preset.yaml) parameter file:"

    ```yaml
      - arg:
          name: launch_obstacle_stop_module
          default: "true"
      - arg:
          name: launch_obstacle_slow_down_module
          default: "true"
      - arg:
          name: launch_obstacle_cruise_module
          default: "true"
    ```

For more information on the parameters and inner working algorithms of the obstacle stop module,
please refer to the [documentation page](https://autowarefoundation.github.io/autoware_core/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/).

In our campus environment, we want the vehicle to stop also for the point cloud.
So we turn on the point cloud check of the obstacle stop module:

!!! note "[`obstacle_stop.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml) parameter file:"

    ```diff
          obstacle_filtering:
            check_inside:
              default: true
    -         pointcloud: false
    +         pointcloud: true
    ```

In our YTU environment and test vehicle conditions,
due to the short vehicle width, the trajectory is also short.
To improve the detection range of the obstacle stop module,
we will increase the lateral margin for point cloud obstacles.
This margin must not be larger than the `lateral_margin` of `pointcloud_preprocessing` in `motion_velocity_planner.param.yaml` (2.0 m by default), because the planner removes the points outside that margin first.

!!! note "[`obstacle_stop.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml) parameter file:"

    ```diff
          obstacle_filtering:
            ...
            lateral_margin: # lateral margin between the obstacles and ego's footprint
              nominal:
                default: 0.3
                unknown: 0.3
    -           pointcloud: 0.3
    +           pointcloud: 0.6
    ```

The following image illustrates the motion behavior of the trajectory,
considering the lateral margin and the stop margin parameters.
It was taken with the `obstacle_stop_planner` package of an earlier Autoware version.

<figure markdown>
  ![obstacle-stop-planner](images/obstacle-stop-planner.png){ align=center }
  <figcaption>
  The obstacle stop planner checks the detection area with the given lateral margin.
If any points from the input cloud are detected,
it inserts a stop point behind the detected point cloud with the stop margin.
  </figcaption>
</figure>

Autoware has a lot of changeable parameters and methods.
The mentioned sample parameter tuning examples are just a small part of what is possible.
For more information on the modules and their parameters,
please refer to the [Autoware Universe documentation](https://autowarefoundation.github.io/autoware_universe/main/).
