# Planning component design

## Overview

The Planning component generates the trajectory message that will be subscribed to by the Control component based on the environmental state obtained from the Localization and the Perception components.

## Requirements

The goal of the Planning component is to generate a trajectory (path and velocity) of the ego vehicle that is safe and well-regulated while satisfying the given mission.

**Goals:**

- The basic functions are provided so that a simple ODD can be defined.
- The functionality is modularized to accommodate the third-party components. That is, a complicated or realistic ODD needs not be defined by the basic functions provided by default.
- The capability is extensible with the third-party components or the decision of human operators.
- The mechanism and policy are separated to allow the system or operators to change the behavior of the ego vehicle. Ultimately speaking, the policy can be set to crash an obstacle and the mechanism always follows. Otherwise, the system is not safe from the design point of view.

**Non-goals:**

- The Planning component is not self-contained but can be extended with third parties.
- The Planning component is not aimed at the complete functionality and capability.
- The Planning component is not designed to always outperform human drivers.
- The Planning component is not capable of “never crashes”.

## High-level architecture

This diagram describes the high-level architecture of the Planning Component.

![overall-planning-architecture](image/high-level-planning-diagram.drawio.svg)

The Planning component consists of the following sub-components:

- **Mission Planning**: Calculates the route based on the given goal and map information.
- **Scenario Planning**: Determines the trajectory based on the current scenario, such as Lane Driving or Parking.
  - **Lane Driving**: Calculates the trajectory for driving within constructed lanes.
    - **Behavior Planner**: Calculates suitable trajectory based on safety considerations and traffic rules.
    - **Motion Planner**: Calculates suitable trajectory for the vehicle by taking into account safety factors, vehicle motion considerations, and instructions from the behavior planner.
  - **Parking**: Calculates the trajectory for parking in unstructured areas.
- **Validation**: Verifies the safety of the trajectory.

Each component contains some modules that can be dynamically loaded and unloaded based on the situation. For instance, the Behavior Planning component includes modules such as lane change, intersection, and crosswalk modules.

Our planning components are built based on the microautonomy architecture with Autoware. We adopt a modular system framework where the tasks are implemented as modules that can be dynamically loaded and unloaded to achieve different features depending on the given use cases.

## Component interface

The following describes the input/output concept between Planning Component and other components. See the [Planning Component Interface (WIP)](../../autoware-interfaces/components/planning.md) page for the current implementation.

### Input to the planning component

- **From Map**
  - Vector map: Contains all static information about the environment, including lane connection information for route planning, lane geometry for generating a reference path, and traffic rule-related information.
- **From Perception**
  - Detected object information: Provides real-time information about objects that cannot be known in advance, such as pedestrians and other vehicles. The Planning Component plans maneuvers to avoid collisions with these objects.
  - Detected obstacle information: Supplies real-time information about the location of obstacles, which is more primitive than Detected Object and used for emergency stops and other safety measures.
  - Occupancy map information: Offers real-time information about the presence of pedestrians and other vehicles and occluded area information.
  - Traffic light recognition result: Provides the current state of each traffic light in real time. The Planning Component extracts relevant information for the planned path and determines whether to stop at intersections.
- **From Localization**
  - Vehicle motion information: Includes the ego vehicle's position, velocity, acceleration, and other motion-related data.
- **From System**
  - Operation mode: Indicates whether the vehicle is operating in Autonomous mode.
- **From Human Machine Interface (HMI)**
  - Feature execution: Allows for executing/authorizing autonomous driving operations, such as lane changes or entering intersections, by human operators.
- **From API Layer**
  - Goal: Represents the final position that the Planning Component aims to reach.
  - Checkpoint: Represents a midpoint along the route to the destination. This is used during route calculation.
  - Velocity limit: Sets the maximum speed limit for the vehicle.

### Output from the planning component

- **To Control**
  - Trajectory: Provides a smooth sequence of pose, twist, and acceleration that the Control Component must follow. The trajectory is typically 10 seconds long with a 0.1-second resolution.
  - Turn Signals: Controls the vehicle's turn indicators, such as right, left, hazard, etc. based on the planned maneuvers.
- **To System**
  - Diagnostics: Reports the state of the Planning Component, indicating whether the processing is running correctly and whether a safe plan is being generated.
- **To Human Machine Interface (HMI)**
  - Feature execution availability: Indicates the status of operations that can be executed or are required, such as lane changes or entering intersections.
  - Trajectory candidate: Shows the potential trajectory that will be executed after the user's execution.
- **To API Layer**
  - Planning factors: Provides information about the reasoning behind the current planning behavior. This may include the position of target objects to avoid, obstacles that led to the decision to stop, and other relevant information.

### Internal interface in the planning component

- **Mission Planning to Scenario Planning**
  - Route: Offers guidance for the path that needs to be followed from the starting point to the destination. This path is determined based on information such as lane IDs defined on the map. At the route level, it doesn't explicitly indicate which specific lanes to take, and the route can contain multiple lanes.
- **Behavior Planning to Motion Planning**
  - Path: Provides a rough position and velocity to be followed by the vehicle. These path points are usually defined with an interval of about 1 meter. Although other interval distances are possible, it may impact the precision or performance of the planning component.
  - Drivable area: Defines regions where the vehicle can drive, such as within lanes or physically drivable areas. It assumes that the motion planner will calculate the final trajectory within this defined area.
- **Scenario Planning to Validation**
  - Trajectory: Defines the desired positions, velocities, and accelerations which the Control Component will try to follow. Trajectory points are defined at intervals of approximately 0.1 seconds based on the trajectory velocities.
- **Validation to Control Component**
  - Trajectory: Same as above but with some additional safety considerations.

## How to add new modules (WIP)

As mentioned in the goal session, this planning module is designed to be extensible by third-party components. For specific instructions on how to add new modules and expand its functionality, please refer to the provided documentation or guidelines (WIP).

## Supported Functions

| Feature                                      | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Requirements                                                                | Figure                                                                          |
| -------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| Route Planning                               | Plan route from the ego vehicle position to the destination.<br> <br> Reference implementation is in [Mission Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/mission_planner/), enabled by launching the `mission_planner` node.                                                                                                                                                                                              | - Lanelet map (driving lanelets)                                            | ![route-planning](image/features-route-planning.drawio.svg)                     |
| Path Planning from Route                     | Plan path to be followed from the given route. <br> <br> Reference implementation is in [Behavior Path Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/).                                                                                                                                                                                                                                                | - Lanelet map (driving lanelets)                                            | ![lane-follow](image/features-lane-follow.drawio.svg)                           |
| Obstacle Avoidance                           | Plan path to avoid obstacles by steering operation. <br> <br> Reference implementation is in [Avoidance](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/docs/behavior_path_planner_avoidance_design/), [Obstacle Avoidance Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_avoidance_planner/). Enable flag in parameter: `launch obstacle_avoidance_planner true`        | - objects information                                                       | ![obstacle-avoidance](image/features-avoidance.drawio.svg)                      |
| Path Smoothing                               | Plan path to achieve smooth steering. <br> <br> Reference implementation is in [Obstacle Avoidance Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_avoidance_planner/).                                                                                                                                                                                                                                               | - Lanelet map (driving lanelet)                                             | ![path-smoothing](image/features-path-smoothing.drawio.svg)                     |
| Narrow Space Driving                         | Plan path to drive within the drivable area. Furthermore, when it is not possible to drive within the drivable area, stop the vehicle to avoid exiting the drivable area. <br> <br> Reference implementation is in [Obstacle Avoidance Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_avoidance_planner/).                                                                                                           | - Lanelet map (high-precision lane boundaries)                              | ![narrow-space-driving](image/features-narrow-space-driving.drawio.svg)         |
| Lane Change                                  | Plan path for lane change to reach the destination. <br> <br> Reference implementation is in [Lane Change](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/docs/behavior_path_planner_lane_change_design/).. Enable flag in both parameters:                                                                                                                                                                      | - Lanelet map (driving lanelets)                                            | ![lane-change](image/features-lane-change.drawio.svg)                           |
| Pull Over                                    | Plan path for pull over to park at the road shoulder. <br> <br> Reference implementation is in [Goal Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/docs/behavior_path_planner_goal_planner_design/).                                                                                                                                                                                                   | - Lanelet map (shoulder lane)                                               | ![pull-over](image/features-pull-over.drawio.svg)                               |
| Pull Out                                     | Plan path for pull over to start from the road shoulder. <br> <br> Reference implementation is in [Pull Out Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/docs/behavior_path_planner_pull_out_design/).                                                                                                                                                                                                 | - Lanelet map (shoulder lane)                                               | ![pull-out](image/features-pull-out.drawio.svg)                                 |
| Path Shift                                   | Plan path in lateral direction in response to external instructions. <br> <br> Reference implementation is in [Side Shift Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/docs/behavior_path_planner_side_shift_design/).                                                                                                                                                                                 | - None                                                                      | ![side-shift](image/features-side-shift.drawio.svg)                             |
| Obstacle Stop                                | Plan velocity to stop for an obstacle on the path. <br> <br> Reference implementation is in [Obstacle Stop Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_stop_planner/), [Obstacle Cruise Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_cruise_planner/). `launch obstacle_stop_planner` and enable flag: `TODO`, `launch obstacle_cruise_planner` and enable flag: `TODO` | - objects information                                                       | ![obstacle-stop](image/features-obstacle-stop.drawio.svg)                       |
| Obstacle Deceleration                        | Plan velocity to decelerate for an obstacle located around the path. <br> <br> Reference implementation is in [Obstacle Stop Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_stop_planner/), [Obstacle Cruise Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_cruise_planner/).                                                                                                | - objects information                                                       | ![obstacle-decel](image/features-obstacle-decel.drawio.svg)                     |
| Adaptive Cruise Control                      | Plan velocity to follow the vehicle driving in front of the ego vehicle. <br> <br> Reference implementation is in [Obstacle Stop Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_stop_planner/), [Obstacle Cruise Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_cruise_planner/).                                                                                            | - objects information                                                       | ![adaptive-cruise](image/features-adaptive-cruise.drawio.svg)                   |
| Decelerate for cut-in vehicles               | Plan velocity to avoid a risk for cutting-in vehicle to ego lane. <br> <br> Reference implementation is in [Obstacle Cruise Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_cruise_planner/).                                                                                                                                                                                                                         | - objects information                                                       | ![cut-in](image/features-cut-in.drawio.svg)                                     |
| Surround Check at starting                   | Plan velocity to prevent moving when an obstacle exists around the vehicle. <br> <br> Reference implementation is in [Surround Obstacle Checker](https://autowarefoundation.github.io/autoware.universe/main/planning/surround_obstacle_checker/).                                                                                                                                                                                                           | - objects information                                                       | ![surround-check](image/features-surround-check.drawio.svg)                     |
| Curve Deceleration                           | Plan velocity to decelerate the speed on a curve. <br> <br> Reference implementation is in [Motion Velocity Smoother](https://autowarefoundation.github.io/autoware.universe/main/planning/motion_velocity_smoother/).                                                                                                                                                                                                                                       | - None                                                                      | ![decel-on-curve](image/features-decel-on-curve.drawio.svg)                     |
| Curve Deceleration for Obstacle              | Plan velocity to decelerate the speed on a curve for a risk of obstacle collision around the path. <br> <br> Reference implementation is in [Obstacle Velocity Limiter](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_velocity_limiter/).                                                                                                                                                                                    | - objects information <br> - Lanelet map (static obstacle)                  | ![decel-on-curve-obstacles](image/features-decel-on-curve-obstacles.drawio.svg) |
| Crosswalk                                    | Plan velocity to stop or decelerate for pedestrians approaching or walking on a crosswalk. <br> <br> Reference implementation is in [Crosswalk Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/crosswalk-design/).                                                                                                                                                                                    | - objects information <br> - Lanelet map (pedestrian crossing)              | ![crosswalk](image/features-crosswalk.drawio.svg)                               |
| Intersection Oncoming Vehicle Check          | Plan velocity for turning right/left at intersection to avoid a risk with oncoming other vehicles. <br> <br> Reference implementation is in [Intersection Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/intersection-design/).                                                                                                                                                                      | - objects information <br> - Lanelet map (intersection lane and yield lane) | ![intersection](image/features-intersection.drawio.svg)                         |
| Intersection Blind Spot Check                | Plan velocity for turning right/left at intersection to avoid a risk with other vehicles or motorcycles coming from behind blind spot. <br> <br> Reference implementation is in [Intersection Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/intersection-design/).                                                                                                                                  | - objects information <br> - Lanelet map (intersection lane)                | ![blind-spot](image/features-blind-spot.drawio.svg)                             |
| Intersection Occlusion Check                 | Plan velocity for turning right/left at intersection to avoid a risk with the possibility of coming vehicles from occlusion area. <br> <br> Reference implementation is in [Intersection Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/intersection-design/).                                                                                                                                       | - objects information <br> - Lanelet map (intersection lane)                | ![intersection-occlusion](image/features-intersection-occlusion.drawio.svg)     |
| Intersection Traffic Jam Detection           | Plan velocity for intersection not to enter the intersection when a vehicle is stopped ahead for a traffic jam. <br> <br> Reference implementation is in [Intersection Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/intersection-design/).                                                                                                                                                         | - objects information <br> - Lanelet map (intersection lane)                | ![intersection-traffic-jam](image/features-intersection-traffic-jam.drawio.svg) |
| Traffic Light                                | Plan velocity for intersection according to a traffic light signal. <br> <br> Reference implementation is in [Traffic Light Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/traffic-light-design/).                                                                                                                                                                                                   | - Traffic light color information                                           | ![traffic-light](image/features-traffic-light.drawio.svg)                       |
| Run-out Check                                | Plan velocity to decelerate for the possibility of nearby objects running out into the path. <br> <br> Reference implementation is in [Run Out Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/run-out-design/).                                                                                                                                                                                      | - objects information                                                       | ![run-out](image/features-run-out.drawio.svg)                                   |
| Stop Line                                    | Plan velocity to stop at a stop line. <br> <br> Reference implementation is in [Stop Line Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/stop-line-design/).                                                                                                                                                                                                                                         | - Lanelet map (stop line)                                                   | ![stop-line](image/features-stop-line.drawio.svg)                               |
| Occlusion Spot Check                         | Plan velocity to decelerate for objects running out from occlusion area, for example, from behind a large vehicle. <br> <br> Reference implementation is in [Occlusion Spot Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/occlusion-spot-design/).                                                                                                                                                  | - objects information <br> - Lanelet map (private/public lane)              | ![occlusion-spot](image/features-occlusion-spot.drawio.svg)                     |
| No Stop Area                                 | Plan velocity not to stop in areas where stopping is prohibited, such as in front of the fire station entrance. <br> <br> Reference implementation is in [No Stopping Area Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/no-stopping-area-design/).                                                                                                                                                 | - Lanelet map (no stopping area)                                            | ![no-stopping-area](image/features-no-stopping-area.drawio.svg)                 |
| Merge from Private Area to Public Road       | Plan velocity for entering the public road from a private driveway to avoid a risk of collision with pedestrians or other vehicles. <br> <br> Reference implementation is in [Merge from Private Area Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/merge-from-private-design/).                                                                                                                    | - objects information <br> - Lanelet map (private/public lane)              | WIP                                                                             |
| Speed Bump                                   | Plan velocity to decelerate for speed bumps. <br> <br> Reference implementation is in [Speed Bump Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/speed-bump-design/).                                                                                                                                                                                                                                | - Lanelet map (speed bump)                                                  | ![speed-bump](image/features-speed-bump.drawio.svg)                             |
| Detection Area                               | Plan velocity to stop at the corresponding stop when an object exist in the designated detection area. <br> <br> Reference implementation is in [Detection Area Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/detection-area-design/).                                                                                                                                                              | - Lanelet map (detection area)                                              | ![detection-area](image/features-detection-area.drawio.svg)                     |
| Out of ODD area                              | Plan velocity to stop before exiting the area designated by ODD (Operational Design Domain). <br> <br> Reference implementation is in (WIP).                                                                                                                                                                                                                                                                                                                 | - Lanelet map (invalid lanelet)                                             | WIP                                                                             |
| Collision Detection when deviating from lane | Plan velocity to avoid conflict with other vehicles driving in the another lane when the ego vehicle is deviating from own lane. <br> <br> Reference implementation is in [Out of Lane Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/out-of-lane-design/).                                                                                                                                          | - objects information <br> - Lanelet map (driving lane)                     | WIP                                                                             |
| Parking                                      | Plan path and velocity for given goal in parking area. <br> <br> Reference implementation is in [Free Space Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/freespace_planner/).                                                                                                                                                                                                                                               | - objects information <br> - Lanelet map (parking area)                     | ![parking](image/features-parking.drawio.svg)                                   |
| Autonomous Emergency Braking (AEB)           | Perform an emergency stop if a collision with an object ahead is anticipated. It is noted that this function is expected as a final safety layer, and this should work even in the event of failures in the Localization or Perception system. <br> <br> Reference implementation is in [Out of Lane Module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/out-of-lane-design/).                            | - Primitive objects                                                         | ![aeb](image/features-aeb.drawio.svg)                                           |
| Minimum Risk Maneuver (MRM)                  | Provide appropriate MRM (Minimum Risk Maneuver) instructions when a hazardous event occurs. For example, when a sensor trouble found, send an instruction for emergency braking, moderate stop, or pulling over to the shoulder, depending on the severity of the situation. <br> <br> Reference implementation is in TODO                                                                                                                                   | - TODO                                                                      | WIP                                                                             |
| Trajectory Validation                        | Check the planned trajectory is safe. If it is unsafe, take appropriate action, such as modify the trajectory, stop sending the trajectory or report to the autonomous driving system. <br> <br> Reference implementation is in [Planning Validator](https://autowarefoundation.github.io/autoware.universe/main/planning/planning_validator/).                                                                                                              | - None                                                                      | ![trajectory-validation](image/features-trajectory-validation.drawio.svg)       |
| Running Lane Map Generation                  | Generate lane map from localization data recorded in manual driving. <br> <br> Reference implementation is in WIP                                                                                                                                                                                                                                                                                                                                            | - None                                                                      | WIP                                                                             |
| Running Lane Optimization                    | Optimize the centerline (reference path) of the map to make it smooth considering the vehicle kinematics. <br> <br> Reference implementation is in [Static Centerline Optimizer](https://autowarefoundation.github.io/autoware.universe/main/planning/static_centerline_optimizer/).                                                                                                                                                                         | - Lanelet map (driving lanes)                                               | WIP                                                                             |

<!-- ![supported-functions](image/planning-functions.drawio.svg) -->

## Reference Implementation

The following diagram describes the reference implementation of the Planning component. By adding new modules or extending the functionalities, various ODDs can be supported.

_Note that some implementation does not adhere to the high-level architecture design and require updating._

![reference-implementation](image/planning-diagram.drawio.svg)

For more details, please refer to the design documents in each package.

- [_mission_planner_](https://autowarefoundation.github.io/autoware.universe/main/planning/mission_planner/): calculate route from start to goal based on the map information.
- [_behavior_path_planner_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/): calculates path and drivable area based on the traffic rules.
  - [_lane_following_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/#lane-following)
  - [_lane_change_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/#lane-change)
  - [_avoidance_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/#avoidance)
  - [_pull_over_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/#pull-over)
  - [_pull_out_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/#pull-out)
  - _side_shift_
- [_behavior_velocity_planner_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/): calculates max speed based on the traffic rules.
  - [_detection_area_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/detection-area-design/)
  - [_blind_spot_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/blind-spot-design/)
  - [_cross_walk_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/crosswalk-design/)
  - [_stop_line_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/stop-line-design/)
  - [_traffic_light_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/traffic-light-design/)
  - [_intersection_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/intersection-design/)
  - [_no_stopping_area_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/no-stopping-area-design/)
  - [_virtual_traffic_light_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/virtual-traffic-light-design/)
  - [_occlusion_spot_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/occlusion-spot-design/)
  - [_run_out_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/run-out-design/)
- [_obstacle_avoidance_planner_](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_avoidance_planner/): calculate path shape under obstacle and drivable area constraints
- [_surround_obstacle_checker_](https://autowarefoundation.github.io/autoware.universe/main/planning/surround_obstacle_checker/): keeps the vehicle being stopped when there are obstacles around the ego-vehicle. It works only when the vehicle is stopped.
- [_obstacle_stop_planner_](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_stop_planner/): When there are obstacles on or near the trajectory, it calculates the maximum velocity of the trajectory points depending on the situation: stopping, slowing down, or adaptive cruise (following the car).
  - [_stop_](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_stop_planner/#obstacle-stop-planner_1)
  - [_slow_down_](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_stop_planner/#slow-down-planner)
  - [_adaptive_cruise_](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_stop_planner/#adaptive-cruise-controller)
- [_costmap_generator_](https://autowarefoundation.github.io/autoware.universe/main/planning/costmap_generator/): generates a costmap for path generation from dynamic objects and lane information.
- [_freespace_planner_](https://autowarefoundation.github.io/autoware.universe/main/planning/freespace_planner/): calculates trajectory considering the feasibility (e.g. curvature) for the freespace scene. Algorithms are described [here](https://autowarefoundation.github.io/autoware.universe/main/planning/freespace_planning_algorithms/).
- _scenario_selector_ : chooses a trajectory according to the current scenario.
- [_external_velocity_limit_selector_](https://autowarefoundation.github.io/autoware.universe/main/planning/external_velocity_limit_selector/): takes an appropriate velocity limit from multiple candidates.
- [_motion_velocity_smoother_](https://autowarefoundation.github.io/autoware.universe/main/planning/motion_velocity_smoother/): calculates final velocity considering velocity, acceleration, and jerk constraints.

### Important Parameters

| Package                      | Parameter                                                     | Type   | Description                                                                                                        |
| ---------------------------- | ------------------------------------------------------------- | ------ | ------------------------------------------------------------------------------------------------------------------ |
| `obstacle_stop_planner`      | `stop_planner.stop_position.max_longitudinal_margin`          | double | distance between the ego and the front vehicle when stopping (when `cruise_planner_type:=obstacle_stop_planner`)   |
| `obstacle_cruise_planner`    | `common.safe_distance_margin`                                 | double | distance between the ego and the front vehicle when stopping (when `cruise_planner_type:=obstacle_cruise_planner`) |
| `behavior_path_planner`      | `avoidance.avoidance.lateral.lateral_collision_margin`        | double | minimum lateral margin to obstacle on avoidance                                                                    |
| `behavior_path_planner`      | `avoidance.avoidance.lateral.lateral_collision_safety_buffer` | double | additional lateral margin to obstacle if possible on avoidance                                                     |
| `obstacle_avoidance_planner` | `option.enable_outside_drivable_area_stop`                    | bool   | If set true, a stop point will be inserted before the path footprint is outside the drivable area.                 |

### Notation

#### [1] self-crossing road and overlapped

To support the self-crossing road and overlapped road in the opposite direction, each planning module has to meet the [specifications](https://autowarefoundation.github.io/autoware.universe/main/common/motion_utils/)

Currently, the supported modules are as follows.

- lane_following (in behavior_path_planner)
- detection_area (in behavior_velocity_planner)
- stop_line (in behavior_velocity_planner)
- virtual_traffic_light (in behavior_velocity_planner)
- obstacle_avoidance_planner
- obstacle_stop_planner
- motion_velocity_smoother

#### [2] Size of Path Points

Some functions do not support paths with only one point. Therefore, each modules should generate the path with more than two path points.
