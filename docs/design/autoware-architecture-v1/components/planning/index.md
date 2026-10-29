# Planning component design

The planning component makes the target trajectory (a path and a speed profile) that the vehicle follows. The trajectory must be safe, obey the traffic rules, and complete the mission.

This page has two parts. The first part gives the high-level requirements and design. The second part describes the reference implementation and the features that it supports.

!!! info "Planning in Autoware 2.0"

    Autoware 2.0 uses the [generator-selector architecture](../../../autoware-architecture-v2/index.md). A generator makes candidate trajectories, and a selector picks one. The rule-based scenario planner on this page is the default generator. The `planning_setting` launch argument selects the generator:

    - `rule_based` (default): scenario planning makes the trajectory, as this page describes.
    - `diffusion_planner`: the Diffusion Planner, a learned planner, makes the trajectory. Mission planning and the planning validator still run.

    The [Autoware 2.0 architecture](../../../autoware-architecture-v2/index.md) page gives the details.

## Goals and non-goals

Autoware is an "autonomous driving platform". Users extend the driving functions for their own needs. Autoware follows the [microautonomy architecture](../../../autoware-concepts/index.md) concept: high extensibility, modular functions, and clearly defined interfaces.

For this reason, the planning component does not try to solve every complex driving scenario. Instead, it gives **a planning development platform that you can customize and extend easily**. With this platform, users can solve many complex use cases for their own needs.

### Goals

- **The basic functions are provided, so that you can define a simple ODD.** Before you extend the functions, the planning component must give the basic functions of autonomous driving. These functions include moving, stopping, and turning. They also include lane changes and obstacle avoidance in relatively safe and simple situations.
- **The functions are modular, so that users can extend them.** The system adapts to different operational design domains (ODDs) with extended functions. Modules work like plug-ins. You can build systems for different needs, for example Level 4 or Level 2 driving, public or private roads, large vehicles, or small robots.
  - You can also remove functions for a specific ODD, for example a private road with no obstacles. As a result, the system can use less power or fewer sensors.
- **Human operators can extend the capability with their decisions.** With operator assistance, the system can handle complex and difficult scenarios. This page does not define the type of operator. It can be a person in the vehicle during prototype development, or a remote operator who connects in an emergency during an autonomous driving service.

### Non-goals

Third-party modules can extend the planning component. As a result, these items are not goals of the planning component of Autoware:

- To give all the functions that users need by default.
- To give the complete functions and performance of an autonomous driving system.
- To give a performance that is always better than human drivers, or to guarantee absolute safety.

These non-goals come from the "platform" vision of Autoware. They do not apply to a typical planning component.

## High-level design

This diagram shows the high-level architecture of the planning component. It is an ideal design, and the current implementation is different in some areas. The [reference implementation](#reference-implementation) section describes the implementation.

{{ diagram("image/high-level-planning-diagram.svg", "The planning component has three sub-components: mission planning, planning modules, and validation.") }}

The planning component follows the microautonomy architecture and uses a modular framework. Each planning function is a module, for example lane change, intersection, or crosswalk. You can add or remove modules statically or dynamically for each use case.

The planning component has these sub-components:

- **Mission planning**: calculates a route from the current position to the destination with the map data. It is similar to the route planning of a fleet management system (FMS) or a car navigation system.
- **Planning modules**: plan the behavior of the vehicle for the mission, for example the target trajectory and the turn signals. There are two types of modules:
  - **Behavior**: calculates a safe route that obeys the traffic rules. It decides on lane changes, on the entry into intersections, and on stops at stop lines.
  - **Motion**: works with the behavior modules to calculate the trajectory of the vehicle. It considers the vehicle motion and the ride comfort. It includes lateral planning (the path shape) and longitudinal planning (the speed).
- **Validation**: makes sure that the planned trajectory is safe and correct, and responds to emergencies. If a planned trajectory is not correct, it starts an emergency response or makes an alternative path.

### Highlights

<div class="aw-features" markdown>

- :fa-cl-s fa-puzzle-piece: **Modular functions**

  Route planning, lane changes, intersection handling, and other planning functions are modules with standard interfaces. You can add or change a module easily. To enable or disable a module, see [how to enable or disable a planning module](https://autowarefoundation.github.io/autoware_universe/main/planning/#how-to-enable-or-disable-planning-module).

- :fa-cl-s fa-route: **Separate mission planning**

  Mission planning can replace the functions of an existing service, for example an FMS. With the interfaces of the high-level design, you can connect third-party services easily.

- :fa-cl-s fa-shield-halved: **Separate validation**

  The planning component is extensible, so it is difficult to keep the same safety level for all functions. For this reason, validation is independent of the planning modules. It keeps a minimum safety level, even when you change the planning modules.

- :fa-cl-s fa-user-gear: **Interface for the HMI**

  The human machine interface (HMI) lets the planning component work with human operators, in the vehicle or remote.

</div>

### Trade-offs of a separate planning component

Autoware separates planning, perception, localization, and control into different components. This separation makes it easier to work with third-party modules. But there is a trade-off between performance and extensibility. For example, perception can process objects that planning does not need, because perception is separate from planning. Also, when planning is separate from control, it is difficult for planning to consider the vehicle dynamics. To decrease these problems, the interfaces can carry more information, or the components can do more computation.

## Customize features

The planning component can work with external modules. This diagram shows three ways to add external functions.

{{ diagram("image/how-to-add-new-modules.svg", "Add or replace modules, replace a sub-component, or replace the whole planning component.") }}

### 1. Add or replace modules

You can add new modules to the planning component, or replace existing modules. This is the usual way to extend the functions. For example, you can add a function that your ODD needs, or simplify an existing function.

To do this, the module interfaces must be well organized. The current implementation does not have a fully modular system yet, so this approach has limits. For more information, see [Customize features in the current implementation](#customize-features-in-the-current-implementation) and [how to enable or disable a planning module](https://autowarefoundation.github.io/autoware_universe/main/planning/#how-to-enable-or-disable-planning-module).

### 2. Replace a sub-component

You can also work at the sub-component level. For example, you can replace mission planning with an existing FMS service. Or you can use a third-party trajectory generator together with the existing validation.

If you follow the [internal interfaces of the planning component](#internal-interfaces), you can extend planning at this level. Close coordination with the existing planning modules can be limited, but you can connect some planning functions with external modules.

### 3. Replace the whole planning component

Companies and research groups that develop their own planning system can use it with the perception and control components of Autoware. To do this, replace the whole planning component and follow the stable interfaces between the components. In this case, you cannot coordinate directly with the existing planning modules.

## Component interface

This section describes the inputs and outputs of the planning component and of its sub-components. The topics are the defaults of the reference implementation. The [planning component interface](../../interfaces/components/planning.md) page gives more details.

### Inputs

| From                          | Data                 | Description                                                                                                                                                      | Topic                                                                     |
| :---------------------------- | :------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------------------------------------------------------------------ |
| Map                           | Vector map           | All static information about the environment: the lane connections for route planning, the lane shapes for the reference path, and the traffic rule information. | `/map/vector_map`                                                         |
| Perception                    | Detected objects     | Objects that the map does not know, for example pedestrians and other vehicles. Planning plans maneuvers that prevent collisions with these objects.             | `/perception/object_recognition/objects`                                  |
| Perception                    | Detected obstacles   | The positions of obstacles. This data is more primitive than the detected objects. Planning uses it for emergency stops and other safety functions.              | `/perception/obstacle_segmentation/pointcloud`                            |
| Perception                    | Occupancy grid map   | Where pedestrians, vehicles, and occluded areas are.                                                                                                             | `/perception/occupancy_grid_map/map`                                      |
| Perception                    | Traffic light states | The current state of each traffic light. Planning finds the traffic lights on the path and decides whether to stop.                                              | `/perception/traffic_light_recognition/traffic_signals`                   |
| Localization                  | Vehicle motion       | The position, velocity, and acceleration of the ego vehicle.                                                                                                     | `/localization/kinematic_state`, `/localization/acceleration`             |
| System                        | Operation mode       | Whether the vehicle drives in autonomous mode.                                                                                                                   | `/system/operation_mode/state`                                            |
| Human machine interface (HMI) | Operator approvals   | A human operator starts or approves a maneuver, for example a lane change or an entry into an intersection. This is the request to cooperate (RTC) interface.    | Services under `/planning/cooperate_commands/`                            |
| AD API                        | Destination (goal)   | The final position that the vehicle must reach.                                                                                                                  | Services `/planning/set_waypoint_route` and `/planning/set_lanelet_route` |
| AD API                        | Checkpoints          | Positions on the way to the destination. Route planning uses them.                                                                                               | Part of the route request                                                 |
| AD API                        | Velocity limit       | The maximum speed of the vehicle.                                                                                                                                | `/planning/scenario_planning/max_velocity_default`                        |

### Outputs

| To                            | Data                           | Description                                                                                                                                                         | Topic                                                          |
| :---------------------------- | :----------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------ | :------------------------------------------------------------- |
| Control                       | Trajectory                     | A smooth sequence of poses, twists, and accelerations that the control component must follow. The trajectory is usually 10 seconds long with a 0.1 second interval. | `/planning/trajectory`                                         |
| Control                       | Turn signals and hazard lights | The commands for the turn indicators (right, left) and the hazard lights, from the planned maneuvers.                                                               | `/planning/turn_indicators_cmd`, `/planning/hazard_lights_cmd` |
| System                        | Diagnostics                    | The state of the planning component: whether the processing runs correctly, and whether the plan is safe.                                                           | `/diagnostics`                                                 |
| Human machine interface (HMI) | Maneuver availability          | The state of the maneuvers that the operator can start or must approve, for example a lane change or an entry into an intersection.                                 | `/planning/cooperate_status/<module>`                          |
| Human machine interface (HMI) | Candidate path                 | The path that the vehicle will drive after the operator approves the maneuver.                                                                                      | `/planning/path_candidate/<module>`                            |
| AD API                        | Planning factors               | The reasons for the current planning behavior, for example the position of an object to avoid or of an obstacle that causes a stop.                                 | `/planning/planning_factors/<module>`                          |

### Internal interfaces

| From → to                            | Data          | Description                                                                                                                                                                                                                                        |
| :----------------------------------- | :------------ | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Mission planning → scenario planning | Route         | The way from the start to the destination, as lane IDs of the map. The route does not select one specific lane, so it can contain more than one lane. The topic is `/planning/mission_planning/route` (`autoware_planning_msgs/msg/LaneletRoute`). |
| Behavior planning → motion planning  | Path          | A rough position and velocity for the vehicle. The path points are usually about 1 m apart. Other intervals are possible, but they can affect the precision or performance of planning. The message is `autoware_planning_msgs/msg/Path`.          |
| Behavior planning → motion planning  | Drivable area | The area where the vehicle can drive, for example inside the lanes or a physically drivable area. Motion planning calculates the final trajectory inside this area. The left and right bounds of the path message carry it.                        |
| Scenario planning → validation       | Trajectory    | The target positions, velocities, and accelerations that the control component follows. The trajectory points are about 0.1 seconds apart. The message is `autoware_planning_msgs/msg/Trajectory`.                                                 |
| Validation → control                 | Trajectory    | The same trajectory after the safety checks of the planning validator.                                                                                                                                                                             |

## Supported features

This section lists the features of the reference implementation. Each feature names the module that implements it and the map data or other input that it needs. The [reference implementation](#reference-implementation) section lists all modules and the launch arguments that enable them.

The drawings use these conventions:

- The blue vehicle is the ego vehicle. Traffic keeps to the left.
- The green line with an arrow is the planned path.
- A cyan area is a section where the ego vehicle slows down.
- A red area is a section that the ego vehicle must not enter yet. A red line across the lane is a stop point.
- A red vehicle, person, or cone is an object that causes the planning behavior. A grey vehicle is another vehicle.

### Route and path planning

#### Route planning

Plan a route from the position of the ego vehicle to the destination.

{{ diagram("image/features-route-planning.svg", "Route planning on the road network, through the checkpoints, to the goal.") }}

- Module: [Mission Planner](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_mission_planner_universe/) (`autoware_mission_planner_universe`). Autoware Core also has the [`autoware_mission_planner`](https://autowarefoundation.github.io/autoware_core/main/planning/autoware_mission_planner/) package.
- Needs: Lanelet map (driving lanelets).

#### Path planning from the route

Plan the path that the vehicle follows on the route (lane following).

{{ diagram("image/features-lane-follow.svg", "Lane following: the path follows the lane center.") }}

- Module: [Behavior Path Planner](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner/). As an alternative, the [Path Generator](https://autowarefoundation.github.io/autoware_core/main/planning/autoware_path_generator/) of Autoware Core makes a path without the behavior path modules (`behavior_path_planner_type:=path_generator`).
- Needs: Lanelet map (driving lanelets).

#### Lane change

Plan a path that changes lanes to reach the destination.

{{ diagram("image/features-lane-change.svg", "Lane change to the lane on the route.") }}

- Module: [Lane Change](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/). Launch arguments: `launch_lane_change_right_module`, `launch_lane_change_left_module`.
- Needs: Lanelet map (driving lanelets).
- Demonstration: [lane change video](https://youtu.be/0jRDGQ84cD4).

#### Obstacle avoidance

Plan a path that steers around obstacles.

{{ diagram("image/features-avoidance.svg", "Avoidance of a parked vehicle.") }}

- Modules: [Static Obstacle Avoidance](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/) (launch argument `launch_static_obstacle_avoidance`) and [Path Optimizer](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/) (launch argument `motion_path_planner_type:=path_optimizer`, the default).
- Needs: Object information.
- Demonstration: [avoidance video](https://youtu.be/A_V9yvfKZ4E).

#### Path smoothing

Plan a path for smooth steering.

{{ diagram("image/features-path-smoothing.svg", "A smooth path into the next lane where a lane ends.") }}

- Modules: [Elastic Band Smoother](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_smoother/) (launch argument `motion_path_smoother_type:=elastic_band`, the default) and [Path Optimizer](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/).
- Needs: Lanelet map (driving lanelets).
- Demonstration: [path smoothing video](https://youtu.be/RhyAF26Ppzs).

#### Narrow space driving

Plan a path inside the drivable area. If the vehicle cannot stay inside the drivable area, stop the vehicle before it leaves the area.

{{ diagram("image/features-narrow-space-driving.svg", "The footprint of the vehicle stays inside a narrow drivable area.") }}

- Module: [Path Optimizer](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/). The parameter `enable_outside_drivable_area_stop` enables the stop.
- Needs: Lanelet map (high-precision lane boundaries).
- Demonstration: [narrow space video](https://youtu.be/URzcLO2E1vY).

#### Pull over

Plan a path that pulls over to park on the road shoulder.

{{ diagram("image/features-pull-over.svg", "Pull over to a goal on the road shoulder.") }}

- Module: [Goal Planner](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/). Launch argument: `launch_goal_planner_module`.
- Needs: Lanelet map (shoulder lane).
- Demonstrations: [simple pull over video](https://youtu.be/r3-kAmTb4hc), [arc forward pull over video](https://youtu.be/ornbzkWxRWU), and [arc backward pull over video](https://youtu.be/if-0tG3AkLo).

#### Pull out

Plan a path that pulls out from the road shoulder to start driving.

{{ diagram("image/features-pull-out.svg", "Pull out from the road shoulder to the lane.") }}

- Module: [Start Planner](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/). Launch argument: `launch_start_planner_module`.
- Needs: Lanelet map (shoulder lane).
- Demonstrations: [simple pull out video](https://youtu.be/xOjnPqoHup4) and [backward pull out video](https://youtu.be/iGieijPcPcQ).

#### Path shift

Shift the path sideways because of an external instruction.

{{ diagram("image/features-side-shift.svg", "The path shifts sideways by the requested offset.") }}

- Module: [Side Shift](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_side_shift_module/). Launch argument: `launch_side_shift_module`.
- Needs: Nothing.

### Velocity planning for obstacles and road shape

#### Obstacle stop

Plan the velocity to stop for an obstacle on the path.

{{ diagram("image/features-obstacle-stop.svg", "Stop before an obstacle on the path.") }}

- Module: [Obstacle Stop](https://autowarefoundation.github.io/autoware_core/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/) in the Motion Velocity Planner. Launch argument: `launch_obstacle_stop_module`.
- Needs: Object information.
- Demonstration: [obstacle stop video](https://youtu.be/d8IRW_xArcE).

#### Obstacle deceleration

Plan the velocity to slow down for an obstacle near the path.

{{ diagram("image/features-obstacle-decel.svg", "Slow down near an obstacle next to the path.") }}

- Module: [Obstacle Slow Down](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/) in the Motion Velocity Planner. Launch argument: `launch_obstacle_slow_down_module`.
- Needs: Object information.
- Demonstration: [obstacle deceleration video](https://youtu.be/gvN1otgeaaw).

#### Adaptive cruise control

Plan the velocity to follow the vehicle in front of the ego vehicle.

{{ diagram("image/features-adaptive-cruise.svg", "Follow the vehicle ahead at a safe distance.") }}

- Module: [Obstacle Cruise](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/) in the Motion Velocity Planner. Launch argument: `launch_obstacle_cruise_module`.
- Needs: Object information.

#### Deceleration for cut-in vehicles

Plan the velocity to reduce the risk from a vehicle that cuts in to the ego lane.

{{ diagram("image/features-cut-in.svg", "Slow down for a vehicle that cuts in to the ego lane.") }}

- Module: [Obstacle Cruise](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/) in the Motion Velocity Planner.
- Needs: Object information.

#### Surround check at start

Plan the velocity to keep the vehicle stopped while an obstacle is near the vehicle. It works only when the vehicle is stopped.

{{ diagram("image/features-surround-check.svg", "The stopped vehicle does not start while a pedestrian is near it.") }}

- Module: [Surround Obstacle Checker](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_surround_obstacle_checker/). Launch argument: `launch_surround_obstacle_checker` (`false` in the default preset).
- Needs: Object information.
- Demonstration: [surround obstacle checker video](https://youtu.be/bbGgtXN3lC4).

#### Run-out check

Plan the velocity to slow down or stop for objects that can run out into the path.

{{ diagram("image/features-run-out.svg", "Stop for a pedestrian who moves toward the path.") }}

- Module: [Run Out](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/) in the Motion Velocity Planner. Launch argument: `launch_run_out_module`.
- Needs: Object information.
- Demonstration: [run out video](https://youtu.be/9IDggldT2t0).

#### Collision check when the vehicle leaves its lane

Plan the velocity to prevent a collision with vehicles in another lane when the ego vehicle goes out of its own lane, for example when a large vehicle takes a narrow turn.

- Module: [Out of Lane](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/) in the Motion Velocity Planner. Launch argument: `launch_out_of_lane_module`.
- Needs: Object information and Lanelet map (driving lanes).

#### Curve deceleration

Plan the velocity to slow down in a curve.

{{ diagram("image/features-decel-on-curve.svg", "Slow down before and in a sharp curve.") }}

- Module: [Velocity Smoother](https://autowarefoundation.github.io/autoware_core/main/planning/autoware_velocity_smoother/). It limits the lateral acceleration with the parameter `lateral_acceleration_limits`.
- Needs: Nothing.

#### Curve deceleration for obstacles

Plan the velocity to slow down in a curve when an obstacle near the path can cause a collision.

{{ diagram("image/features-decel-on-curve-obstacles.svg", "Slow down more in a curve with an obstacle at its edge.") }}

- Module: [Obstacle Velocity Limiter](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/) in the Motion Velocity Planner. Launch argument: `launch_obstacle_velocity_limiter_module`.
- Needs: Object information and Lanelet map (static obstacles).
- Demonstration: [obstacle velocity limiter video](https://youtu.be/I-oFgG6kIAs).

### Traffic rules

#### Crosswalk

Plan the velocity to stop or slow down for pedestrians who approach or walk on a crosswalk.

{{ diagram("image/features-crosswalk.svg", "Stop before a crosswalk when a pedestrian walks onto it.") }}

- Module: [Crosswalk](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/) in the Behavior Velocity Planner. Launch argument: `launch_crosswalk_module`.
- Needs: Object information and Lanelet map (pedestrian crossing).
- Demonstration: [crosswalk video](https://youtu.be/tUvthyIL2W8).

#### Stop line

Plan the velocity to stop at a stop line.

{{ diagram("image/features-stop-line.svg", "Stop at the stop line of the map.") }}

- Module: [Stop Line](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/) in the Behavior Velocity Planner (Autoware Core). Launch argument: `launch_stop_line_module`.
- Needs: Lanelet map (stop line).
- Demonstration: [stop line video](https://youtu.be/eej9jYt-GSE).

#### Traffic light

Plan the velocity at an intersection from the traffic light signal.

{{ diagram("image/features-traffic-light.svg", "Stop at the stop line when the traffic light is red.") }}

- Module: [Traffic Light](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/) in the Behavior Velocity Planner. Launch argument: `launch_traffic_light_module`.
- Needs: Traffic light state information.
- Demonstration: [traffic light video](https://youtu.be/lGA53KljQrM).

#### Intersection: oncoming vehicle check

Plan the velocity for a right or left turn at an intersection, to prevent a collision with oncoming vehicles.

{{ diagram("image/features-intersection.svg", "Wait before a turn across the oncoming lane.") }}

- Module: [Intersection](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/) in the Behavior Velocity Planner. Launch argument: `launch_intersection_module`.
- Needs: Object information and Lanelet map (intersection lanes and yield lanes).
- Demonstration: [intersection video](https://youtu.be/SGD07Hqg4Hk).

#### Intersection: blind spot check

Plan the velocity for a right or left turn at an intersection, to prevent a collision with vehicles, motorcycles, or bicycles that come from behind in the blind spot.

{{ diagram("image/features-blind-spot.svg", "Wait before a left turn when a bicycle comes from behind.") }}

- Module: [Blind Spot](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/) in the Behavior Velocity Planner. Launch argument: `launch_blind_spot_module`.
- Needs: Object information and Lanelet map (intersection lanes).
- Demonstration: [blind spot video](https://youtu.be/oaTCJRafDGA).

#### Intersection: occlusion check

Plan the velocity for a right or left turn at an intersection, to reduce the risk from vehicles that can come out of an occluded area.

{{ diagram("image/features-intersection-occlusion.svg", "Wait before a turn when a large vehicle hides part of the oncoming lane.") }}

- Module: [Intersection](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/) in the Behavior Velocity Planner.
- Needs: Object information and Lanelet map (intersection lanes).
- Demonstration: [intersection occlusion video](https://youtu.be/bAHXMB7kbFc).

#### Intersection: traffic jam detection

Plan the velocity so that the vehicle does not enter an intersection when a vehicle is stopped after it because of a traffic jam.

{{ diagram("image/features-intersection-traffic-jam.svg", "Do not enter the intersection when its exit is blocked.") }}

- Module: [Intersection](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/) in the Behavior Velocity Planner.
- Needs: Object information and Lanelet map (intersection lanes).
- Demonstration: [intersection traffic jam video](https://youtu.be/negK4VbrC5o).

#### Merge from a private area to a public road

Plan the velocity to enter a public road from a private area, to prevent a collision with pedestrians or other vehicles.

- Module: the merge from private road function of the [Intersection](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/) module. Launch argument: `launch_merge_from_private_module` (`false` in the default preset).
- Needs: Object information and Lanelet map (private and public lanes).

#### No stopping area

Plan the velocity so that the vehicle does not stop in an area where stopping is not permitted, for example in front of the exit of a fire station.

{{ diagram("image/features-no-stopping-area.svg", "Stop before a no stopping area when a vehicle is stopped after it.") }}

- Module: [No Stopping Area](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/) in the Behavior Velocity Planner. Launch argument: `launch_no_stopping_area_module`.
- Needs: Lanelet map (no stopping area).

#### Detection area

Plan the velocity to stop at the stop line of a detection area when an object is in the area.

{{ diagram("image/features-detection-area.svg", "Stop at the stop line when objects are in the detection area.") }}

- Module: [Detection Area](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/) in the Behavior Velocity Planner. Launch argument: `launch_detection_area_module`.
- Needs: Lanelet map (detection area).
- Demonstration: [detection area video](https://youtu.be/YzXF4U69lJs).

#### Speed bump

Plan the velocity to slow down for a speed bump.

{{ diagram("image/features-speed-bump.svg", "Slow down before and on a speed bump.") }}

- Module: [Speed Bump](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/) in the Behavior Velocity Planner. Launch argument: `launch_speed_bump_module` (`false` in the default preset).
- Needs: Lanelet map (speed bump).
- Demonstration: [speed bump video](https://youtu.be/FpX3q3YaaCw).

#### No drivable lane

Plan the velocity to stop before a lane that is outside the operational design domain (ODD). If autonomous mode starts in such a lane, stop the vehicle.

{{ diagram("image/features-no-drivable-lane.svg", "Stop before a no drivable lane.") }}

- Module: [No Drivable Lane](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_no_drivable_lane_module/) in the Behavior Velocity Planner. Launch argument: `launch_no_drivable_lane_module` (`false` in the default preset).
- Needs: Lanelet map (no drivable lane).

#### Occlusion spot check

Plan the velocity to slow down for objects that can come out of an occluded area, for example from behind a building or a large vehicle.

{{ diagram("image/features-occlusion-spot.svg", "Slow down where a building hides a side street.") }}

- Module: [Occlusion Spot](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/) in the Behavior Velocity Planner. Launch argument: `launch_occlusion_spot_module` (`false` in the default preset).
- Needs: Object information and Lanelet map (private and public lanes).
- Demonstration: [occlusion spot video](https://youtu.be/3qs8Ivjh1fs).

### Parking

#### Parking

Plan the path and the velocity to a goal in a parking area.

{{ diagram("image/features-parking.svg", "Drive forward, then reverse into an empty parking space.") }}

- Modules: [Freespace Planner](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_freespace_planner/) and [Costmap Generator](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_costmap_generator/). Launch argument: `launch_parking_module`.
- Needs: Object information and Lanelet map (parking area).
- Demonstration: [free space parking video](https://youtu.be/rAIYmwpNWfA).

### Safety

#### Trajectory validation

Make sure that the planned trajectory is safe. If it is not safe, take an action: for example, change the trajectory, stop sending it, or report the problem to the autonomous driving system.

{{ diagram("image/features-trajectory-validation.svg", "A valid trajectory goes to Control. An invalid trajectory does not.") }}

- Module: [Planning Validator](https://autowarefoundation.github.io/autoware_universe/main/planning/planning_validator/autoware_planning_validator/) with its checker plugins.
- Needs: Nothing.

#### Autonomous emergency braking (AEB)

Do an emergency stop if a collision with an object ahead is predicted. This function is the last safety layer. It must work even when localization or perception fails.

{{ diagram("image/features-aeb.svg", "Emergency braking when the vehicle ahead is too close.") }}

- Module: [Autonomous Emergency Braking](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_autonomous_emergency_braking/) in the control component. Launch argument: `launch_autonomous_emergency_braking` (control preset).
- Needs: Primitive objects (point clouds).

#### Minimum risk maneuver (MRM)

Give the correct minimum risk maneuver when a hazardous event occurs. For example, when a sensor fails, send an instruction for an emergency stop, a comfortable stop, or a pull over to the shoulder, for the severity of the situation.

- Modules: the [MRM Handler](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_mrm_handler/) in the system component selects the MRM. Operators such as the [MRM Emergency Stop Operator](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_mrm_emergency_stop_operator/) and the [MRM Comfortable Stop Operator](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_mrm_comfortable_stop_operator/) do it.
- Needs: The system state.

### Map tools

#### Running lane map generation

Make a lane map from the localization data that you record during manual driving.

- Tool: [bag2lanelet](https://autowarefoundation.github.io/autoware_tools/main/bag2lanelet/) in `autoware_tools`.
- Needs: Nothing.

#### Running lane optimization

Optimize the centerline (the reference path) of the map, so that it is smooth for the vehicle kinematics.

- Tool: [Static Centerline Generator](https://autowarefoundation.github.io/autoware_tools/main/planning/autoware_static_centerline_generator/) in `autoware_tools`.
- Needs: Lanelet map (driving lanes).

## Reference implementation

This diagram shows the reference implementation of the planning component. You can add new modules or extend the functions to support different ODDs.

The implementation does not follow the high-level architecture in some areas, because of implementation difficulties. These areas need an update.

{{ diagram("image/planning-diagram.svg", "The reference implementation of the planning component, with the Diffusion Planner as an optional trajectory source.") }}

The launch files are in [`autoware_planning_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_universe_launch/autoware_planning_launch/launch) in `autoware_launch`. A module preset selects the modules that launch. The default preset is [`default_preset.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/preset/default_preset.yaml) in `autoware_planning_config`. To use another preset, set the `planning_module_preset` launch argument. For more information, see [how to enable or disable a planning module](https://autowarefoundation.github.io/autoware_universe/main/planning/#how-to-enable-or-disable-planning-module).

### Nodes

| Node                                                                                                                                                          | Package                                     | What it does                                                                                                                                                                                                                                                                                                  |
| :------------------------------------------------------------------------------------------------------------------------------------------------------------ | :------------------------------------------ | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| [`mission_planner`](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_mission_planner_universe/)                                  | `autoware_mission_planner_universe`         | Calculates the route from the start to the goal from the map. The `route_selector` node of the same package switches between the normal route and the MRM route.                                                                                                                                              |
| [`behavior_path_planner`](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner/)         | `autoware_behavior_path_planner`            | Calculates the path and the drivable area from the traffic rules. It loads the [behavior path modules](#behavior-path-planner-modules) as plugins.                                                                                                                                                            |
| [`behavior_velocity_planner`](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/) | `autoware_behavior_velocity_planner` (Core) | Calculates the maximum speed from the traffic rules. It loads the [behavior velocity modules](#behavior-velocity-planner-modules) as plugins.                                                                                                                                                                 |
| [`elastic_band_smoother`](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_smoother/)                                       | `autoware_path_smoother`                    | Smooths the path. Launch argument: `motion_path_smoother_type` (`elastic_band` or `none`).                                                                                                                                                                                                                    |
| [`path_optimizer`](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/)                                             | `autoware_path_optimizer`                   | Calculates a kinematically feasible path inside the drivable area. Launch argument: `motion_path_planner_type` (`path_optimizer`, `path_sampler`, or `none`).                                                                                                                                                 |
| [`motion_velocity_planner`](https://autowarefoundation.github.io/autoware_core/main/planning/motion_velocity_planner/autoware_motion_velocity_planner/)       | `autoware_motion_velocity_planner` (Core)   | Adjusts the trajectory velocity for the obstacles around the vehicle. It loads the [motion velocity modules](#motion-velocity-planner-modules) as plugins.                                                                                                                                                    |
| [`surround_obstacle_checker`](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_surround_obstacle_checker/)                       | `autoware_surround_obstacle_checker`        | Keeps the vehicle stopped while obstacles are near it. It works only when the vehicle is stopped. Launch argument: `launch_surround_obstacle_checker`.                                                                                                                                                        |
| [`costmap_generator`](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_costmap_generator/)                                       | `autoware_costmap_generator`                | Makes a costmap for the parking scenario from the objects and the lane information.                                                                                                                                                                                                                           |
| [`freespace_planner`](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_freespace_planner/)                                       | `autoware_freespace_planner`                | Calculates a trajectory in free space, for example a parking lot, and considers the feasibility (for example the curvature). The [freespace planning algorithms](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_freespace_planning_algorithms/) page describes the algorithms. |
| [`scenario_selector`](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_scenario_selector/)                                       | `autoware_scenario_selector`                | Selects the trajectory of the current scenario (lane driving or parking).                                                                                                                                                                                                                                     |
| [`external_velocity_limit_selector`](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_external_velocity_limit_selector/)         | `autoware_external_velocity_limit_selector` | Selects one velocity limit from the limits of the AD API and the planning modules.                                                                                                                                                                                                                            |
| [`velocity_smoother`](https://autowarefoundation.github.io/autoware_core/main/planning/autoware_velocity_smoother/)                                           | `autoware_velocity_smoother` (Core)         | Calculates the final velocity with the velocity, acceleration, and jerk limits. Launch argument: `velocity_smoother_type` (`JerkFiltered` by default).                                                                                                                                                        |
| [`hazard_lights_selector`](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_hazard_lights_selector/)                             | `autoware_hazard_lights_selector`           | Selects the hazard lights command from the behavior path planner and from the MRM operators.                                                                                                                                                                                                                  |
| [`planning_validator`](https://autowarefoundation.github.io/autoware_universe/main/planning/planning_validator/autoware_planning_validator/)                  | `autoware_planning_validator`               | Makes sure that the trajectory is valid before it goes to the control component. It loads the [validator checkers](#planning-validator-checkers) as plugins.                                                                                                                                                  |

Autoware Core also has a minimal planning pipeline in [`autoware_core_planning`](https://github.com/autowarefoundation/autoware_core/tree/main/planning/autoware_core_planning): the mission planner, the path generator, the behavior velocity planner, the motion velocity planner, and the velocity smoother.

### Behavior path planner modules

| Module                                                                                                                                                                             | Launch argument                                                                                       | Default preset |
| :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------------- | :------------- |
| [Static obstacle avoidance](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/)   | `launch_static_obstacle_avoidance`                                                                    | On             |
| [Avoidance by lane change](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/)     | `launch_avoidance_by_lane_change_module`                                                              | On             |
| [Dynamic obstacle avoidance](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/) | `launch_dynamic_obstacle_avoidance`                                                                   | Off            |
| [Lane change](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/)                               | `launch_lane_change_right_module`, `launch_lane_change_left_module`                                   | On             |
| External request lane change (`autoware_behavior_path_external_request_lane_change_module`)                                                                                        | `launch_external_request_lane_change_right_module`, `launch_external_request_lane_change_left_module` | Off            |
| [Goal planner](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/)                             | `launch_goal_planner_module`                                                                          | On             |
| [Start planner](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/)                           | `launch_start_planner_module`                                                                         | On             |
| [Side shift](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_side_shift_module/)                                 | `launch_side_shift_module`                                                                            | On             |
| [Bidirectional traffic](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/)           | `launch_bidirectional_traffic_module`                                                                 | On             |
| [Direction change](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_direction_change_module/)                     | `launch_direction_change_module`                                                                      | Off            |
| [Sampling planner](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/) (experimental)      | `launch_sampling_planner_module`                                                                      | Off            |

Lane following is not a separate module. The behavior path planner itself makes the reference path from the lane centerline.

### Behavior velocity planner modules

| Module                                                                                                                                                                                                  | Launch argument                       | Default preset |
| :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | :------------------------------------ | :------------- |
| [Crosswalk](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/)                                                | `launch_crosswalk_module`             | On             |
| [Walkway](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_walkway_module/)                                                    | `launch_walkway_module`               | On             |
| [Traffic light](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/)                                        | `launch_traffic_light_module`         | On             |
| [Intersection](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)                                          | `launch_intersection_module`          | On             |
| Merge from private road (in the [intersection package](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)) | `launch_merge_from_private_module`    | Off            |
| [Roundabout](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/)                                              | `launch_roundabout_module`            | Off            |
| [Blind spot](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/)                                              | `launch_blind_spot_module`            | On             |
| [Detection area](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/)                                      | `launch_detection_area_module`        | On             |
| [Virtual traffic light](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/)                        | `launch_virtual_traffic_light_module` | On             |
| [No stopping area](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/)                                  | `launch_no_stopping_area_module`      | On             |
| [Stop line](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/) (Core)                                             | `launch_stop_line_module`             | On             |
| [Occlusion spot](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/)                                      | `launch_occlusion_spot_module`        | Off            |
| [Speed bump](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/)                                              | `launch_speed_bump_module`            | Off            |
| [No drivable lane](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_no_drivable_lane_module/)                                  | `launch_no_drivable_lane_module`      | Off            |

### Motion velocity planner modules

| Module                                                                                                                                                                                                      | Launch argument                               | Default preset |
| :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :-------------------------------------------- | :------------- |
| [Obstacle stop](https://autowarefoundation.github.io/autoware_core/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/) (Core)                                             | `launch_obstacle_stop_module`                 | On             |
| [Obstacle slow down](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/)                                      | `launch_obstacle_slow_down_module`            | On             |
| [Obstacle cruise](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/)                                            | `launch_obstacle_cruise_module`               | On             |
| [Dynamic obstacle stop](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/)                                | `launch_dynamic_obstacle_stop_module`         | On             |
| [Out of lane](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/)                                                    | `launch_out_of_lane_module`                   | On             |
| [Obstacle velocity limiter](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/)                        | `launch_obstacle_velocity_limiter_module`     | On             |
| [Run out](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/)                                                            | `launch_run_out_module`                       | On             |
| [Boundary departure prevention](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/) (experimental) | `launch_boundary_departure_prevention_module` | On             |
| [Road user stop](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/)                                              | `launch_road_user_stop_module`                | On             |

### Planning validator checkers

| Checker                                                                                                                                                                               | Launch argument                         | Default preset |
| :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | :-------------------------------------- | :------------- |
| Latency checker (`autoware_planning_validator_latency_checker`)                                                                                                                       | `launch_latency_checker`                | On             |
| [Trajectory checker](https://autowarefoundation.github.io/autoware_universe/main/planning/planning_validator/autoware_planning_validator_trajectory_checker/)                         | `launch_trajectory_checker`             | On             |
| [Intersection collision checker](https://autowarefoundation.github.io/autoware_universe/main/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/) | `launch_intersection_collision_checker` | On             |
| [Rear collision checker](https://autowarefoundation.github.io/autoware_universe/main/planning/planning_validator/autoware_planning_validator_rear_collision_checker/)                 | `launch_rear_collision_checker`         | On             |

## Important information in the current implementation

The current implementation has two important differences from the high-level design: the scenario layer, and the clear separation of behavior and motion. The current performance and implementation difficulties are the reasons for these differences. It is still open whether to add them to the high-level design or to change the implementation.

### The scenario planning layer

Driving in structured lanes and driving in free space, for example a parking lot, need different interfaces. For example, lane driving can use a route of map IDs, but planning in free space cannot. The scenario layer switches the planning sub-components for each scenario (lane driving, parking, and others). This gives a flexible interface design. But it is difficult to use the same modules in different scenarios.

### Separation of behavior and motion

A classic approach divides planning into "behavior", which decides the action, and "motion", which calculates the final movement. But this separation decreases the performance, and the performance decreases more when the separation increases. For example, behavior makes decisions without the result of the motion calculation, so its decisions are usually conservative. If behavior and motion are one part, the motion performance and the decisions depend on each other. Then it is difficult to extend only the decisions, for example for the traffic rules of a region.

This [earlier design discussion](https://github.com/tier4/AutowareArchitectureProposal.proj/blob/main/docs/design/software_architecture/Planning/DesignRationale.md) gives more background.

## Customize features in the current implementation

You can add functions at the module level in the current implementation. But not all functions have a common interface. This section describes how to extend the current implementation at the module level.

{{ diagram("image/reference-implementation-add-new-modules.svg", "Extension points: plugin modules, new ROS nodes, and new scenarios.") }}

### Add a module to a planner node

The [behavior path planner](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner/), the [behavior velocity planner](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/), and the [motion velocity planner](https://autowarefoundation.github.io/autoware_core/main/planning/motion_velocity_planner/autoware_motion_velocity_planner/) load their modules as plugins. The [planning validator](https://autowarefoundation.github.io/autoware_universe/main/planning/planning_validator/autoware_planning_validator/) loads its checkers as plugins too. If you write a module that follows the module interface of the node, the node can load and unload it. The documentation of each package explains how to add a module. For example, the [template module](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_template_module/) is a starting point for a new behavior velocity module.

### Add a ROS node to the planning component

You can also add a function as a ROS node in motion planning. Each node in the current pipeline adds information to the trajectory that the nodes before it calculated. A new node in this pipeline can extend the functions.

### Add or replace a scenario

The current implementation switches between scenarios. With this switch, you can change many modules together. You can also add a new scenario, for example highway driving.

To add a scenario, make it a ROS node and connect it to the `scenario_selector` node. A new scenario can add large new functions without an effect on the other scenarios, for example lane driving. But scenarios coordinate only through the scenario switch. They cannot coordinate with the existing planning modules.
