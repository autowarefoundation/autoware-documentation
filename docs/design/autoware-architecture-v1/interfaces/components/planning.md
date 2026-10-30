# Planning

This page describes the interface of the Planning component. See the [Planning component design](../../components/planning/index.md) for the concepts and the data flow.

The tables show the default setup of [`autoware_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_universe_launch/autoware_planning_launch), where the `planning_setting` launch argument is `rule_based`. With `diffusion_planner`, a learned planner makes the trajectory instead of the scenario planning modules. The outputs of the component stay the same, and the planner also uses the tracked objects (`/perception/object_recognition/tracking/objects`).

## Input

{{ diagram("images/planning-interface-input.svg", "Inputs come from other components.") }}

### From Map

| Name       | Topic             | Type                                                                                                                                         | Description                                            |
| ---------- | ----------------- | -------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------ |
| Vector map | `/map/vector_map` | [autoware_map_msgs/msg/LaneletMapBin](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/msg/LaneletMapBin.msg) | Map of the environment where the planning takes place. |

### From Localization

| Name                    | Topic                           | Type                                                                                                                                                    | Description                                        |
| ----------------------- | ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| Vehicle kinematic state | `/localization/kinematic_state` | [nav_msgs/msg/Odometry](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg)                                               | Current position, orientation and velocity of ego. |
| Vehicle acceleration    | `/localization/acceleration`    | [geometry_msgs/msg/AccelWithCovarianceStamped](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/AccelWithCovarianceStamped.msg) | Current acceleration of ego.                       |

The acceleration is a separate topic today. The [autoware_localization_msgs/msg/KinematicState](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_localization_msgs/msg/KinematicState.msg) message can hold the pose, the twist, and the acceleration together, but no node in the default setup publishes it. See [Localization](localization.md#outputs).

### From Perception

| Name               | Topic                                                   | Type                                                                                                                                                                         | Description                                                                                                                                                                                                                     |
| ------------------ | ------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Objects            | `/perception/object_recognition/objects`                | [autoware_perception_msgs/msg/PredictedObjects](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/PredictedObjects.msg)             | Set of perceived objects around ego that need to be avoided or followed when planning a trajectory. This contains semantic information such as the object class (for example, vehicle or pedestrian) or a shape of the objects. |
| Obstacles          | `/perception/obstacle_segmentation/pointcloud`          | [sensor_msgs/msg/PointCloud2](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg)                                                        | Set of perceived obstacles around ego that need to be avoided or followed when planning a trajectory. This only contains a primitive information of the obstacle. No shape nor velocity information.                            |
| Occupancy grid map | `/perception/occupancy_grid_map/map`                    | [nav_msgs/msg/OccupancyGrid](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg)                                                          | Contains the presence of obstacles and blind spot information (represented as UNKNOWN).                                                                                                                                         |
| Traffic signal     | `/perception/traffic_light_recognition/traffic_signals` | [autoware_perception_msgs/msg/TrafficLightGroupArray](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/TrafficLightGroupArray.msg) | Contains the traffic signal information such as a color (green, amber, red) and an arrow (right, left, straight).                                                                                                               |

The type of the `Obstacles` information is a sensor message type (`PointCloud2`) today.

### From API

| Name                   | Topic or service                                                             | Type                                                                                                                                                                                  | Description                                                                                                                     |
| ---------------------- | ---------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| Max velocity           | `/planning/scenario_planning/max_velocity_default`                           | [autoware_internal_planning_msgs/msg/VelocityLimit](https://github.com/autowarefoundation/autoware_internal_msgs/blob/main/autoware_internal_planning_msgs/msg/VelocityLimit.msg)     | The maximum velocity that the operator sets.                                                                                    |
| Operation mode         | `/system/operation_mode/state`                                               | [autoware_adapi_v1_msgs/msg/OperationModeState](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/operation_mode/msg/OperationModeState.msg) | The current operation mode, for example autonomous or manual.                                                                   |
| Set lanelet route      | `/planning/set_lanelet_route` (service)                                      | [autoware_planning_msgs/srv/SetLaneletRoute](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/srv/SetLaneletRoute.srv)                            | Sets a route that is a list of lanelet segments. With `allow_modification`, it also changes the route while the vehicle drives. |
| Set waypoint route     | `/planning/set_waypoint_route` (service)                                     | [autoware_planning_msgs/srv/SetWaypointRoute](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/srv/SetWaypointRoute.srv)                          | Sets a route from a goal pose and waypoints. With `allow_modification`, it also changes the route while the vehicle drives.     |
| Clear route            | `/planning/clear_route` (service)                                            | [autoware_planning_msgs/srv/ClearRoute](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/srv/ClearRoute.srv)                                      | Clears the route.                                                                                                               |
| MRM set waypoint route | `/planning/mission_planning/route_selector/mrm/set_waypoint_route` (service) | [autoware_planning_msgs/srv/SetWaypointRoute](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/srv/SetWaypointRoute.srv)                          | Sets a route for a minimal risk maneuver (MRM). `set_lanelet_route` is also available under the same namespace.                 |
| MRM clear route        | `/planning/mission_planning/route_selector/mrm/clear_route` (service)        | [autoware_planning_msgs/srv/ClearRoute](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/srv/ClearRoute.srv)                                      | Clears the MRM route. The normal route then becomes active again.                                                               |

The AD API routing services (`/api/routing/set_route`, `/api/routing/set_route_points`, `/api/routing/change_route`, `/api/routing/change_route_points`, and `/api/routing/clear_route`) call the three route services above. The route selector of `autoware_mission_planner_universe` picks between the normal route and the MRM route.

## Output

{{ diagram("images/planning-interface-output.svg", "Outputs go to other components.") }}

### To Control

| Name           | Topic                           | Type                                                                                                                                                                 | Description                                                                                |
| -------------- | ------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| Trajectory     | `/planning/trajectory`          | [autoware_planning_msgs/msg/Trajectory](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/Trajectory.msg)                     | A sequence of space and velocity and acceleration points to be followed by the controller. |
| Turn indicator | `/planning/turn_indicators_cmd` | [autoware_vehicle_msgs/msg/TurnIndicatorsCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/TurnIndicatorsCommand.msg) | Turn indicator signal to be followed by the vehicle.                                       |
| Hazard light   | `/planning/hazard_lights_cmd`   | [autoware_vehicle_msgs/msg/HazardLightsCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/HazardLightsCommand.msg)     | Hazard light signal to be followed by the vehicle.                                         |

### To System

| Name        | Topic          | Type                                                                                                                                  | Description                                                                   |
| ----------- | -------------- | ------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------- |
| Diagnostics | `/diagnostics` | [diagnostic_msgs/msg/DiagnosticArray](https://github.com/ros2/common_interfaces/blob/rolling/diagnostic_msgs/msg/DiagnosticArray.msg) | Diagnostic status of the Planning component reported to the System component. |

### To API

| Name            | Topic                          | Type                                                                                                                                                                                          | Description                                                                                                                                                                          |
| --------------- | ------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Route           | `/planning/route`              | [autoware_planning_msgs/msg/LaneletRoute](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/LaneletRoute.msg)                                          | The current route.                                                                                                                                                                   |
| Route state     | `/planning/route_state`        | [autoware_planning_msgs/msg/RouteState](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/RouteState.msg)                                              | The state of the route, for example `UNSET`, `SET`, `ARRIVED`, or `REROUTING`.                                                                                                       |
| Path candidate  | `/planning/path_candidate/*`   | [autoware_planning_msgs/msg/Path](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/Path.msg)                                                          | The path that a module of the behavior path planner is about to take. Users can interrupt the operation based on the path candidate information. There is one topic for each module. |
| Planning factor | `/planning/planning_factors/*` | [autoware_internal_planning_msgs/msg/PlanningFactorArray](https://github.com/autowarefoundation/autoware_internal_msgs/blob/main/autoware_internal_planning_msgs/msg/PlanningFactorArray.msg) | Why a module stops, slows down, shifts, or turns the vehicle. There is one topic for each module. The AD API converts them into velocity factors and steering factors.               |

## Planning internal interface

This section explains the communication between the different planning modules shown in the [Planning component design](../../components/planning/index.md#high-level-design).

{{ diagram("images/planning-interface-internal.svg", "Internal interfaces connect the modules inside the Planning component.") }}

### From Mission Planning to Scenario Planning

| Name  | Topic                              | Type                                                                                                                                                 | Description                                                                          |
| ----- | ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| Route | `/planning/mission_planning/route` | [autoware_planning_msgs/msg/LaneletRoute](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/LaneletRoute.msg) | A sequence of lane IDs on a Lanelet map, from the starting point to the destination. |

### From Behavior Planning to Motion Planning

| Name | Topic                                                             | Type                                                                                                                                 | Description                                                                                                                                                                                                                                                                                                       |
| ---- | ----------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Path | `/planning/scenario_planning/lane_driving/behavior_planning/path` | [autoware_planning_msgs/msg/Path](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/Path.msg) | A sequence of approximate vehicle positions for driving, along with information on the maximum speed and the drivable areas. Modules receiving this message are expected to make changes to the path within the constraints of the drivable areas and the maximum speed, generating the desired final trajectory. |

### From Scenario Planning to Validation

| Name       | Topic                                                      | Type                                                                                                                                             | Description                                                                                                                                                           |
| ---------- | ---------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Trajectory | `/planning/scenario_planning/velocity_smoother/trajectory` | [autoware_planning_msgs/msg/Trajectory](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/Trajectory.msg) | A sequence of precise vehicle positions, speeds, and accelerations required for driving. The planning validator checks it and publishes it as `/planning/trajectory`. |

With the `diffusion_planner` setting, the validator takes `/planning/trajectory_generator/trajectory` instead.
