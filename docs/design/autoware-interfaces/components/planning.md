# Planning

This page provides specific specifications about the Interface of the Planning Component. Please refer to the [planning architecture design document](../../autoware-architecture/planning/index.md) for high-level concepts and data flow.

**TODO: The detailed definitions (meanings of elements included in each topic) are not described yet, need to be updated.**

## Input

![planning-input](images/planning-interface-input.drawio.svg)

### From Map Component

| Name       | Topic             | Type                                                                                                                                                 | Description                                            |
| ---------- | ----------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------ |
| Vector Map | `/map/vector_map` | [autoware_auto_mapping_msgs/msg/HADMapBin](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_mapping_msgs/msg/HADMapBin.idl) | Map of the environment where the planning takes place. |

### From Localization Component

| Name                    | Topic                           | Type                                                                                                                                      | Description                                        |
| ----------------------- | ------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| Vehicle Kinematic State | `/localization/kinematic_state` | [nav_msgs/msg/Odometry](https://docs.ros.org/en/latest/api/nav_msgs/html/msg/Odometry.html)                                               | Current position, orientation and velocity of ego. |
| Vehicle Acceleration    | `/localization/acceleration`    | [geometry_msgs/msg/AccelWithCovarianceStamped](https://docs.ros.org/en/latest/api/geometry_msgs/html/msg/AccelWithCovarianceStamped.html) | Current acceleration of ego.                       |

**TODO**: acceleration information should be merged into the kinematic state.

### From Perception Component

| Name               | Topic                                                   | Type                                                                                                                                                                         | Description                                                                                                                                                                                                               |
| ------------------ | ------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Objects            | `/perception/object_recognition/objects`                | [autoware_auto_perception_msgs/msg/PredictedObjects](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_perception_msgs/msg/PredictedObjects.idl)     | Set of perceived objects around ego that need to be avoided or followed when planning a trajectory. This contains semantics information such as a object class (e.g. vehicle, pedestrian, etc) or a shape of the objects. |
| Obstacles          | `/perception/obstacle_segmentation/pointcloud`          | [sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/latest/api/sensor_msgs/html/msg/PointCloud2.html)                                                                      | Set of perceived obstacles around ego that need to be avoided or followed when planning a trajectory. This only contains a primitive information of the obstacle. No shape nor velocity information.                      |
| Occupancy Grid Map | `/perception/occupancy_grid_map/map`                    | [nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/latest/api/nav_msgs/html/msg/OccupancyGrid.html)                                                                        | Contains the presence of obstacles and blind spot information (represented as UNKNOWN).                                                                                                                                   |
| Traffic Signal     | `/perception/traffic_light_recognition/traffic_signals` | [autoware_auto_perception_msgs/msg/TrafficSignalArray](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_perception_msgs/msg/TrafficSignalArray.idl) | Contains the traffic signal information such as a color (green, yellow, read) and an arrow (right, left, straight).                                                                                                       |

**TODO**: The type of the `Obstacles` information should not depend on the specific sensor message type (now `PointCloud`). It needs to be fixed.

### From API

| Name                 | Topic                                                            | Type                                                                                                                                                                                  | Description                                                           |
| -------------------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------- |
| Max Velocity         | `/planning/scenario_planning/max_velocity_default`               | [autoware_adapi_v1_msgs/srv/SetRoutePoints](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/routing/srv/SetRoutePoints.srv)                | Indicate the maximum value of the vehicle speed plan                  |
| Operation Mode       | `/system/operation_mode/state`                                   | [autoware_adapi_v1_msgs/msg/OperationModeState](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/operation_mode/msg/OperationModeState.msg) | Indicates the current operation mode (automatic/manual, etc.).        |
| Route Set            | `/planning/mission_planning/set_route`                           | [autoware_adapi_v1_msgs/srv/SetRoute](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/routing/srv/SetRoute.srv)                            | Indicates to set the route when the vehicle is stopped.               |
| Route Points Set     | `/planning/mission_planning/set_route_points`                    | [autoware_adapi_v1_msgs/srv/SetRoutePoints](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/routing/srv/SetRoutePoints.srv)                | Indicates to set the route with points when the vehicle is stopped.   |
| Route Change         | `/planning/mission_planning/change_route`                        | [autoware_adapi_v1_msgs/srv/SetRoute](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/routing/srv/SetRoute.srv)                            | Indicates to change the route when the vehicle is moving.             |
| Route Points Change  | `/planning/mission_planning/change_route_points`                 | [autoware_adapi_v1_msgs/srv/SetRoutePoints](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/routing/srv/SetRoutePoints.srv)                | Indicates to change the route with points when the vehicle is moving. |
| Route Clear          | `/planning/mission_planning/clear_route`                         | [autoware_adapi_v1_msgs/srv/ClearRoute](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/routing/srv/ClearRoute.srv)                        | Indicates to clear the route information.                             |
| MRM Route Set Points | `/planning/mission_planning/mission_planner/srv/set_mrm_route`   | [autoware_adapi_v1_msgs/srv/SetRoutePoints](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/routing/srv/SetRoutePoints.srv)                | Indicates to set the emergency route.                                 |
| MRM Route Clear      | `/planning/mission_planning/mission_planner/srv/clear_mrm_route` | [autoware_adapi_v1_msgs/srv/SetRoutePoints](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/routing/srv/SetRoutePoints.srv)                | Indicates to clear the emergency route.                               |

## Output

![planning-output](images/planning-interface-output.drawio.svg)

### To Control

| Name           | Topic                           | Type                                                                                                                                                                         | Description                                                                                |
| -------------- | ------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| Trajectory     | `/planning/trajectory`          | [autoware_auto_planning_msgs/msg/Trajectory](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_planning_msgs/msg/Trajectory.idl)                     | A sequence of space and velocity and acceleration points to be followed by the controller. |
| Turn Indicator | `/planning/turn_indicators_cmd` | [autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand.idl) | Turn indicator signal to be followed by the vehicle.                                       |
| Hazard Light   | `/planning/hazard_lights_cmd`   | [autoware_auto_vehicle_msgs/msg/HazardLightsCommand](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/HazardLightsCommand.idl)     | Hazard light signal to be followed by the vehicle.                                         |

### To System

| Name        | Topic                         | Type                                                                                                                   | Description                                                                   |
| ----------- | ----------------------------- | ---------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------- |
| Diagnostics | `/planning/hazard_lights_cmd` | [diagnostic_msgs/msg/DiagnosticArray](http://docs.ros.org/en/latest/api/diagnostic_msgs/html/msg/DiagnosticArray.html) | Diagnostic status of the Planning component reported to the System component. |

### To API

| Name            | Topic                          | Type                                                                                                                                                                              | Description                                                                                                         |
| --------------- | ------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| Path Candidate  | `/planning/path_candidate/*`   | [autoware_auto_planning_msgs/msg/Path](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_planning_msgs/msg/Path.idl)                                      | The path Autoware is about to take. Users can interrupt the operation based on the path candidate information.      |
| Steering Factor | `/planning/steering_factor/*`  | [autoware_adapi_v1_msgs/msg/SteeringFactorArray](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/planning/msg/SteeringFactorArray.msg) | Information about the steering maneuvers performed by Autoware (e.g., steering to the right for a right turn, etc.) |
| Velocity Factor | `/planning/velocity_factors/*` | [autoware_adapi_v1_msgs/msg/VelocityFactorArray](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/planning/msg/VelocityFactorArray.msg) | Information about the velocity maneuvers performed by Autoware (e.g., stop for an obstacle, etc.)                   |

## Planning internal interface

This section explains the communication between the different planning modules shown in the [Planning Architecture Design](../../autoware-architecture/planning/index.md#high-level-architecture).

![planning-internal](images/planning-interface-internal.drawio.svg)

### From Mission Planning to Scenario Planning

| Name  | Topic                              | Type                                                                                                                                                 | Description                                                                          |
| ----- | ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| Route | `/planning/mission_planning/route` | [autoware_planning_msgs/msg/LaneletRoute](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/LaneletRoute.msg) | A sequence of lane IDs on a Lanelet map, from the starting point to the destination. |

### From Behavior Planning to Motion Planning

| Name | Topic                                                             | Type                                                                                                                                         | Description                                                                                                                                                                                                                                                                                                       |
| ---- | ----------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Path | `/planning/scenario_planning/lane_driving/behavior_planning/path` | [autoware_auto_planning_msgs/msg/Path](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_planning_msgs/msg/Path.idl) | A sequence of approximate vehicle positions for driving, along with information on the maximum speed and the drivable areas. Modules receiving this message are expected to make changes to the path within the constraints of the drivable areas and the maximum speed, generating the desired final trajectory. |

### From Scenario Planning to Validation

| Name       | Topic                                    | Type                                                                                                                                                     | Description                                                                                                                                           |
| ---------- | ---------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| Trajectory | `/planning/scenario_planning/trajectory` | [autoware_auto_planning_msgs/msg/Trajectory](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_planning_msgs/msg/Trajectory.idl) | A sequence of precise vehicle positions, speeds, and accelerations required for driving. It is expected that the vehicle will follow this trajectory. |
