# Component interfaces

The components of Autoware talk to each other through the component interfaces. A component interface is a ROS topic or service with a fixed name, message type, and QoS. See [Autoware interface design](../index.md) for the concept.

The [`autoware_component_interface_specs`](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_component_interface_specs) package in Autoware Core is the source of truth for these interfaces. It defines each interface as a C++ struct, and it writes all of them to a machine-readable list, [`interface_manifest.json`](https://github.com/autowarefoundation/autoware_core/blob/main/common/autoware_component_interface_specs/interface_manifest.json). Every interface has the version `0.1.0` today. A `0.x` version means that the interface can still change.

## Components

<div class="grid cards aw-cards aw-cards--small" markdown>

- :fa-cl-s fa-satellite-dish: **[Sensing](sensing.md)**

  Sensor data after preprocessing.

- :fa-cl-s fa-map: **[Map](map.md)**

  Point cloud maps and Lanelet2 maps.

- :fa-cl-s fa-location-crosshairs: **[Localization](localization.md)**

  The pose, velocity, and acceleration of the vehicle.

- :fa-cl-s fa-eye: **[Perception](perception.md)**

  Objects, obstacles, and traffic lights.

- :fa-cl-s fa-route: **[Planning](planning.md)**

  The route and the trajectory.

- :fa-cl-s fa-gauge-high: **[Control](control.md)**

  Commands to the vehicle.

- :fa-cl-s fa-car: **[Vehicle interface](vehicle-interface.md)**

  Commands in, vehicle status out.

- :fa-cl-s fa-ruler-combined: **[Vehicle dimensions](vehicle-dimensions.md)**

  The axes and the size of the vehicle.

</div>

## Interfaces in Autoware Core

The table lists every interface in `interface_manifest.json`. All topics use the `reliable` QoS reliability. Services use the default service QoS of ROS 2. A linked name opens the detailed specification of the interface.

| Domain       | Interface                                                                                 | Type                                                         | Kind    | QoS                      |
| ------------ | ----------------------------------------------------------------------------------------- | ------------------------------------------------------------ | ------- | ------------------------ |
| Control      | [`/control/command/control_cmd`](list/control-command-control-cmd.md)                     | `autoware_control_msgs/msg/Control`                          | Topic   | depth 1, volatile        |
| Control      | [`/control/command/gear_cmd`](list/control-command-gear-cmd.md)                           | `autoware_vehicle_msgs/msg/GearCommand`                      | Topic   | depth 1, volatile        |
| Control      | [`/control/command/turn_indicators_cmd`](list/control-command-turn-indicators-cmd.md)     | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand`            | Topic   | depth 1, volatile        |
| Control      | [`/control/command/hazard_lights_cmd`](list/control-command-hazard-lights-cmd.md)         | `autoware_vehicle_msgs/msg/HazardLightsCommand`              | Topic   | depth 1, volatile        |
| Control      | `/control/trajectory_follower/lateral/predicted_trajectory`                               | `autoware_planning_msgs/msg/Trajectory`                      | Topic   | depth 1, volatile        |
| Control      | [`/control/control_mode_request`](list/control-control-mode-request.md)                   | `autoware_vehicle_msgs/srv/ControlModeCommand`               | Service |                          |
| Localization | `/localization/initialize`                                                                | `autoware_localization_msgs/srv/InitializeLocalization`      | Service |                          |
| Localization | `/localization/initialization_state`                                                      | `autoware_adapi_v1_msgs/msg/LocalizationInitializationState` | Topic   | depth 1, transient_local |
| Localization | `/localization/kinematic_state`                                                           | `nav_msgs/msg/Odometry`                                      | Topic   | depth 1, volatile        |
| Localization | `/localization/acceleration`                                                              | `geometry_msgs/msg/AccelWithCovarianceStamped`               | Topic   | depth 1, volatile        |
| Map          | `/map/vector_map`                                                                         | `autoware_map_msgs/msg/LaneletMapBin`                        | Topic   | depth 1, transient_local |
| Map          | `/map/map_projector_info`                                                                 | `autoware_map_msgs/msg/MapProjectorInfo`                     | Topic   | depth 1, transient_local |
| Map          | `/map/get_differential_pointcloud_map`                                                    | `autoware_map_msgs/srv/GetDifferentialPointCloudMap`         | Service |                          |
| Map          | `/map/get_partial_pointcloud_map`                                                         | `autoware_map_msgs/srv/GetPartialPointCloudMap`              | Service |                          |
| Perception   | `/perception/object_recognition/objects`                                                  | `autoware_perception_msgs/msg/PredictedObjects`              | Topic   | depth 1, volatile        |
| Perception   | `/perception/object_recognition/tracking/objects`                                         | `autoware_perception_msgs/msg/TrackedObjects`                | Topic   | depth 1, volatile        |
| Perception   | `/perception/traffic_light_recognition/traffic_signals`                                   | `autoware_perception_msgs/msg/TrafficLightGroupArray`        | Topic   | depth 1, volatile        |
| Planning     | `/planning/set_lanelet_route`                                                             | `autoware_planning_msgs/srv/SetLaneletRoute`                 | Service |                          |
| Planning     | `/planning/set_waypoint_route`                                                            | `autoware_planning_msgs/srv/SetWaypointRoute`                | Service |                          |
| Planning     | `/planning/clear_route`                                                                   | `autoware_planning_msgs/srv/ClearRoute`                      | Service |                          |
| Planning     | `/planning/route_state`                                                                   | `autoware_planning_msgs/msg/RouteState`                      | Topic   | depth 1, transient_local |
| Planning     | `/planning/route`                                                                         | `autoware_planning_msgs/msg/LaneletRoute`                    | Topic   | depth 1, transient_local |
| Planning     | `/planning/trajectory`                                                                    | `autoware_planning_msgs/msg/Trajectory`                      | Topic   | depth 1, volatile        |
| Sensing      | `/sensing/vehicle_velocity_converter/twist_with_covariance`                               | `geometry_msgs/msg/TwistWithCovarianceStamped`               | Topic   | depth 10, volatile       |
| System       | `/system/operation_mode/state`                                                            | `autoware_adapi_v1_msgs/msg/OperationModeState`              | Topic   | depth 1, transient_local |
| System       | `/system/operation_mode/change_operation_mode`                                            | `autoware_system_msgs/srv/ChangeOperationMode`               | Service |                          |
| System       | `/system/operation_mode/change_autoware_control`                                          | `autoware_system_msgs/srv/ChangeAutowareControl`             | Service |                          |
| System       | `/system/fail_safe/mrm_state`                                                             | `autoware_adapi_v1_msgs/msg/MrmState`                        | Topic   | depth 1, volatile        |
| Vehicle      | [`/vehicle/status/velocity_status`](list/vehicle-status-velocity-status.md)               | `autoware_vehicle_msgs/msg/VelocityReport`                   | Topic   | depth 1, volatile        |
| Vehicle      | [`/vehicle/status/steering_status`](list/vehicle-status-steering-status.md)               | `autoware_vehicle_msgs/msg/SteeringReport`                   | Topic   | depth 1, volatile        |
| Vehicle      | [`/vehicle/status/gear_status`](list/vehicle-status-gear-status.md)                       | `autoware_vehicle_msgs/msg/GearReport`                       | Topic   | depth 1, volatile        |
| Vehicle      | [`/vehicle/status/turn_indicators_status`](list/vehicle-status-turn-indicators-status.md) | `autoware_vehicle_msgs/msg/TurnIndicatorsReport`             | Topic   | depth 1, volatile        |
| Vehicle      | [`/vehicle/status/hazard_lights_status`](list/vehicle-status-hazard-lights-status.md)     | `autoware_vehicle_msgs/msg/HazardLightsReport`               | Topic   | depth 1, volatile        |
| Vehicle      | [`/vehicle/status/control_mode`](list/vehicle-status-control-mode.md)                     | `autoware_vehicle_msgs/msg/ControlModeReport`                | Topic   | depth 1, volatile        |

## Interfaces in Autoware Universe

The [`autoware_component_interface_specs_universe`](https://github.com/autowarefoundation/autoware_universe/tree/main/common/autoware_component_interface_specs_universe) package defines more interfaces that only Autoware Universe uses.

| Domain  | Interface                                      | Type                                             | Kind    | QoS                      |
| ------- | ---------------------------------------------- | ------------------------------------------------ | ------- | ------------------------ |
| Control | `/control/command/actuation_cmd`               | `tier4_vehicle_msgs/msg/ActuationCommandStamped` | Topic   | depth 1, volatile        |
| Control | `/control/vehicle_cmd_gate/set_pause`          | `tier4_control_msgs/srv/SetPause`                | Service |                          |
| Control | `/control/vehicle_cmd_gate/is_paused`          | `tier4_control_msgs/msg/IsPaused`                | Topic   | depth 1, transient_local |
| Control | `/control/vehicle_cmd_gate/is_start_requested` | `tier4_control_msgs/msg/IsStartRequested`        | Topic   | depth 1, transient_local |
| Control | `/control/vehicle_cmd_gate/set_stop`           | `tier4_control_msgs/srv/SetStop`                 | Service |                          |
| Control | `/control/vehicle_cmd_gate/is_stopped`         | `tier4_control_msgs/msg/IsStopped`               | Topic   | depth 1, transient_local |
| Vehicle | `/vehicle/status/battery_charge`               | `tier4_vehicle_msgs/msg/BatteryStatus`           | Topic   | depth 1, volatile        |
| Vehicle | `/vehicle/doors/command`                       | `autoware_adapi_v1_msgs/srv/SetDoorCommand`      | Service |                          |
| Vehicle | `/vehicle/doors/layout`                        | `autoware_adapi_v1_msgs/srv/GetDoorLayout`       | Service |                          |
| Vehicle | `/vehicle/doors/status`                        | `autoware_adapi_v1_msgs/msg/DoorStatusArray`     | Topic   | depth 1, transient_local |

## Write a specification page

Each detailed specification page uses one of these templates. Copy the template, fill in the front matter, and replace the text of each section.

- [Topic template](../templates/topic.md)
- [Service template](../templates/service.md)
