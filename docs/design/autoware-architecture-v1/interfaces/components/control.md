# Control

This page describes the interface of the Control component. See the [Control component design](../../components/control/index.md) for the concepts.

The Control component follows the trajectory from Planning. It sends the commands to the vehicle interface. The default implementation is in [`autoware_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_universe_launch/autoware_control_launch).

{{ diagram("images/Control-Bus-ODD-Architecture.svg", "The main nodes and topics of the Control component.") }}

## Inputs

| Name                    | Topic                                                    | Type                                                                                                                                                                 | Description                                                                                                            |
| ----------------------- | -------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| Trajectory              | `/planning/trajectory`                                   | [autoware_planning_msgs/msg/Trajectory](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/Trajectory.msg)                     | The trajectory to follow. See the outputs of [Planning](planning.md#to-control).                                       |
| Turn indicators command | `/planning/turn_indicators_cmd`                          | [autoware_vehicle_msgs/msg/TurnIndicatorsCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/TurnIndicatorsCommand.msg) | The turn indicators that Planning requests.                                                                            |
| Hazard lights command   | `/planning/hazard_lights_cmd`                            | [autoware_vehicle_msgs/msg/HazardLightsCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/HazardLightsCommand.msg)     | The hazard lights that Planning requests.                                                                              |
| Vehicle kinematic state | `/localization/kinematic_state`                          | [nav_msgs/msg/Odometry](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg)                                                            | The current pose and velocity of the vehicle. See the outputs of [Localization](localization.md).                      |
| Vehicle acceleration    | `/localization/acceleration`                             | [geometry_msgs/msg/AccelWithCovarianceStamped](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/AccelWithCovarianceStamped.msg)              | The current acceleration of the vehicle.                                                                               |
| Steering status         | `/vehicle/status/steering_status`                        | [autoware_vehicle_msgs/msg/SteeringReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/SteeringReport.msg)               | The current steering tire angle. See the outputs of the [vehicle interface](vehicle-interface.md#outputs-to-autoware). |
| Gear status             | `/vehicle/status/gear_status`                            | [autoware_vehicle_msgs/msg/GearReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/GearReport.msg)                       | The current gear. The shift decider uses it.                                                                           |
| Control mode            | `/vehicle/status/control_mode`                           | [autoware_vehicle_msgs/msg/ControlModeReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/ControlModeReport.msg)         | The current control mode of the vehicle, for example manual or autonomous.                                             |
| Operation mode change   | `/system/operation_mode/change_operation_mode` (service) | [autoware_system_msgs/srv/ChangeOperationMode](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_system_msgs/srv/ChangeOperationMode.srv)       | A request from the AD API to change the operation mode.                                                                |

The control checkers, for example the autonomous emergency braking, also use `/perception/object_recognition/objects` and `/perception/obstacle_segmentation/pointcloud`. The lane departure checker uses `/map/vector_map` and `/planning/mission_planning/route`.

## Outputs

| Name                    | Topic                                                       | Type                                                                                                                                                                                  | Description                                                                                                         |
| ----------------------- | ----------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| Control command         | `/control/command/control_cmd`                              | [autoware_control_msgs/msg/Control](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Control.msg)                                              | The target lateral and longitudinal motion. See [the specification](list/control-command-control-cmd.md).           |
| Gear command            | `/control/command/gear_cmd`                                 | [autoware_vehicle_msgs/msg/GearCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/GearCommand.msg)                                      | The target gear. See [the specification](list/control-command-gear-cmd.md).                                         |
| Turn indicators command | `/control/command/turn_indicators_cmd`                      | [autoware_vehicle_msgs/msg/TurnIndicatorsCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/TurnIndicatorsCommand.msg)                  | The target turn indicators. See [the specification](list/control-command-turn-indicators-cmd.md).                   |
| Hazard lights command   | `/control/command/hazard_lights_cmd`                        | [autoware_vehicle_msgs/msg/HazardLightsCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/HazardLightsCommand.msg)                      | The target hazard lights. See [the specification](list/control-command-hazard-lights-cmd.md).                       |
| Control mode request    | `/control/control_mode_request` (service)                   | [autoware_vehicle_msgs/srv/ControlModeCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/srv/ControlModeCommand.srv)                        | A request to the vehicle to change the control mode. See [the specification](list/control-control-mode-request.md). |
| Operation mode state    | `/system/operation_mode/state`                              | [autoware_adapi_v1_msgs/msg/OperationModeState](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/operation_mode/msg/OperationModeState.msg) | The current operation mode. In the default setup, the operation mode transition manager publishes it.               |
| Predicted trajectory    | `/control/trajectory_follower/lateral/predicted_trajectory` | [autoware_planning_msgs/msg/Trajectory](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/Trajectory.msg)                                      | The path that the lateral controller predicts for the vehicle.                                                      |

## Control command message

The [autoware_control_msgs/msg/Control](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Control.msg) message has these fields. See the [README of autoware_control_msgs](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/README.md) for their meaning.

- builtin_interfaces/Time stamp
- builtin_interfaces/Time control_time
- [autoware_control_msgs/msg/Lateral](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Lateral.msg) lateral
  - builtin_interfaces/Time stamp
  - builtin_interfaces/Time control_time
  - float32 steering_tire_angle
  - float32 steering_tire_rotation_rate
  - bool is_defined_steering_tire_rotation_rate
- [autoware_control_msgs/msg/Longitudinal](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Longitudinal.msg) longitudinal
  - builtin_interfaces/Time stamp
  - builtin_interfaces/Time control_time
  - float32 velocity
  - float32 acceleration
  - float32 jerk
  - bool is_defined_acceleration
  - bool is_defined_jerk

For a vehicle that takes pedal or torque commands, the [autoware_raw_vehicle_cmd_converter](https://github.com/autowarefoundation/autoware_universe/tree/main/vehicle/autoware_raw_vehicle_cmd_converter) converts this command into actuation commands. It uses the actuation status of the vehicle (`/vehicle/status/actuation_status`). See [Vehicle interface](vehicle-interface.md#vehicle-adapter).
