# Vehicle interface

This page describes the interface of the vehicle interface component. See the [vehicle interface design](../../components/vehicle/index.md) for the concepts and the data flow.

{{ diagram("images/vehicle-interface.svg", "The vehicle interface component sits between the Control component and the vehicle.") }}

The `Vehicle Interface Component` receives `Vehicle commands` and publishes `Vehicle statuses`.
It communicates with the vehicle by the vehicle-specific protocol.

The optional `Vehicle command adapter` converts generalized control command (target steering, steering rate, velocity, acceleration) into vehicle-specific control values (steering-torque, wheel-torque, voltage, pressure, acceleration pedal position, etc).

## Communication with the vehicle

The interface to communicate with the vehicle varies between brands and models.
For example, a vehicle can use CAN (Controller Area Network) with a ROS 2 driver, such as [pacmod3](https://github.com/astuff/pacmod3).
In addition, an Autoware-specific interface is often necessary, such as [pacmod_interface](https://github.com/tier4/pacmod_interface/tree/main/pacmod_interface).

## Vehicle adapter

Autoware's basic control command express the target motion of the vehicle in terms of speed, acceleration, steering angle, and steering rate.
This does not suit all vehicles. Thus, there are two types of vehicles.

- Type 1: vehicle that is directly controlled by a subset of speed, acceleration, steering angle, and steering rate.
- Type 2: vehicle that uses custom commands (motor torque, voltage, pedal pressure, etc).

For vehicles of type 2,
a vehicle adapter is necessary to convert the Autoware control command into the vehicle specific commands.
For an example, see the [autoware_raw_vehicle_cmd_converter](https://autowarefoundation.github.io/autoware_universe/main/vehicle/autoware_raw_vehicle_cmd_converter/).
It converts the target speed and steering angle to accelerator, brake, and steering inputs, and publishes them as `/control/command/actuation_cmd`.
It uses `/vehicle/status/actuation_status` as feedback.

## Inputs from Autoware

| Name                   | Topic                                  | Type                                                                                                                                                                  | Description                                                                                                                  |
| ---------------------- | -------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| Control command        | `/control/command/control_cmd`         | [autoware_control_msgs/msg/Control](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Control.msg)                              | Target controls of the vehicle (steering angle, velocity, ...)                                                               |
| Control mode command   | `/control/control_mode_request`        | [autoware_vehicle_msgs/srv/ControlModeCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/srv/ControlModeCommand.srv)        | Request to switch between manual and autonomous driving                                                                      |
| Gear command           | `/control/command/gear_cmd`            | [autoware_vehicle_msgs/msg/GearCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/GearCommand.msg)                      | Target gear of the vehicle                                                                                                   |
| Hazard lights command  | `/control/command/hazard_lights_cmd`   | [autoware_vehicle_msgs/msg/HazardLightsCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/HazardLightsCommand.msg)      | Target values of the hazard lights                                                                                           |
| Turn indicator command | `/control/command/turn_indicators_cmd` | [autoware_vehicle_msgs/msg/TurnIndicatorsCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/TurnIndicatorsCommand.msg)  | Target values of the turn signals                                                                                            |
| Actuation command      | `/control/command/actuation_cmd`       | [tier4_vehicle_msgs/msg/ActuationCommandStamped](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_vehicle_msgs/msg/ActuationCommandStamped.msg) | Target accelerator, brake, and steering values. Only for a vehicle of type 2 that uses `autoware_raw_vehicle_cmd_converter`. |

## Outputs to Autoware

| Name                   | Topic                                    | Type                                                                                                                                                                     | Optional ?                           | Description                                                             |
| ---------------------- | ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------ | ----------------------------------------------------------------------- |
| Actuation status       | `/vehicle/status/actuation_status`       | [tier4_vehicle_msgs/msg/ActuationStatusStamped](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_vehicle_msgs/msg/ActuationStatusStamped.msg)      | Yes (vehicle with mechanical inputs) | Current acceleration, brake, and steer values reported by the vehicle   |
| Control mode           | `/vehicle/status/control_mode`           | [autoware_vehicle_msgs/msg/ControlModeReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/ControlModeReport.msg)             |                                      | Current control mode (manual, autonomous, ...)                          |
| Door status            | `/vehicle/doors/status`                  | [autoware_adapi_v1_msgs/msg/DoorStatusArray](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/vehicle/msg/DoorStatusArray.msg) | Yes                                  | Current door status                                                     |
| Gear report            | `/vehicle/status/gear_status`            | [autoware_vehicle_msgs/msg/GearReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/GearReport.msg)                           |                                      | Current gear of the vehicle                                             |
| Hazard light status    | `/vehicle/status/hazard_lights_status`   | [autoware_vehicle_msgs/msg/HazardLightsReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/HazardLightsReport.msg)           |                                      | Current hazard lights status                                            |
| Steering status        | `/vehicle/status/steering_status`        | [autoware_vehicle_msgs/msg/SteeringReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/SteeringReport.msg)                   |                                      | Current steering angle of the steering tire                             |
| Turn indicators status | `/vehicle/status/turn_indicators_status` | [autoware_vehicle_msgs/msg/TurnIndicatorsReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/TurnIndicatorsReport.msg)       |                                      | Current state of the left and right turn indicators                     |
| Velocity status        | `/vehicle/status/velocity_status`        | [autoware_vehicle_msgs/msg/VelocityReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/VelocityReport.msg)                   |                                      | Current velocities of the vehicle (longitudinal, lateral, heading rate) |

The [Component interfaces](index.md) page lists the QoS of these topics. The commands and the statuses of the vehicle interface also have detailed specifications:

- [/control/command/control_cmd](list/control-command-control-cmd.md)
- [/control/command/gear_cmd](list/control-command-gear-cmd.md)
- [/control/command/turn_indicators_cmd](list/control-command-turn-indicators-cmd.md)
- [/control/command/hazard_lights_cmd](list/control-command-hazard-lights-cmd.md)
- [/control/control_mode_request](list/control-control-mode-request.md)
- [/vehicle/status/control_mode](list/vehicle-status-control-mode.md)
- [/vehicle/status/velocity_status](list/vehicle-status-velocity-status.md)
- [/vehicle/status/steering_status](list/vehicle-status-steering-status.md)
- [/vehicle/status/gear_status](list/vehicle-status-gear-status.md)
- [/vehicle/status/turn_indicators_status](list/vehicle-status-turn-indicators-status.md)
- [/vehicle/status/hazard_lights_status](list/vehicle-status-hazard-lights-status.md)
