# Vehicle Interface

This page describes the Vehicle Interface Component. Please refer to the [Vehicle Interface design document](../../autoware-architecture/vehicle/index.md) for high-level concepts and data flow.

![Vehicle interface diagram](images/vehicle-interface.drawio.svg)

The `Vehicle Interface Component` receives `Vehicle commands` and publishes `Vehicle statuses`.
It communicates with the vehicle by the vehicle-specific protocol.

The optional `Vehicle command adapter` converts generalized control command (target steering, steering rate, velocity, acceleration) into vehicle-specific control values (steering-torque, wheel-torque, voltage, pressure, acceleration pedal position, etc).

## Communication with the vehicle

The interface to communicate with the vehicle varies between brands and models.
For example a vehicle specific message protocol like CAN (Controller Area Network) with a ROS 2 interface (e.g., [pacmod](https://github.com/astuff/pacmod3)).
In addition, an Autoware specific interface is often necessary (e.g., [pacmod_interface](https://github.com/tier4/pacmod_interface/tree/main/pacmod_interface)).

## Vehicle adapter

Autoware's basic control command express the target motion of the vehicle in terms of speed, acceleration, steering angle, and steering rate.
This may not be suitable for all vehicles and we thus distinguish between two types of vehicles.

- Type 1: vehicle that is directly controlled by a subset of speed, acceleration, steering angle, and steering rate.
- Type 2: vehicle that uses custom commands (motor torque, voltage, pedal pressure, etc).

For vehicles of type 2,
a vehicle adapter is necessary to convert the Autoware control command into the vehicle specific commands.
For an example, see the [raw_vehicle_cmd_converter](https://autowarefoundation.github.io/autoware.universe/main/vehicle/raw_vehicle_cmd_converter/)
which converts the target speed and steering angle to acceleration, steering, and brake mechanical inputs.

## Inputs from Autoware

| Name                   | Topic                                  | Type                                                                                                                                                                             | Description                                                    |
| ---------------------- | -------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- |
| Control command        | `/control/command/control_cmd`         | [autoware_auto_control_msgs/msg/AckermannControlCommand](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_control_msgs/msg/AckermannControlCommand.idl) | Target controls of the vehicle (steering angle, velocity, ...) |
| Control mode command   | `/control/control_mode_request`        | [ControlModeCommand](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/srv/ControlModeCommand.srv)                                          | Request to switch between manual and autonomous driving        |
| Gear command           | `/control/command/gear_cmd`            | [autoware_auto_vehicle_msgs/msg/GearCommand](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/GearCommand.idl)                         | Target gear of the vehicle                                     |
| Hazard lights command  | `/control/command/hazard_lights_cmd`   | [autoware_auto_vehicle_msgs/msg/HazardLightsCommand](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/HazardLightsCommand.idl)         | Target values of the hazard lights                             |
| Turn indicator command | `/control/command/turn_indicators_cmd` | [autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand.idl)     | Target values of the turn signals                              |

## Outputs to Autoware

| Name                   | Topic                                    | Type                                                                                                                                                                        | Optional ?                           | Description                                                             |
| ---------------------- | ---------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------ | ----------------------------------------------------------------------- |
| Actuation status       | `/vehicle/status/actuation_status`       | [tier4_vehicle_msgs/msg/ActuationStatusStamped](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_vehicle_msgs/msg/ActuationStatusStamped.msg)         | Yes (vehicle with mechanical inputs) | Current acceleration, brake, and steer values reported by the vehicle   |
| Control mode           | `/vehicle/status/control_mode`           | [autoware_auto_vehicle_msgs/msg/ControlModeReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/ControlModeReport.idl)        |                                      | Current control mode (manual, autonomous, ...)                          |
| Door status            | `/vehicle/status/door_status`            | [tier4_api_msgs/msg/DoorStatus](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_api_msgs/msg/DoorStatus.msg)                                         | Yes                                  | Current door status                                                     |
| Gear report            | `/vehicle/status/gear_status`            | [autoware_auto_vehicle_msgs/msg/GearReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/GearReport.idl)                      |                                      | Current gear of the vehicle                                             |
| Hazard light status    | `/vehicle/status/hazard_lights_status`   | [autoware_auto_vehicle_msgs/msg/HazardLightsReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/HazardLightsReport.idl)      |                                      | Current hazard lights status                                            |
| Steering status        | `/vehicle/status/steering_status`        | [autoware_auto_vehicle_msgs/msg/SteeringReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/SteeringReport.idl)              |                                      | Current steering angle of the steering tire                             |
| Steering wheel status  | `/vehicle/status/steering_wheel_status`  | [tier4_vehicle_msgs/msg/SteeringWheelStatusStamped](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_vehicle_msgs/msg/SteeringWheelStatusStamped.msg) | Yes                                  | Current steering wheel angle                                            |
| Turn indicators status | `/vehicle/status/turn_indicators_status` | [autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport.idl)  |                                      | Current state of the left and right turn indicators                     |
| Velocity status        | `/vehicle/status/velocity_status`        | [autoware_auto_vehicle_msgs/msg/VelocityReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/VelocityReport.idl)              |                                      | Current velocities of the vehicle (longitudinal, lateral, heading rate) |
