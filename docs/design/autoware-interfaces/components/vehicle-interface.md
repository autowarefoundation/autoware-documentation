# Vehicle Interface

This page provides specific specifications about the interface of the Vehicle Interface Component. Please refer to the [Vehicle Interface design document](../../autoware-architecture/vehicle/) for high-level concepts and data flow.

**TODO: The detailed definitions (meanings of elements included in each topic) are not described yet, need to be updated.**

The `Vehicle Interface` receives the `Vehicle Signal Commands` and `Vehicle Control Commands` and publishes the vehicle status. It also communicates with vehicle by the vehicle-specific protocol.

The `Adapter` converts generalized control command (target steering, steering rate, velocity, acceleration, jerk) into vehicle-specific control values (steering-torque, wheel-torque, voltage, pressure, accel pedal position, etc).

## Inputs

### From Planning Component
### From Control Component

| Name                    | Topic                           | Type                                                                                                                                      | Description                                        |
| ----------------------- | ------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| Control command | `/control/command/control_cmd` | [autoware_auto_control_msgs/msg/AckermannControlCommand](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_control_msgs/msg/AckermannControlCommand.idl) | Target controls of the vehicle (steering angle, velocity, ...) |
| Emergency command | `/control/command/emergency_cmd` | [tier4_vehicle_msgs/msg/VehicleEmergencyStamped](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_vehicle_msgs/msg/VehicleEmergencyStamped.msg) | Whether the vehicle should enter its emergency mode |
| Gear command | `/control/command/gear_cmd` | [autoware_auto_vehicle_msgs/msg/GearCommand](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/GearCommand.idl) | Target gear of the vehicle |
| Hazard lights command | `/control/command/hazard_lights_cmd` | [autoware_auto_vehicle_msgs/msg/HazardLightsCommand](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/HazardLightsCommand.idl) | Target values of the hazard lights |
| Turn indicator command | `/control/command/turn_indicators_cmd` | [autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand.idl) | Target values of the turn signals |


### From API

## Outputs

### To Autoware

| Name                    | Topic                           | Type                                                                                                                                      | Description                                        |
| ----------------------- | ------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| Actuation status | `/vehicle/status/actuation_status` | [tier4_vehicle_msgs/msg/ActuationStatusStamped](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_vehicle_msgs/msg/ActuationStatusStamped.msg) | Currentr acceleration, brake, and steer values reported by the vehicle |
| Control mode | `/vehicle/status/control_mode` | [autoware_auto_vehicle_msgs/msg/ControlModeReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/ControlModeReport.idl) | Current control mode (manual, autonomous, ...) |
| Door status | `/vehicle/status/door_status` | [tier4_api_msgs/msg/DoorStatus](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_api_msgs/msg/DoorStatus.msg) | Current door status |
| Gear report | `/vehicle/status/gear_status` | [autoware_auto_vehicle_msgs/msg/GearReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/GearReport.idl) | Current gear of the vehicle |
| Hazard light status | `/vehicle/status/hazard_lights_status` | [autoware_auto_vehicle_msgs/msg/HazardLightsReport]() | Current hazard lights status |
| Steering status | `/vehicle/status/steering_status` | [autoware_auto_vehicle_msgs/msg/SteeringReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/SteeringReport.idl) | Current steering angle of the steering tire |
| Steering wheel status | `/vehicle/status/steering_wheel_status` | [tier4_vehicle_msgs/msg/SteeringWheelStatusStamped](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_vehicle_msgs/msg/SteeringWheelStatusStamped.msg) | Current steering wheel angle |
| Turn indicators status | `/vehicle/status/turn_indicators_status` | [autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport.idl) | Current state of the left and right turn indicators |
| Velocity status | `/vehicle/status/velocity_status` | [autoware_auto_vehicle_msgs/msg/VelocityReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/VelocityReport.idl) | Current velocities of the vehicle (longitudinal, lateral, heading rate) |

### To the vehicle

Vehicle specific messages protocol like CAN (Controller Area Network).


### To Control Component

### To Localization Component

Odometry of the vehicle. Used by the Localization module to update the pose of the vehicle in the map.

- [geometry_msgs/TwistWithCovarianceStamped](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html) odometry


## Vehicle Communication

## Internal interface

| Actuation command | /control/command/actuation_cmd | tier4_vehicle_msgs/msg/ActuationCommandStamped | Target TODO |