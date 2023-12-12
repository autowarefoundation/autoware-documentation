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
| Control command | `/control/command/control_cmd` | autoware_auto_control_msgs/msg/AckermannControlCommand | Target control of the vehicle (steering, velocity, ...) |
| Emergency command | `/control/command/emergency_cmd` | tier4_vehicle_msgs/msg/VehicleEmergencyStamped | TODO |
| Gear command | `/control/command/gear_cmd` | autoware_auto_vehicle_msgs/msg/GearCommand | Target gear of the vehicle |
| Hazard lights command | `/control/command/hazard_lights_cmd` | autoware_auto_vehicle_msgs/msg/HazardLightsCommand | Control of hazard lights |
| Turn indicator command | `/control/command/turn_indicators_cmd` | autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand | Control of turn signals |


### From API

## Outputs

### To Autoware

| Name                    | Topic                           | Type                                                                                                                                      | Description                                        |
| ----------------------- | ------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| | `/vehicle/status/actuation_status` | tier4_vehicle_msgs/msg/ActuationStatusStamped | TODO |
| | `/vehicle/status/control_mode` | autoware_auto_vehicle_msgs/msg/ControlModeReport | TODO |
| | `/vehicle/status/door_status` | tier4_api_msgs/msg/DoorStatus | TODO |
| | `/vehicle/status/gear_status` | autoware_auto_vehicle_msgs/msg/GearReport | TODO |
| | `/vehicle/status/hazard_lights_status` | autoware_auto_vehicle_msgs/msg/HazardLightsReport | TODO |
| | `/vehicle/status/steering_status` | autoware_auto_vehicle_msgs/msg/SteeringReport | TODO |
| | `/vehicle/status/steering_wheel_status` | tier4_vehicle_msgs/msg/SteeringWheelStatusStamped | TODO |
| | `/vehicle/status/turn_indicators_status` | autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport | TODO |
| | `/vehicle/status/velocity_status` | autoware_auto_vehicle_msgs/msg/VelocityReport | TODO |

### To the vehicle

Vehicle specific messages protocol like CAN (Controller Area Network).


### To Control Component

### To Localization Component

Odometry of the vehicle. Used by the Localization module to update the pose of the vehicle in the map.

- [geometry_msgs/TwistWithCovarianceStamped](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html) odometry


## Vehicle Communication

## Internal interface

| Actuation command | /control/command/actuation_cmd | tier4_vehicle_msgs/msg/ActuationCommandStamped | Target TODO |