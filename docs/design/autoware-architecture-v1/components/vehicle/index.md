# Vehicle interface design

## Abstract

The Vehicle Interface component provides an interface between Autoware and a vehicle that passes control signals to the vehicle’s drive-by-wire system and receives vehicle information that is passed back to Autoware.

## 1. Requirements

Goals:

- The Vehicle Interface component converts Autoware commands to a vehicle-specific format and converts vehicle status in a vehicle-specific format to Autoware messages.
- The interface between Autoware and the Vehicle component is abstracted and independent of hardware.
- The interface is extensible such that additional vehicle-specific commands can be easily added. For example, headlight control.

Non-goals:

- Accuracy of responses from the vehicle will not be defined, but example accuracy requirements from reference designs are provided as examples.
- Response speed will not be defined.

## 2. Architecture

The Vehicle Interface component consists of the following components:

- A Raw Vehicle Command Converter component that will pass through vehicle commands from the Control component if velocity/acceleration control is supported by the drive-by-wire system. Otherwise, the Control commands will be modified according to the control method (for example converting a target acceleration from the Control component to a vehicle specific accel/brake pedal value through the use of an acceleration map). In Autoware, the [`autoware_raw_vehicle_cmd_converter`](https://autowarefoundation.github.io/autoware_universe/main/vehicle/autoware_raw_vehicle_cmd_converter/) package does this conversion.
- A Vehicle Interface component (vehicle specific) that acts as an interface between Autoware and a vehicle to communicate control signals and to obtain information about the vehicle (steer output, tire angle, etc.)

Each component runs as one or more ROS 2 nodes that the vehicle launch package starts. The component has no plugin mechanism that loads or unloads parts while Autoware runs. `autoware_raw_vehicle_cmd_converter` registers its node as a ROS 2 component, but its own launch file and `awsim_labs_vehicle_launch` start it as a separate process. Vehicle-specific interfaces such as [`pacmod_interface`](https://github.com/tier4/pacmod_interface) build normal executables. The mechanism of the Vehicle Interface component is depicted by the following figures:

{{ diagram("image/vehicle-interface-architecture.svg", "The commands that go into the vehicle interface and the status that comes out of it.") }}

{{ diagram("image/vehicle-interface-overview.svg", "The vehicle command adapter sits between the control component and the vehicle interface.") }}

The `tier4_vehicle_launch` package of `autoware_launch` starts the vehicle component. It runs `robot_state_publisher` with the vehicle and sensor description, and it includes the `vehicle_interface.launch.xml` file of the vehicle launch package (`<vehicle_model>_launch`) when `launch_vehicle_interface` is `true`. Each vehicle launch package decides which nodes to run. For example, `awsim_labs_vehicle_launch` runs `autoware_raw_vehicle_cmd_converter`, and `sample_vehicle_launch` runs no vehicle interface node.

## 3. Features

The Vehicle Interface component can provide the following features in functionality and capability:

- Basic functions
  - Converting Autoware control commands to vehicle specific command
  - Converting vehicle specific status information (velocity, steering) to Autoware status message

- Diagnostics
  - List available features
  - Provide a warning if the Control component tries to use a feature that is not available in the Vehicle Interface component

Additional functionality and capability features may be added, depending on the vehicle hardware. Some example features are listed below:

- Safety features
  - Disengage autonomous driving via manual intervention.
    - This can be done through the use of an emergency disengage button, or by a safety driver manually turning the steering wheel or pressing the brake
- Optional controls
  - Turn indicator
  - Handbrake
  - Headlights
  - Hazard lights
  - Doors
  - Horn
  - Wipers

Autoware has standard messages for the turn indicators, the hazard lights, the gear, and the doors. The other optional controls need vehicle-specific messages.

## 4. Interface and data structure

The [`autoware_component_interface_specs`](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_component_interface_specs) package of Autoware Core defines the topics between the control component and the vehicle interface. The messages are in [`autoware_vehicle_msgs`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_vehicle_msgs) and [`autoware_control_msgs`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_control_msgs). The [vehicle interface page](../../interfaces/components/vehicle-interface.md) gives the full list with links to the message definitions.

### Inputs from Autoware

The vehicle command gate of the Control component sends these commands. Planning makes the turn indicators and hazard lights commands, and the shift decider makes the gear command.

| Input                        | Topic                                  | Type                                                     |
| :--------------------------- | :------------------------------------- | :------------------------------------------------------- |
| Control command              | `/control/command/control_cmd`         | `autoware_control_msgs/msg/Control`                      |
| Gear command                 | `/control/command/gear_cmd`            | `autoware_vehicle_msgs/msg/GearCommand`                  |
| Turn indicators command      | `/control/command/turn_indicators_cmd` | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand`        |
| Hazard lights command        | `/control/command/hazard_lights_cmd`   | `autoware_vehicle_msgs/msg/HazardLightsCommand`          |
| Control mode request         | `/control/control_mode_request`        | `autoware_vehicle_msgs/srv/ControlModeCommand` (service) |
| Emergency command (optional) | `/control/command/emergency_cmd`       | `tier4_vehicle_msgs/msg/VehicleEmergencyStamped`         |
| Door command (optional)      | `/vehicle/doors/command`               | `autoware_adapi_v1_msgs/srv/SetDoorCommand` (service)    |

The control command has a lateral part (steering tire angle and rotation rate) and a longitudinal part (velocity, acceleration, and jerk). If the vehicle needs mechanical inputs, the raw vehicle command converter changes the control command to an actuation command (accel, brake, and steering values) on `/control/command/actuation_cmd`.

### Outputs to Autoware

| Output                      | Topic                                    | Type                                             |
| :-------------------------- | :--------------------------------------- | :----------------------------------------------- |
| Velocity status             | `/vehicle/status/velocity_status`        | `autoware_vehicle_msgs/msg/VelocityReport`       |
| Steering status             | `/vehicle/status/steering_status`        | `autoware_vehicle_msgs/msg/SteeringReport`       |
| Gear status                 | `/vehicle/status/gear_status`            | `autoware_vehicle_msgs/msg/GearReport`           |
| Turn indicators status      | `/vehicle/status/turn_indicators_status` | `autoware_vehicle_msgs/msg/TurnIndicatorsReport` |
| Hazard lights status        | `/vehicle/status/hazard_lights_status`   | `autoware_vehicle_msgs/msg/HazardLightsReport`   |
| Control mode                | `/vehicle/status/control_mode`           | `autoware_vehicle_msgs/msg/ControlModeReport`    |
| Actuation status (optional) | `/vehicle/status/actuation_status`       | `tier4_vehicle_msgs/msg/ActuationStatusStamped`  |
| Door status (optional)      | `/vehicle/doors/status`                  | `autoware_adapi_v1_msgs/msg/DoorStatusArray`     |

The velocity status goes to the Localization component. The `autoware_vehicle_velocity_converter` package of the Sensing component converts it to `geometry_msgs/msg/TwistWithCovarianceStamped`. The control mode tells if the vehicle is under autonomous control or manual control.

## 5. Concerns, assumptions, and limitations

### Concerns

- Architectural trade-offs and scalability
- The raw vehicle command converter uses the actuation messages of `tier4_vehicle_msgs` (`ActuationCommandStamped` and `ActuationStatusStamped`). `autoware_vehicle_msgs` now also defines `ActuationCommandStamped` and `ActuationReportStamped`, but the converter does not use them yet.

### Assumptions

- The control command is for an Ackermann-style platform. If the vehicle has more than one steering tire, the steering tire angle is for a virtual tire at the average lateral position of the steering tires.
- The vehicle interface publishes the velocity status and the steering status at about 10 Hz, and the control command comes at about 33 Hz. The [interface list](../../interfaces/components/index.md) gives the rate of each topic.
- A vehicle without gears simulates the gear behavior. A vehicle without turn indicators or hazard lights ignores those commands and reports them as disabled.

### Limitations

- The control mode request supports these modes: `AUTONOMOUS`, `AUTONOMOUS_STEER_ONLY`, `AUTONOMOUS_VELOCITY_ONLY`, `MANUAL`, and `NO_COMMAND`. The control mode report also has `DISENGAGED` and `NOT_READY`.
- Autoware has no standard message for the handbrake, the headlights, the horn, or the wipers.

## 6. Examples of accuracy requirements by ODD

This section has no examples yet.
