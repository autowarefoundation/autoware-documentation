# Control component design

## Abstract

This document presents the design concept of the Control Component. The content is as follows:

- [Autoware Control Design](#autoware-control-design)
  - Outlining the policy for Autoware's control, which deals with only general information for autonomous driving systems and provides generic control commands to the vehicle.
- [Vehicle Adaptation Design](#vehicle-adaptation-design)
  - Describing the policy for vehicle adaptation, which utilizes adapter mechanisms to standardize the characteristics of the vehicle's drive system and integrate it with Autoware.
- [Control Feature Design](#control-feature-design)
  - Demonstrating the features provided by Autoware's control.
  - Presenting the approach towards the functions installed in the vehicle such as ABS.
- [Packages](#packages)
  - Listing the packages that implement the Control Component and how to select them.

## Autoware control design

The Control Component generates the control signal to which the Vehicle Component subscribes. The generated control signals are computed based on the reference trajectories from the Planning Component.

{{ diagram("image/control-component.svg", "The trajectory follower makes the control command, and the vehicle command gate selects the command that goes to the vehicle.") }}

The Control Component has these main parts:

<div class="aw-features aw-features--small" markdown>

- :fa-cl-s fa-route: **Trajectory follower**

  Makes a control command that follows the reference trajectory from planning. A lateral controller (MPC or pure pursuit) computes the steering. A longitudinal controller (PID) computes the velocity and the acceleration.

- :fa-cl-s fa-gears: **Shift decider**

  Decides the gear command (for example drive or reverse) from the control command of the trajectory follower.

- :fa-cl-s fa-shuffle: **Vehicle command gate**

  Selects one command source: the trajectory follower, an external operator, or the emergency commands of the minimal risk maneuver (MRM). It limits abnormal values, for example a sudden steering change, and sends the commands to the vehicle.

- :fa-cl-s fa-toggle-on: **Operation mode transition manager**

  Manages the operation mode (stop, autonomous, local, or remote). It checks that a transition to autonomous mode is safe, and requests the control mode from the vehicle.

- :fa-cl-s fa-gamepad: **External command selector**

  Selects the remote or the local operator command. The external command converter turns pedal inputs into a control command for the gate.

- :fa-cl-s fa-shield-halved: **Control checkers**

  Check the output of the control component and report diagnostics. Examples are the control validator, autonomous emergency braking (AEB), the lane departure checker, and the collision detector.

</div>

The Autoware control system is designed as a platform for automated driving systems that can be compatible with a diverse range of vehicles.

The control process in Autoware uses general information (such as target acceleration and deceleration) and no vehicle-specific information (such as brake pressure) is used. Hence it can be adjusted independently of the vehicle's drive interface enabling easy integration or performance tuning.

Furthermore, significant differences that affect vehicle motion constraints, such as two-wheel steering or four-wheel steering, are addressed by switching the control vehicle model, achieving control specialized for each characteristic.

Autoware's control module outputs the necessary information to control the vehicle as a substitute for a human driver. The control command is the [`autoware_control_msgs/msg/Control`](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Control.msg) message on `/control/command/control_cmd`. It has these values:

- Lateral: the target steering tire angle and the target steering tire rotation rate
- Longitudinal: the target velocity, the target acceleration, and the target jerk

Note that vehicle-specific values such as pedal positions and low-level information such as individual wheel rotation speeds are excluded from the command. The control component also sends the gear, turn indicators, and hazard lights commands to the vehicle. See the [vehicle interface page](../../interfaces/components/vehicle-interface.md) for the topics.

## Vehicle adaptation design

### Vehicle interface adapter

Autoware is designed to be an autonomous driving platform able to accommodate vehicles with various drivetrain types.

This is an explanation of how Autoware handles the standardization of systems with different vehicle drivetrain. The interfaces for vehicle drivetrain are diverse, including steering angle, steering angular velocity, steering torque, speed, accel/brake pedals, and brake pressure. To accommodate these differences, Autoware adds an adapter module between the control component and the vehicle interface. This module performs the conversion between the proprietary message types used by the vehicle (such as brake pressure) and the generic types used by Autoware (such as desired acceleration). By providing this conversion information, the differences in vehicle drivetrain can be accommodated.

In Autoware, the [`autoware_raw_vehicle_cmd_converter`](https://autowarefoundation.github.io/autoware_universe/main/vehicle/autoware_raw_vehicle_cmd_converter/) package is this adapter. It converts the target acceleration to accel and brake pedal values with lookup tables (the accel map and the brake map). It converts the target steering to a steering command with a steering map and a feedback controller, or with a variable gear ratio.

If the information is not known in advance, an automatic calibration tool can be used. Calibration will occur within limited degrees of freedom, generating the information necessary for the drivetrain conversion automatically. For example, [`autoware_accel_brake_map_calibrator`](https://autowarefoundation.github.io/autoware_universe/main/vehicle/autoware_accel_brake_map_calibrator/) updates the accel and brake maps from driving data.

This configuration is summarized in the following diagram.

{{ diagram("image/vehicle-interface-hw-abstraction.svg", "An adapter converts the generic control command into commands for each drive-by-wire interface.") }}

### Examples of several vehicle interfaces

This is an example of the several drivetrain types in the vehicle interface.

| Vehicle      | Lateral interface    | Longitudinal interface                                            | Note                                                                                                                                               |
| ------------ | -------------------- | ----------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| Lexus        | Steering angle       | Accel/brake pedal position                                        | Acceleration lookup table conversion for longitudinal                                                                                              |
| JPN TAXI     | Steering angle       | Accel/brake pedal position                                        | Acceleration lookup table conversion for longitudinal                                                                                              |
| GSM8         | Steering EPS voltage | Acceleration motor voltage, Deceleration brake hydraulic pressure | lookup table and PID conversion for lateral and longitudinal                                                                                       |
| YMC Golfcart | Steering angle       | Velocity                                                          |                                                                                                                                                    |
| Logiee       | yaw rate             | Velocity                                                          |                                                                                                                                                    |
| F1 TENTH     | Steering angle       | Motor RPM                                                         | [interface code](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/drivers/vesc_interface/src/vesc_interface.cpp) |

## Control feature design

The following lists the features provided by Autoware's Control/Vehicle component, as well as the conditions and assumptions required to utilize them effectively.

The proper operation of the ODD is limited by factors such as whether the functions are enabled, delay time, calibration accuracy and degradation rate, and sensor accuracy.

| Feature                                         | Description                                                                                                                                                                    | Requirements/Assumptions                                                          | Note                                                                                                                                                                                                                           | Limitation for now                                                                                                                                                         |
| ----------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Lateral Control                                 | Control the drivetrain system related to lateral vehicle motion                                                                                                                |                                                                                   | Trying to increase the number of vehicle types that can be supported in the future.                                                                                                                                            | Only front-steering type is supported.                                                                                                                                     |
| Longitudinal Control                            | Control the drivetrain system related to longitudinal vehicle motion                                                                                                           |                                                                                   |                                                                                                                                                                                                                                |                                                                                                                                                                            |
| Slope Compensation                              | Supports precise vehicle motion control on slopes                                                                                                                              | Gradient information can be obtained from maps or sensors attached to the chassis | If gradient information is not available, the gradient is estimated from the vehicle's pitch angle.                                                                                                                            |                                                                                                                                                                            |
| Delay Compensation                              | Controls the drivetrain system appropriately in the presence of time delays                                                                                                    | The drivetrain delay information is provided in advance                           | If there is no delay information, the drivetrain delay is estimated automatically (automatic calibration). However, the effect of delay cannot be completely eliminated, especially in scenarios with sudden changes in speed. | Only fixed delay times can be set for longitudinal and lateral drivetrain systems separately. It does not accommodate different delay times for the accelerator and brake. |
| Drivetrain IF Conversion (Lateral Control)      | Converts the drivetrain-specific information of the vehicle into the drivetrain information used by Autoware (for example target steering angular velocity → steering torque)  | The conversion information is provided in advance                                 | If there is no conversion information, the conversion map is estimated automatically (automatic calibration).                                                                                                                  | The degree of freedom for conversion is limited (2D lookup table + PID FB).                                                                                                |
| Drivetrain IF Conversion (Longitudinal Control) | Converts the drivetrain-specific information of the vehicle into the drivetrain information used by Autoware (for example target acceleration → accelerator/brake pedal value) | The conversion information is provided in advance                                 | If there is no conversion information, the conversion map is estimated automatically (automatic calibration).                                                                                                                  | The degree of freedom for conversion is limited (2D lookup table + PID FB).                                                                                                |
| Automatic Calibration                           | Automatically estimates and applies values such as drivetrain IF conversion map and delay time.                                                                                | The drivetrain status can be obtained (must)                                      |                                                                                                                                                                                                                                |                                                                                                                                                                            |
| Anomaly Detection                               | Notifies when there is a discrepancy in the calibration or unexpected drivetrain behavior                                                                                      | The drivetrain status can be obtained (must)                                      |                                                                                                                                                                                                                                |                                                                                                                                                                            |
| Steering Zero Point Correction                  | Corrects the midpoint of the steering to achieve appropriate steering control                                                                                                  | The drivetrain status can be obtained (must)                                      |                                                                                                                                                                                                                                |                                                                                                                                                                            |
| Steering Deadzone Correction                    | Corrects the deadzone of the steering to achieve appropriate steering control                                                                                                  | The steering deadzone parameter is provided in advance                            | If the parameter is unknown, the deadzone parameter is estimated from driving information                                                                                                                                      | Not available now                                                                                                                                                          |
| Steering Deadzone Estimation                    | Dynamically estimates the steering deadzone from driving data                                                                                                                  |                                                                                   |                                                                                                                                                                                                                                | Not available now                                                                                                                                                          |
| Weight Compensation                             | Performs appropriate vehicle control according to weight                                                                                                                       | Weight information can be obtained from sensors                                   | If there is no weight sensor, estimate the weight from driving information.                                                                                                                                                    | Currently not available                                                                                                                                                    |
| Weight Estimation                               | Dynamically estimates weight from driving data                                                                                                                                 |                                                                                   |                                                                                                                                                                                                                                | Currently not available                                                                                                                                                    |

These packages implement the available features:

- Slope compensation and longitudinal delay compensation: [`autoware_pid_longitudinal_controller`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_pid_longitudinal_controller/) (`enable_slope_compensation`, `slope_source`, and `delay_compensation_time`)
- Lateral delay compensation: [`autoware_mpc_lateral_controller`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_mpc_lateral_controller/) (`input_delay`)
- Drivetrain conversion: [`autoware_raw_vehicle_cmd_converter`](https://autowarefoundation.github.io/autoware_universe/main/vehicle/autoware_raw_vehicle_cmd_converter/)
- Automatic calibration of the accel and brake maps: [`autoware_accel_brake_map_calibrator`](https://autowarefoundation.github.io/autoware_universe/main/vehicle/autoware_accel_brake_map_calibrator/)
- Steering zero point correction: [`autoware_steer_offset_estimator`](https://autowarefoundation.github.io/autoware_universe/main/vehicle/autoware_steer_offset_estimator/)

The list above does not cover wheel control systems such as ABS commonly used in vehicles. Regarding these features, the following considerations are taken into account.

### Integration with vehicle-side functions

ABS (Anti-lock Brake System) and ESC (Electric Stability Control) are two functions that may be pre-installed on a vehicle, directly impacting its controllability.
The control modules of Autoware assume that both ABS and ESC are installed on the vehicle and their absence may cause unreliable controls depending on the target ODD. For example, with low-velocity driving in a controlled environment, these functions are not necessary.

Also, note that this statement does not negate the development of ABS functionality in autonomous driving systems.

### Autoware capabilities and vehicle requirements

As an alternative to human driving, autonomous driving systems essentially aim to handle tasks that humans can perform. This includes not only controlling the steering wheel, accel, and brake, but also automatically detecting issues such as poor brake response or a misaligned steering angle. However, this is a trade-off, as better vehicle performance will lead to superior system behavior, ultimately affecting the design of ODD.

On the other hand, for tasks that are not typically anticipated or cannot be handled by a human driver, processing in the vehicle ECU is expected. Examples of such scenarios include cases where the brake response is clearly delayed or when the vehicle rotates due to a single-side tire slipping. These tasks are typically handled by ABS or ESC.

## Packages

The control packages are in [Autoware Universe](https://github.com/autowarefoundation/autoware_universe/tree/main/control) and [Autoware Core](https://github.com/autowarefoundation/autoware_core/tree/main/control). The `autoware_control_launch` package of `autoware_launch` launches them in the `/control` namespace. These launch arguments select the controllers:

- `lateral_controller_mode`: `mpc` (default) or `pure_pursuit`
- `longitudinal_controller_mode`: `pid` (default)
- `module_preset`: the preset file that turns the optional modules on or off. The default preset launches the external command selector and converter, the lane departure checker, the control validator, AEB, the collision detector, and the control evaluator.

| Part                               | Package                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Repository |
| :--------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :--------- |
| Trajectory follower                | [`autoware_trajectory_follower_node`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_trajectory_follower_node/)                                                                                                                                                                                                                                                                                                                                                                                                                        | Universe   |
| Lateral controllers                | [`autoware_mpc_lateral_controller`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_mpc_lateral_controller/), [`autoware_pure_pursuit`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_pure_pursuit/)                                                                                                                                                                                                                                                                                                     | Universe   |
| Longitudinal controller            | [`autoware_pid_longitudinal_controller`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_pid_longitudinal_controller/)                                                                                                                                                                                                                                                                                                                                                                                                                  | Universe   |
| Experimental controller            | [`autoware_smart_mpc_trajectory_follower`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_smart_mpc_trajectory_follower/) (launch argument `trajectory_follower_mode`, under development)                                                                                                                                                                                                                                                                                                                                              | Universe   |
| Shift decider                      | [`autoware_shift_decider`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_shift_decider/)                                                                                                                                                                                                                                                                                                                                                                                                                                              | Universe   |
| Vehicle command gate               | [`autoware_vehicle_cmd_gate`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_vehicle_cmd_gate/) (default), or [`autoware_control_command_gate`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_control_command_gate/) with [`autoware_stop_mode_operator`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_stop_mode_operator/) when `use_control_command_gate` is `true`                                                                                                   | Universe   |
| Operation mode transition manager  | [`autoware_operation_mode_transition_manager`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_operation_mode_transition_manager/)                                                                                                                                                                                                                                                                                                                                                                                                      | Universe   |
| External commands                  | [`autoware_external_cmd_selector`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_external_cmd_selector/), [`autoware_external_cmd_converter`](https://autowarefoundation.github.io/autoware_universe/main/vehicle/autoware_external_cmd_converter/)                                                                                                                                                                                                                                                                                   | Universe   |
| Control checkers                   | [`autoware_control_validator`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_control_validator/), [`autoware_autonomous_emergency_braking`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_autonomous_emergency_braking/), [`autoware_lane_departure_checker`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_lane_departure_checker/), [`autoware_collision_detector`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_collision_detector/) | Universe   |
| Optional checkers (off by default) | [`autoware_obstacle_collision_checker`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_obstacle_collision_checker/), [`autoware_predicted_path_checker`](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_predicted_path_checker/)                                                                                                                                                                                                                                                                         | Universe   |
| Evaluation                         | [`autoware_control_evaluator`](https://autowarefoundation.github.io/autoware_universe/main/evaluator/autoware_control_evaluator/)                                                                                                                                                                                                                                                                                                                                                                                                                                    | Universe   |
| Minimal controller                 | [`autoware_simple_pure_pursuit`](https://autowarefoundation.github.io/autoware_core/main/control/autoware_simple_pure_pursuit/)                                                                                                                                                                                                                                                                                                                                                                                                                                      | Core       |
| Minimal command gate               | [`autoware_command_gate`](https://autowarefoundation.github.io/autoware_core/main/control/autoware_command_gate/)                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Core       |

The Core packages give a minimal control stack. The `autoware_core_control` launch file runs `autoware_simple_pure_pursuit`, and it runs `autoware_command_gate` only when `launch_command_gate` is `true` (for a Core-only system).
