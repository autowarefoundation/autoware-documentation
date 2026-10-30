---
last_updated: 2026-01-21
interface_type: topic
interface_name: /control/command/control_cmd
data_type_name: autoware_control_msgs/msg/Control
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Control.msg
rate: 33
qos_reliability: reliable
qos_durability: volatile
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Send the control command to the vehicle. This is the command format that can be used universally in vehicles based on the [Ackermann kinematic model][ackermann-kinematic-model].
At this stage, do not expect anything other than simple control such as smoothly approaching the target value.
For example, the time-series control involving acceleration and deceleration to reach a target speed must have been completed in a previous stage.
Therefore, it is generally assumed that the target value will be sent at a time close enough that linear interpolation is possible.
If your vehicle interface would like to support time series data directly, please join the discussion of the [ControlHorizon](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/ControlHorizon.msg) message.

The [`autoware_component_interface_specs`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/common/autoware_component_interface_specs/include/autoware/component_interface_specs/control.hpp) package defines this topic with `volatile` durability. In the default launch of Autoware, the [`autoware_vehicle_cmd_gate`](https://github.com/autowarefoundation/autoware_universe/blob/0.53.1/control/autoware_vehicle_cmd_gate/src/vehicle_cmd_gate.cpp) node publishes this topic with `transient_local` durability. `volatile` subscribers also receive its messages. Make sure that the publishers and the subscribers in your system use compatible QoS settings.

[ackermann-kinematic-model]: ../../../../../tutorials/integrating-autoware/creating-vehicle-interface-package/ackermann-kinematic-model.md

## Message

For details about the message, [see the readme of autoware_control_msgs](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/README.md).

The velocity, acceleration, and jerk values at a given time must be planned in advance and must be consistent, which is typically done by the planning/control component.
If these values are inconsistent, which value takes priority depends on the vehicle implementation.

Sending the latest command will invalidate previous commands, which means that commands from different time stamps are not kept as a time series.
Only the latest command, not the latest time stamp command, is valid.

## Errors

Command Timeout: If the command is not sent frequently enough to control the current vehicle speed, the vehicle executes an emergency stop.

Out of Range: If the command exceeds the given physical limit, the value is clamped to the maximum.

## Support

This interface is required. If there is only a vehicle-specific interface, provide a converter.
For vehicles controlled by typical accelerator and brake pedals, consider using the [autoware_raw_vehicle_cmd_converter](https://github.com/autowarefoundation/autoware_universe/tree/main/vehicle/autoware_raw_vehicle_cmd_converter).

## Limitations

Command filter: The commands may be filtered by a rate limiter to prevent physically impossible or unsafe rapid acceleration or steering.

## Use cases

- Control the vehicle for autonomous driving.
- Relay commands from the operator.

## Requirement

- Support sending the control command to the vehicle.
- Ignore the control command depending on the control mode.
- Report the error as diagnostics in the following cases:
  - Topic rate is too low or too high.
  - An unacceptable target value is sent.
  - It fails to achieve the target value.

## Design

Topic rate: In Autoware 2.0.0, the controller sets the rate of this topic. [`autoware_trajectory_follower_node`](https://github.com/autowarefoundation/autoware_universe/blob/0.53.1/control/autoware_trajectory_follower_node/src/controller_node.cpp) calculates a command on a timer with the period `ctrl_period`. The [`trajectory_follower_node.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_universe_launch/autoware_control_config/config/trajectory_follower/trajectory_follower_node.param.yaml) file of `autoware_launch` sets `ctrl_period: 0.03`, which gives about 33 Hz. [`autoware_vehicle_cmd_gate`](https://github.com/autowarefoundation/autoware_universe/blob/0.53.1/control/autoware_vehicle_cmd_gate/src/vehicle_cmd_gate.cpp) publishes each command that it receives on this topic immediately, so this topic has the same rate. The gate also has a timer with `update_rate: 10.0` in [`vehicle_cmd_gate.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_universe_launch/autoware_control_config/config/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml). This timer publishes the gear, turn indicator, and hazard light commands. It publishes a stop command on this topic only when the system emergency heartbeat times out or when an external emergency stop is active.

Basis for the rate: The controllers use `ctrl_period` as their time step. For example, the MPC lateral controller rounds the steering delay to a whole number of control periods, and it limits the steering change in each period. The [`mpc.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_universe_launch/autoware_control_config/config/trajectory_follower/lateral/mpc.param.yaml) file sets the steering delay to 0.24 s (`input_delay`) and the steering time constant to 0.27 s (`vehicle_model_steer_tau`). The period of 0.03 s is much shorter than these values. As a result, the controller corrects the command several times while the steering responds. If you change `ctrl_period`, the controller behavior also changes.

Rate diagnostics: The `topic_state_monitor_control_command_control_cmd` node of [`autoware_topic_state_monitor`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/system/autoware_topic_state_monitor) monitors this topic. The [`topics.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_launch/config/system/component_state_monitor/topics.yaml) file sets `warn_rate: 5.0`, `error_rate: 1.0`, and `timeout: 1.0`. The status becomes a warning below 5 Hz. It becomes an error below 1 Hz or when no message arrives for 1 s. The [`control.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_launch/config/system/diagnostics/control.yaml) diagnostic graph reports this status as `/autoware/control/topic_rate_check/control_command`. These limits are much lower than the normal rate, so they find a controller that stopped or became very slow. The monitor does not find a rate that is too high.

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2026-01-21 | First release in the new format. |
