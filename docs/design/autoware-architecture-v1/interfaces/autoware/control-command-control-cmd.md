---
architecture: autoware components
interface_type: topic
interface_name: /control/command/control_cmd
data_type: "[autoware_control_msgs/msg/Control](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Control.msg)"
rate: 33
qos_reliability: reliable
qos_durability: volatile
qos_depth: 1
last_updated: 2025-12-01
endpoints:
  control: pub
  vehicle: sub
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

[ackermann-kinematic-model]: ../../../../tutorials/integrating-autoware/creating-vehicle-interface-package/ackermann-kinematic-model.md

## Message

For details about the message, [see the readme of autoware_control_msgs](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/README.md).

The velocity, acceleration, and jerk values ​​at a given time must be planned in advance and must be consistent, which is typically done by the planning/control component.
If these values ​​are inconsistent, which value takes priority depends on the vehicle implementation.

Sending the latest command will invalidate previous commands, which means that commands from different time stamps are not kept as a time series.
Only the latest command, not the latest time stamp command, is valid.

## Errors

- If an unacceptable target value is sent, report the error as diagnostics.
- If it fails to achieve the target value, report the error as diagnostics.

## Support

This interface is required. If there is only a vehicle-specific interface, provide a converter.
For vehicles controlled by typical accelerator and brake pedals, consider using the [autoware_raw_vehicle_cmd_converter](https://github.com/autowarefoundation/autoware_universe/tree/main/vehicle/autoware_raw_vehicle_cmd_converter).

## Limitations

- None.

## Requirement

- Support sending the control command to the vehicle.
- Ignore the control command depending on the control mode.

## Design

TODO: Explanation of the basis for setting the topic frequency.

## History

| Date       | Description |
| ---------- | ----------- |
| 2025-12-01 | Release.    |
