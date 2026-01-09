---
architecture: autoware components
interface_type: topic
interface_name: /vehicle/status/control_mode
data_type: "[autoware_vehicle_msgs/msg/ControlModeReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/ControlModeReport.msg)"
rate: 10 or N/A
qos_reliability: reliable
qos_durability: volatile or transient_local
qos_depth: 1
last_updated: 2025-12-01
endpoints:
  vehicle: pub
  control: sub
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Get the current control mode status of the vehicle. The status are listed in the table below.
It is recommended to set QoS to transient_local and publish only when the status changes, but currently many implementations publish the status periodically.
Therefore, ensure consistency across the entire system.

Please note that ignoring a command does not mean stopping the vehicle.
The vehicle may drive in any behavior, by manual control interfaces such as the driver's seat.

- velocity group
  - /control/command/control_cmd (longitudinal field)
  - /control/command/gear_cmd
- steering group
  - /control/command/control_cmd (lateral field)
  - /control/command/turn_indicators_cmd
- others group
  - /control/command/hazard_lights_cmd
  - /vehicle/doors/command

| control mode             | velocity group | steering group | others group |
| ------------------------ | -------------- | -------------- | ------------ |
| AUTONOMOUS               | accept         | accept         | accept       |
| AUTONOMOUS_STEER_ONLY    | ignore         | accept         | accept       |
| AUTONOMOUS_VELOCITY_ONLY | accept         | ignore         | accept       |
| MANUAL                   | ignore         | ignore         | ignore       |
| DISENGAGED               | T.B.D.         | T.B.D.         | T.B.D.       |
| NO_COMMAND               | T.B.D.         | T.B.D.         | T.B.D.       |
| NOT_READY                | T.B.D.         | T.B.D.         | T.B.D.       |

## Message

The `stamp` field is the status received time or hardware time such as VCU. In the case of periodic publication, use the latest time, not the last status change.

For the `mode` field, use the valid values listed above.

## Errors

If the status cannot be received or an unknown status is received, stop publishing the topic and report the error as diagnostics.

## Support

Support for `MANUAL` and `AUTONOMOUS` modes is required.

## Limitations

- None.

## Use Cases

- Control the vehicle for autonomous driving.
- Display current control mode status to the operator.

## Requirement

- Support getting the current control mode status of the vehicle.

## Prerequisites

- None.

## Design

- None.

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2025-12-01 | First release in the new format. |
