---
last_updated: 2026-01-21
interface_type: topic
interface_name: /vehicle/status/control_mode
data_type_name: autoware_vehicle_msgs/msg/ControlModeReport
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/ControlModeReport.msg
rate: 10 or N/A
qos_reliability: reliable
qos_durability: volatile
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Get the current control mode status of the vehicle. The status are listed in the table below.

The [`autoware_component_interface_specs`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/common/autoware_component_interface_specs/include/autoware/component_interface_specs/vehicle.hpp) package defines this topic with `volatile` durability. The vehicle interface publishes this topic. Known publishers, for example [`pacmod_interface`](https://github.com/tier4/pacmod_interface/blob/main/pacmod_interface/src/pacmod_interface/pacmod_interface.cpp) and [`autoware_simple_planning_simulator`](https://github.com/autowarefoundation/autoware_simple_planning_simulator/blob/1.1.0/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp), publish the status continuously with `volatile` durability, not only when the status changes. The design intent is different: use `transient_local` durability and publish only when the status changes. Make sure that the publishers and the subscribers in your system use compatible QoS settings.

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

Unknown status: If the vehicle interface cannot get the status due to connection loss, etc., the status is stopped and a diagnostic error is reported.

Invalid status: If the vehicle interface receives an undefined status, stop publishing status and report as diagnostics.

## Support

Support for `MANUAL` and `AUTONOMOUS` modes is required.

## Limitations

Latency: There is an inevitable latency between sending a mode change request and this interface updating to the new state. Autoware must handle this transition period.

Override detection: Some vehicle platforms do not explicitly report override activation. In such cases, the vehicle interface calculates this state by comparing command and feedback, which may introduce detection delay.

## Use Cases

- Control the vehicle for autonomous driving.
- Display current control mode status to the operator.

## Requirement

- Support getting the current control mode status of the vehicle.
- Report the error as diagnostics if the status cannot be received or an unknown status is received.

## Design

None.

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2026-01-21 | First release in the new format. |
