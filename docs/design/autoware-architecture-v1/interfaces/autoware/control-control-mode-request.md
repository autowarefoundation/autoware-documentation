---
architecture: autoware components
interface_type: topic
interface_name: /control/control_mode_request
data_type: "[autoware_vehicle_msgs/srv/ControlModeCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/srv/ControlModeCommand.srv)"
updated: 2025-12-01
timeout: ---
endpoints:
  vehicle: srv
  control: cli
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/service.jinja2' %}

## Description

Send a change request of the control mode to the vehicle. The control mode manages whether the vehicle accepts commands from Autoware.

The following table shows which commands are accepted in each mode.

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

## Request

The `stamp` field is the request sent time. For the `mode` field, use the valid values listed above.

## Response

If the mode change was successful or not needed, the `success` field will be true, otherwise it will be false.

## Errors

If an unsupported or unknown command is requested, ignore it and return a response with the `success` field set to false.

## Support

This interface is required.
If the mode change is not possible through this interface, for example, only a hardware switch is supported, always return a failure response.

## Limitations

The mode change without using this interface may result in sudden changes in vehicle behavior.
In this case, switch when the command difference is small, such as when the vehicle is stopped.

## Use Cases

- Switch to manual driving.
- Switch to Autoware control when the vehicle is stopped.
- Switch to Autoware control during manual driving.

## Requirement

- If the vehicle does not support mode switching via Autoware, always return a failure response.
- If the vehicle supports mode switching via Autoware, accept MANUAL and AUTONOMOUS at least.

## Design

- Simply change the mode as requested.

## History

| Date       | Description |
| ---------- | ----------- |
| 2025-12-01 | Release.    |
