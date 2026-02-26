---
last_updated: 2026-01-21
architecture: autoware components
interface_type: topic
interface_name: /control/command/turn_indicators_cmd
data_type_name: autoware_vehicle_msgs/msg/TurnIndicatorsCommand
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/TurnIndicatorsCommand.msg
rate: 10 or N/A
qos_reliability: reliable
qos_durability: volatile or transient_local
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Send the turn indicators change command to the vehicle. The command is `ENABLE_RIGHT` or `ENABLE_LEFT`, or `DISABLE`.
It is recommended to set QoS to transient_local and publish only when the command changes, but currently many implementations publish the command periodically.
Therefore, ensure consistency across the entire system.

## Message

The `stamp` field is the command sent time. In the case of periodic publication, use the latest time, not the last command change.
For the `command` field, use the valid values listed above. The `NO_COMMAND` value can be used internally by programs, but will never be sent as a topic.

## Errors

Invalid command: If the vehicle interface receives an undefined command, it is ignored and a diagnostic error is reported.

## Support

This interface is required. If the vehicle does not have turn indicators, always ignore the command.

## Limitations

- Regarding the actual lighting status of the vehicle, hazard lights may have priority.

## Use Cases

- Control the vehicle for autonomous driving.
- Relay commands from the operator.

## Requirement

- Support sending the turn indicators command to the vehicle.
- Report the error as diagnostics if an unsupported or unknown command is sent.

## Design

Separation from hazard lights: Hazard lights and turn indicators are defined in separate interfaces to allow independent state management. For example, an autonomous stack might request "Turn Left" for routing, but a safety system might simultaneously request "Hazard Enable" due to an emergency. This simplifies the logic because an autonomous stack and safety system no longer have to worry about their states being overwritten by each other's commands.

Exclusive state: Unlike hazard lights, turn indicators are directional and mutually exclusive (Left vs. Right). Therefore, they are defined as an Enum in a single command interface rather than separate boolean flags.

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2026-01-21 | First release in the new format. |
