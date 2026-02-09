---
last_updated: 2026-01-21
interface_type: topic
interface_name: /control/command/hazard_lights_cmd
data_type_name: autoware_vehicle_msgs/msg/HazardLightsCommand
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/HazardLightsCommand.msg
rate: 10 or N/A
qos_reliability: reliable
qos_durability: volatile or transient_local
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Send the hazard lights change command to the vehicle. The command is `ENABLE` or `DISABLE`.
It is recommended to set QoS to transient_local and publish only when the command changes, but currently many implementations publish the command periodically.
Therefore, ensure consistency across the entire system.

## Message

The `stamp` field is the command sent time. In the case of periodic publication, use the latest time, not the last command change.
For the `command` field, use the valid values listed above. The `NO_COMMAND` value can be used internally by programs, but will never be sent as a topic.

## Errors

TBD: Check the relevant diagnostics.

## Support

This interface is required. If the vehicle does not have hazard lights, always ignore the command.

## Limitations

None.

## Use Cases

- Control the vehicle for autonomous driving.
- Relay commands from the operator.

## Requirement

- Support sending the hazard lights command to the vehicle.
- Report the error as diagnostics if an unsupported or unknown command is sent.

## Design

Separation from turn indicators: Hazard lights and turn indicators are defined in separate interfaces to allow independent state management. For example, an autonomous stack might request "Turn Left" for routing, but a safety system might simultaneously request "Hazard Enable" due to an emergency. This simplifies the logic because an autonomous stack and safety system no longer have to worry about their states being overwritten by each other's commands.

Priority logic: The vehicle interface must prioritize HazardLightsCommand.ENABLE over any TurnIndicatorsCommand. This ensures safety signals (hazards) always take precedence over navigation signals.

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2026-01-21 | First release in the new format. |
