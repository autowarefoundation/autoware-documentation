---
architecture: autoware components
interface_type: topic
interface_name: /control/command/hazard_lights_cmd
data_type: "[autoware_vehicle_msgs/msg/HazardLightsCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/HazardLightsCommand.msg)"
rate: 10 or N/A
qos_reliability: reliable
qos_durability: volatile or transient_local
qos_depth: 1
last_updated: 2025-12-01
endpoints:
  localization: pub
  planning: sub
  perception: sub
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

If an unsupported or unknown command is sent, ignore it and report the error as diagnostics.

## Support

This interface is required. If the vehicle does not have hazard lights, always ignore the command.

## Limitations

- None.

## Use Cases

- Control the vehicle for autonomous driving.
- Relay commands from the operator.

## Requirement

- Support sending the hazard lights command to the vehicle.

## Prerequisites

- None.

## Design

In the early stages, turn indicators and hazard lights were managed as different states of the same interface, but were split because the states needed to be managed separately.

## History

| Date       | Description |
| ---------- | ----------- |
| 2025-12-01 | Release.    |
