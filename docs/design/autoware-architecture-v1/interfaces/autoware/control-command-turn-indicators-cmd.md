---
architecture: autoware components
interface_type: topic
interface_name: /control/command/turn_indicators_cmd
data_type: "[autoware_vehicle_msgs/msg/TurnIndicatorsCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/TurnIndicatorsCommand.msg)"
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

Send the turn indicators change command to the vehicle. The command is `ENABLE_RIGHT` or `ENABLE_LEFT`, or `DISABLE`.
It is recommended to set QoS to transient_local and publish only when the command changes, but currently many implementations publish the command periodically.
Therefore, ensure consistency across the entire system.

## Message

The `stamp` field is the command sent time. In the case of periodic publication, use the latest time, not the last command change.

For the `command` field, use the valid values listed above. The `NO_COMMAND` value can be used internally by programs, but will never be sent as a topic.

## Errors

If an unsupported or unknown command is sent, ignore it and report the error as diagnostics.

## Support

This interface is required. If the vehicle does not have turn indicators, always ignore the command.

## Limitations

- Regarding the actual lighting status of the vehicle, hazard lights may have priority.

## Use Cases

- Control the vehicle for autonomous driving.
- Relay commands from the operator.

## Requirement

- Support sending the turn indicators command to the vehicle.

## Prerequisites

- None.

## Design

In the early stages, turn indicators and hazard lights were managed as different states of the same interface, but were split because the states needed to be managed separately.

## History

| Date       | Description |
| ---------- | ----------- |
| 2025-12-01 | Release.    |
