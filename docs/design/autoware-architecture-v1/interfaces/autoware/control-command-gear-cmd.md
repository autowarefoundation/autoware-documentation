---
architecture: autoware components
interface_type: topic
interface_name: /control/command/gear_cmd
data_type: "[autoware_vehicle_msgs/msg/GearCommand](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/GearCommand.msg)"
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

Send the gear change command to the vehicle. The commands are listed in the table below.
It is recommended to set QoS to transient_local and publish only when the command changes, but currently many implementations publish the command periodically.
Therefore, ensure consistency across the entire system.

| Value   | Description                                                                                 |
| ------- | ------------------------------------------------------------------------------------------- |
| PARKING | The engine or motor is disconnected from the tires and the stopping mechanism is activated. |
| NEUTRAL | The engine or motor is disconnected from the tires.                                         |
| DRIVE   | The engine or motor is connected to the tires in the forward direction.                     |
| REVERSE | The engine or motor is connected to the tires in the backwoard direction.                   |

## Message

The `stamp` field is the command sent time. In the case of periodic publication, use the latest time, not the last command change.

For the `command` field, use the valid values listed above. The `NONE` value can be used internally by programs, but will never be sent as a topic.
Values ​​such as `LOW` and `DRIVE_2` ​​can be used if the vehicle has its own special gear types, but a dedicated implementation is required to handle this.

## Errors

If an unsupported or unknown command is sent, ignore it and report the error as diagnostics.

## Support

This interface is required. If the vehicle does not have gears, simulate the gear behavior.

## Limitations

- None.

## Use Cases

- Control the vehicle for autonomous driving.
- Relay commands from the operator.

## Requirement

- Support sending the gear command to the vehicle.
- Support vehicle-specific gear status if necessary.

## Prerequisites

- None.

## Design

- Support four typical gear types: PARKING, NEUTRAL, DRIVE, and REVERSE.
- Unused values ​​can be used for special gear types.
- Simulate gear if necessary to increase reusability.

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2025-12-01 | First release in the new format. |
