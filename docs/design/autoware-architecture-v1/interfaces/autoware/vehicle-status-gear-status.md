---
architecture: autoware components
interface_type: topic
interface_name: /vehicle/status/gear_status
data_type: "[autoware_vehicle_msgs/msg/GearReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/GearReport.msg)"
rate: 10 or N/A
qos_reliability: reliable
qos_durability: volatile or transient_local
qos_depth: 1
last_updated: 2025-12-01
endpoints:
  vehicle: pub
  control: sub
  adapi: sub
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Get the current gear status of the vehicle. The gear status are listed in the table below.
It is recommended to set QoS to transient_local and publish only when the status changes, but currently many implementations publish the status periodically.

| Value   | Description                                                                                 |
| ------- | ------------------------------------------------------------------------------------------- |
| PARKING | The engine or motor is disconnected from the tires and the stopping mechanism is activated. |
| NEUTRAL | The engine or motor is disconnected from the tires.                                         |
| DRIVE   | The engine or motor is connected to the tires in the forward direction.                     |
| REVERSE | The engine or motor is connected to the tires in the backwoard direction.                   |

## Message

The `stamp` field is the time when the status was obtained from the vehicle. In the case of periodic publication, use the latest time, not the last status changed.

For the `report` field, use the valid gear values listed above. `NONE` can be used as an invalid value to indicate not received, but will never be sent as a topic.
Values ​​such as `LOW` and `DRIVE_2` ​​can be used if the vehicle has its own special gear types, but a dedicated implementation is required to handle this.

NOTE: Maybe we need an extension that separates the actual gear status and the abstracted gear status.

## Errors

If vehicle status cannot be obtained, stop publishing the topic and send the error as diagnostics.

## Support

This interface is required. If the vehicle does not have gears, simulate the gear behavior.

## Limitations

None.

## Use Cases

- Use for autonomous vehicle control.
- Display current gear status to the operator.

## Requirement

- Support obtaining the current vehicle gear status.
- Support vehicle-specific gear status if necessary.

## Prerequisites

None.

## Design

ギアが存在する車両については、取得したギア状態を送信する。
ギアがない車両については、処理の一般化のためにドライバでギアを再現する。
細かくギアが制御できる車両のためにユーザー定義領域を確保する？

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2025-12-01 | First release in the new format. |
