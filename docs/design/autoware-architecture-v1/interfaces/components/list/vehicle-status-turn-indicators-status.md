---
last_updated: 2026-01-21
interface_type: topic
interface_name: /vehicle/status/turn_indicators_status
data_type_name: autoware_vehicle_msgs/msg/TurnIndicatorsReport
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/TurnIndicatorsReport.msg
rate: 10 or N/A
qos_reliability: reliable
qos_durability: volatile or transient_local
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Get the current turn indicators status of the vehicle. The status is `ENABLE_RIGHT` or `ENABLE_LEFT`, or `DISABLE`.
It is recommended to set QoS to transient_local and publish only when the status changes, but currently many implementations publish the status periodically.
Therefore, ensure consistency across the entire system.

Note that this status indicates the logical activation state of the turn indicators system (i.e., whether the function is active), not the instantaneous physical state of the light bulbs (i.e., whether the bulb is lit or unlit during a blinking cycle). Therefore, the status typically remains ENABLE continuously while the turn indicators are blinking.

## Message

The `stamp` field is the status received time or hardware time such as VCU. In the case of periodic publication, use the latest time, not the last status change.
For the `report` field, use the valid values listed above.

## Errors

TBD: Check the relevant diagnostics.

## Support

This interface is required. If the vehicle does not have turn indicators, always treat it as `DISABLE`.

## Limitations

Hazard priority: If hazard lights are active, the vehicle hardware typically overrides the turn indicators. In such cases, this interface may report DISABLE or ENABLE, depending on the specific vehicle implementation, but the physical lights will be blinking as hazards.

Logical state: This interface reports the logical activation state (e.g., stalk position or system state). It typically does not toggle ENABLE/DISABLE in sync with the physical blinking of the light bulbs.

## Use Cases

- Control the vehicle for autonomous driving.
- Display current turn indicators status to the operator.

## Requirement

- Support getting the current turn indicators status of the vehicle.
- Report the error as diagnostics if the status cannot be received or an unknown status is received.

## Design

In the early stages, turn indicators and hazard lights were managed as different states of the same interface, but were split because the states needed to be managed separately.

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2026-01-21 | First release in the new format. |
