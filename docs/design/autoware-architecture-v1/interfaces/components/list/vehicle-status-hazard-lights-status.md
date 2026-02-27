---
last_updated: 2026-01-21
interface_type: topic
interface_name: /vehicle/status/hazard_lights_status
data_type_name: autoware_vehicle_msgs/msg/HazardLightsReport
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/HazardLightsReport.msg
rate: 10 or N/A
qos_reliability: reliable
qos_durability: volatile or transient_local
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Get the current hazard lights status of the vehicle. The status is `ENABLE` or `DISABLE`.
It is recommended to set QoS to transient_local and publish only when the status changes, but currently many implementations publish the status periodically.
Therefore, ensure consistency across the entire system.

Note that this status indicates the logical activation state of the hazard lights system (i.e., whether the function is active), not the instantaneous physical state of the light bulbs (i.e., whether the bulb is lit or unlit during a blinking cycle). Therefore, the status typically remains ENABLE continuously while the hazard lights are blinking.

## Message

The `stamp` field is the status received time or hardware time such as VCU. In the case of periodic publication, use the latest time, not the last status change.
For the `report` field, use the valid values listed above.

## Errors

Unknown status: If the vehicle interface cannot get the status due to connection loss, etc., the status is stopped and a diagnostic error is reported.

Invalid status: If the vehicle interface receives an undefined status, stop publishing status and report as diagnostics.

Hardware Fault: If the vehicle platform reports a sensor fault, a diagnostic error is reported.

## Support

This interface is required. If the vehicle does not have hazard lights, always treat it as `DISABLE`.

## Limitations

Logical state: This interface reports the logical activation state (e.g., stalk position or system state). It typically does not toggle ENABLE/DISABLE in sync with the physical blinking of the light bulbs.

## Use Cases

- Control the vehicle for autonomous driving.
- Display current hazard lights status to the operator.

## Requirement

- Support getting the current hazard lights status of the vehicle.
- Report the error as diagnostics if the status cannot be received or an unknown status is received.

## Design

In the early stages, turn indicators and hazard lights were managed as different states of the same interface, but were split because the states needed to be managed separately.

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2026-01-21 | First release in the new format. |
