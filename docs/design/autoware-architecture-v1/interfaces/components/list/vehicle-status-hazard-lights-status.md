---
last_updated: 2026-01-21
interface_type: topic
interface_name: /vehicle/status/hazard_lights_status
data_type_name: autoware_vehicle_msgs/msg/HazardLightsReport
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/HazardLightsReport.msg
rate: 10 or N/A
qos_reliability: reliable
qos_durability: volatile
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Get the current hazard lights status of the vehicle. The status is `ENABLE` or `DISABLE`.

The [`autoware_component_interface_specs`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/common/autoware_component_interface_specs/include/autoware/component_interface_specs/vehicle.hpp) package defines this topic with `volatile` durability. The vehicle interface publishes this topic. Known publishers, for example [`pacmod_interface`](https://github.com/tier4/pacmod_interface/blob/main/pacmod_interface/src/pacmod_interface/pacmod_interface.cpp) and [`autoware_simple_planning_simulator`](https://github.com/autowarefoundation/autoware_simple_planning_simulator/blob/1.1.0/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp), publish the status continuously with `volatile` durability, not only when the status changes. The design intent is different: use `transient_local` durability and publish only when the status changes. Make sure that the publishers and the subscribers in your system use compatible QoS settings.

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

## Use cases

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
