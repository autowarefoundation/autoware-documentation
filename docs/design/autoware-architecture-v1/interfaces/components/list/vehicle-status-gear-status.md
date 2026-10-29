---
last_updated: 2026-01-21
interface_type: topic
interface_name: /vehicle/status/gear_status
data_type_name: autoware_vehicle_msgs/msg/GearReport
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/GearReport.msg
rate: 10 or N/A
qos_reliability: reliable
qos_durability: volatile
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Get the current gear status of the vehicle. The statuses are listed in the table below.

The [`autoware_component_interface_specs`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/common/autoware_component_interface_specs/include/autoware/component_interface_specs/vehicle.hpp) package defines this topic with `volatile` durability. The vehicle interface publishes this topic. Known publishers, for example [`pacmod_interface`](https://github.com/tier4/pacmod_interface/blob/main/pacmod_interface/src/pacmod_interface/pacmod_interface.cpp) and [`autoware_simple_planning_simulator`](https://github.com/autowarefoundation/autoware_simple_planning_simulator/blob/1.1.0/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp), publish the status continuously with `volatile` durability, not only when the status changes. The design intent is different: use `transient_local` durability and publish only when the status changes. Make sure that the publishers and the subscribers in your system use compatible QoS settings.

| Value   | Description                                                                                 |
| ------- | ------------------------------------------------------------------------------------------- |
| PARK    | The engine or motor is disconnected from the tires and the stopping mechanism is activated. |
| NEUTRAL | The engine or motor is disconnected from the tires.                                         |
| DRIVE   | The engine or motor is connected to the tires in the forward direction.                     |
| REVERSE | The engine or motor is connected to the tires in the backward direction.                    |

## Message

The `stamp` field is the status received time or hardware time such as VCU. In the case of periodic publication, use the latest time, not the last status change.
For the `report` field, use the valid values listed above. The `NONE` value can be used internally by programs, but will never be sent as a topic.
Values such as `LOW` and `DRIVE_2` can be used if the vehicle has its own special gear types, but a dedicated implementation is required to handle this.

## Errors

Unknown status: If the vehicle interface cannot get the status due to connection loss, etc., the status is stopped and a diagnostic error is reported.

Invalid status: If the vehicle interface receives an undefined status, stop publishing status and report as diagnostics.

Hardware Fault: If the vehicle platform reports a sensor fault, a diagnostic error is reported.

## Support

This interface is required. If the vehicle does not have gear, simulate the gear behavior.

## Limitations

Latency: Gear shifting is a mechanical process that takes time (typically 0.5s to 2.0s). The status reported here reflects the actual engaged gear, so there will be a delay after sending a command. Autoware must handle this transition period.

## Use Cases

- Control the vehicle for autonomous driving.
- Display current gear status to the operator.

## Requirement

- Support getting the current gear status of the vehicle.
- Support vehicle-specific gear status if necessary.

## Design

- Support four typical gear types: PARK, NEUTRAL, DRIVE, and REVERSE.
- Unused values can be used for special gear types.
- Simulate gear if necessary to increase reusability.

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2026-01-21 | First release in the new format. |
