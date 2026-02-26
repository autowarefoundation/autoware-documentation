---
last_updated: 2026-01-21
interface_type: topic
interface_name: /vehicle/status/velocity_status
data_type_name: autoware_vehicle_msgs/msg/VelocityReport
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/VelocityReport.msg
rate: 10
qos_reliability: reliable
qos_durability: volatile
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Get the current velocity status of the vehicle.

## Message

See the [message definition]({{ data_type_link }}) for details.

## Errors

Unknown status: If the vehicle interface cannot get the status due to connection loss, etc., the status is stopped and a diagnostic error is reported.

Hardware Fault: If the vehicle platform reports a sensor fault, a diagnostic error is reported.

## Support

This interface is required.

## Limitations

T.B.C.

## Use Cases

- Control the vehicle for autonomous driving.
- Display current velocity status to the operator.

## Requirement

- Support getting the current velocity status of the vehicle.
- Report the error as diagnostics if the status cannot be received or an unknown status is received.

## Design

Coordinate system & sign convention: The interface follows the standard vehicle coordinate system (ISO 8855 / ROS REP-103).

- Longitudinal velocity: Positive (+) indicates **forward** motion. Negative (-) indicates **backward** motion.
- Lateral velocity: Positive (+) indicates motion to the **left**. Negative (-) indicates motion to the **right**.
- Heading rate: Positive (+) indicates **counter-clockwise** rotation (left turn).

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2026-01-21 | First release in the new format. |
