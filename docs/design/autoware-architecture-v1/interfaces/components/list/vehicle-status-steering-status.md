---
last_updated: 2026-01-21
interface_type: topic
interface_name: /vehicle/status/steering_status
data_type_name: autoware_vehicle_msgs/msg/SteeringReport
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/SteeringReport.msg
rate: 10
qos_reliability: reliable
qos_durability: volatile
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Get the current steering status of the vehicle.

## Message

See the [message definition]({{ data_type_link }}) for details.

## Errors

TBD: Check the relevant diagnostics.

## Support

This interface is required.

## Limitations

None.

## Use Cases

- Control the vehicle for autonomous driving.
- Display current steering status to the operator.

## Requirement

- Support getting the current steering status of the vehicle.
- Report the error as diagnostics if the status cannot be received or an unknown status is received.

## Design

None.

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2026-01-21 | First release in the new format. |
