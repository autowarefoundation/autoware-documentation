---
architecture: autoware components
interface_type: topic
interface_name: /vehicle/status/steering_status
data_type: "[autoware_vehicle_msgs/msg/SteeringReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/SteeringReport.msg)"
updated: 2025-12-01
rate: 10
qos_reliability: reliable
qos_durability: volatile
qos_depth: 1
endpoints:
  localization: pub
  planning: sub
  perception: sub
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Get the current steering status of the vehicle.

## Message

See the message definition: {{ data_type }}.

## Errors

If the status cannot be received or an unknown status is received, stop publishing the topic and report the error as diagnostics.

## Support

This interface is required.

## Limitations

- None.

## Use Cases

- Control the vehicle for autonomous driving.
- Display current steering status to the operator.

## Requirement

- Support getting the current steering status of the vehicle.

## Design

None.

## History

| Date       | Description |
| ---------- | ----------- |
| 2025-12-01 | Release.    |
