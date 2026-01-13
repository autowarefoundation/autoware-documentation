---
architecture: autoware components
interface_type: topic
interface_name: /vehicle/status/hazard_lights_status
data_type: "[autoware_vehicle_msgs/msg/HazardLightsReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/HazardLightsReport.msg)"
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

Get the current hazard lights status of the vehicle. The status is `ENABLE` or `DISABLE`.
It is recommended to set QoS to transient_local and publish only when the status changes, but currently many implementations publish the status periodically.
Therefore, ensure consistency across the entire system.

## Message

The `stamp` field is the status received time or hardware time such as VCU. In the case of periodic publication, use the latest time, not the last status change.
For the `report` field, use the valid values listed above.

## Errors

If the status cannot be received or an unknown status is received, stop publishing the topic and report the error as diagnostics.

## Support

This interface is required. If the vehicle does not have hazard lights, always treat it as `DISABLE`.

## Limitations

- None.

## Use Cases

- Control the vehicle for autonomous driving.
- Display current hazard lights status to the operator.

## Requirement

- Support getting the current hazard lights status of the vehicle.

## Prerequisites

- None.

## Design

In the early stages, turn indicators and hazard lights were managed as different states of the same interface, but were split because the states needed to be managed separately.

## History

| Date       | Description |
| ---------- | ----------- |
| 2025-12-01 | Release.    |
