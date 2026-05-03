---
last_updated: 2026-02-27
interface_type: topic
interface_name: /map/vector_map
data_type_name: autoware_map_msgs/msg/LaneletMapBin
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/msg/LaneletMapBin.msg
rate: N/A
qos_reliability: reliable
qos_durability: transient_local
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Provide the entire lanelet2 map for other components.

## Message

## Errors

Load failure: If the map fails to load because it doesn't exist, is in the wrong format, etc., the map is not published and a diagnostic error is reported.

## Support

This interface may not be provided if partial map loading interface is provided.

## Limitations

Large data: If large maps are used, DDS configuration may be required. [See this page for details.](https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/)

## Use Cases

## Requirement

## Design

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2026-02-27 | First release in the new format. |
