---
last_updated: 2026-02-27
interface_type: service
interface_name: /map/get_partial_pointcloud_map
data_type_name: autoware_map_msgs/srv/GetPartialPointCloudMap
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/srv/GetPartialPointCloudMap.srv
timeout: ---
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/service.jinja2' %}

## Description

## Service

## Errors

Load failure: If the map fails to load because it doesn't exist, is in the wrong format, etc., a failure response is returned and a diagnostic error is reported.

## Support

This interface may not be provided if entire map loading interface is provided.

## Limitations

Large data: If large maps are used, DDS configuration may be required. [See this page for details.](https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/)

## Use Cases

## Requirement

## Design

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2026-02-27 | First release in the new format. |
