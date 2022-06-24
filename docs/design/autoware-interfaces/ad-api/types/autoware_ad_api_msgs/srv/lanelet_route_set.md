# autoware_ad_api_msgs/srv/LaneletRouteSet

## Definition

```txt
std_msgs/Header header
geometry_msgs/Pose[<=1] start
geometry_msgs/Pose goal
autoware_ad_api_msgs/LaneletSegment[] segments
---
autoware_ad_api_msgs/ResponseStatus status
```

## This type uses

- [autoware_ad_api_msgs/msg/LaneletSegment](../../autoware_ad_api_msgs/msg/lanelet_segment.md)
- [autoware_ad_api_msgs/msg/ResponseStatus](../../autoware_ad_api_msgs/msg/response_status.md)
