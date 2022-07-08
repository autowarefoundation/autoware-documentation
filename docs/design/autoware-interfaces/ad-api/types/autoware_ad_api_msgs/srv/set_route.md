# autoware_ad_api_msgs/srv/SetRoute

## Definition

```txt
std_msgs/Header header
geometry_msgs/Pose[<=1] start
geometry_msgs/Pose goal
autoware_ad_api_msgs/RouteSegment[] segments
---
autoware_ad_api_msgs/ResponseStatus status
```

## This type uses

- [autoware_ad_api_msgs/msg/ResponseStatus](../../autoware_ad_api_msgs/msg/response_status.md)
- [autoware_ad_api_msgs/msg/RouteSegment](../../autoware_ad_api_msgs/msg/route_segment.md)
