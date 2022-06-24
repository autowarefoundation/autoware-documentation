# /api/route/lanelet/notice

- Method: notification
- Type: [autoware_ad_api_msgs/msg/LaneletRouteNotice](../../../../types/autoware_ad_api_msgs/msg/lanelet_route_notice.md)

## Description

Get the route with the waypoint segments in lanelet format. It is empty if route is not set.

## Message

| Name  | Type                                  | Description                 |
| ----- | ------------------------------------- | --------------------------- |
| route | autoware_ad_api_msgs/msg/LaneletRoute | The route in lanelet format |
