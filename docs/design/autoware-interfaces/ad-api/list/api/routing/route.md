# /api/routing/route

- Method: notification
- Type: [autoware_ad_api_msgs/msg/RouteOptional](../../../types/autoware_ad_api_msgs/msg/route_optional.md)

## Description

Get the route with the waypoint segments in lanelet format. It is empty if route is not set.

## Message

| Name  | Type                           | Description                 |
| ----- | ------------------------------ | --------------------------- |
| route | autoware_ad_api_msgs/msg/Route | The route in lanelet format |
