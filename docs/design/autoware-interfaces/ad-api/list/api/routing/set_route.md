# /api/routing/set_route

- Method: function call
- Type: [autoware_ad_api_msgs/srv/SetRoute](../../../types/autoware_ad_api_msgs/srv/set_route.md)

## Description

Set the route with the waypoint segments in lanelet format. If start pose is not specified, the current pose will be used.

## Request

| Name     | Type                                  | Description                         |
| -------- | ------------------------------------- | ----------------------------------- |
| header   | std_msgs/msg/Header                   | header for pose transformation      |
| start    | geometry_msgs/msg/Pose                | start pose                          |
| goal     | geometry_msgs/msg/Pose                | goal pose                           |
| segments | autoware_ad_api_msgs/msg/RouteSegment | waypoint segments in lanelet format |

## Response

| Name   | Type                                    | Description     |
| ------ | --------------------------------------- | --------------- |
| status | autoware_ad_api_msgs/msg/ResponseStatus | response status |
