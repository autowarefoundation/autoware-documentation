# /api/routing/set_route_points

- Method: function call
- Type: [autoware_ad_api_msgs/srv/SetRoutePoints](../../../types/autoware_ad_api_msgs/srv/set_route_points.md)

## Description

Set the route with the waypoint poses. If start pose is not specified, the current pose will be used.

## Request

| Name      | Type                        | Description                    |
| --------- | --------------------------- | ------------------------------ |
| header    | std_msgs/msg/Header         | header for pose transformation |
| start     | geometry_msgs/msg/Pose[<=1] | start pose                     |
| goal      | geometry_msgs/msg/Pose      | goal pose                      |
| waypoints | geometry_msgs/msg/Pose[]    | waypoint poses                 |

## Response

| Name   | Type                                    | Description     |
| ------ | --------------------------------------- | --------------- |
| status | autoware_ad_api_msgs/msg/ResponseStatus | response status |
