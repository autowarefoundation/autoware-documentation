---
interface:
  name: /api/routing/set_route_points
  type: autoware_adapi_v1_msgs/srv/SetRoutePoints
  method: function call
request:
  - name: header
    text: header for pose transformation
  - name: goal
    text: goal pose
  - name: waypoints
    text: waypoint poses
response:
  - name: status
    text: response status
---

# /api/routing/set_route_points

{% include 'design/autoware-interfaces/templates/interface-header.jinja2' %}

## Description

Set the route with the waypoint poses. If start pose is not specified, the current pose will be used.

## Request

{% include 'design/autoware-interfaces/templates/interface-request.jinja2' %}

## Response

{% include 'design/autoware-interfaces/templates/interface-response.jinja2' %}
