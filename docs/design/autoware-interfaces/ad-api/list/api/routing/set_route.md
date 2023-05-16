---
interface:
  name: /api/routing/set_route
  type: autoware_adapi_v1_msgs/srv/SetRoute
  method: function call
request:
  - name: header
    text: header for pose transformation
  - name: goal
    text: goal pose
  - name: segments
    text: waypoint segments in lanelet format
response:
  - name: status
    text: response status
---

# /api/routing/set_route

{% include 'design/autoware-interfaces/templates/interface-header.jinja2' %}

## Description

Set the route with the waypoint segments in lanelet format. If start pose is not specified, the current pose will be used.

## Request

{% include 'design/autoware-interfaces/templates/interface-request.jinja2' %}

## Response

{% include 'design/autoware-interfaces/templates/interface-response.jinja2' %}
