---
interface:
  name: /api/routing/route
  type: autoware_adapi_v1_msgs/msg/Route
  method: notification
message:
  - name: header
    text: header for pose transformation
  - name: data
    text: The route in lanelet format
---

# /api/routing/route

{% include 'design/autoware-interfaces/templates/interface-header.jinja2' %}

## Description

Get the route with the waypoint segments in lanelet format. It is empty if route is not set.

## Message

{% include 'design/autoware-interfaces/templates/interface-message.jinja2' %}
