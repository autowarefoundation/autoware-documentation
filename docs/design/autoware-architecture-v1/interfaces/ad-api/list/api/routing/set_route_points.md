---
title: /api/routing/set_route_points
status: v1.0.0
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/SetRoutePoints
  req:
    - name: header
      text: header for pose transformation
    - name: option
      text: route options, see [routing](../../../features/routing.md)
    - name: goal
      text: goal pose
    - name: waypoints
      text: waypoint poses
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Set the route with the waypoint poses. The current pose of the vehicle is the start of the route.
This API only accepts the route when the route state is UNSET. In any other state, clear the route first.
{% endblock %}
