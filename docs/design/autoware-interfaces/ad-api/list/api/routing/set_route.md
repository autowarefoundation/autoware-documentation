---
title: /api/routing/set_route
status: v1.0.0
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/SetRoute
  req:
    - name: header
      text: header for pose transformation
    - name: goal
      text: goal pose
    - name: segments
      text: waypoint segments in lanelet format
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Set the route with the waypoint segments in lanelet format. If start pose is not specified, the current pose will be used.
This API only accepts the route when the route state is UNSET. In any other state, clear the route first.
{% endblock %}
