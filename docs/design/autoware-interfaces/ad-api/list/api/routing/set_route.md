---
title: /api/routing/set_route
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
{% endblock %}
