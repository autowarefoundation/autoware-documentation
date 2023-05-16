---
title: /api/routing/route
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/Route
  msg:
    - name: header
      text: header for pose transformation
    - name: data
      text: The route in lanelet format
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the route with the waypoint segments in lanelet format. It is empty if route is not set.
{% endblock %}
