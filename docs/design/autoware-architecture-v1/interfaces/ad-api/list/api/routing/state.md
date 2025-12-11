---
title: /api/routing/state
status: v1.0.0
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/RouteState
  msg:
    - name: state
      text: A value of the route state.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the route state.
For details, see the [routing](../../../features/routing.md).
{% endblock %}
