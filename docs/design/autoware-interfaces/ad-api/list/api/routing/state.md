---
title: /api/routing/state
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/RouteState
  msg:
    - name: state
      text: A value of the route state.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the route state.
For details, see the [routing](../../../features/routing.md).
{% endblock %}
