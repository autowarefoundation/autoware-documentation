---
title: /api/routing/clear_route
status: v1.0.0
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/ClearRoute
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Clear the route. This API fails when the vehicle is using the route.
This API only guarantees that a request has been accepted.
Use [the state API](./state.md) to verify whether the request has been reflected.
{% endblock %}
