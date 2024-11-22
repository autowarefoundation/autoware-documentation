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

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Clear the route. This API fails when the vehicle is using the route.
{% endblock %}
