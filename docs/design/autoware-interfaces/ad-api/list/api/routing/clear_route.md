---
title: /api/routing/clear_route
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/ClearRoute
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Clear the route.
{% endblock %}
