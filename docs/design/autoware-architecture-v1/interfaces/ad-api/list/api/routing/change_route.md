---
title: /api/routing/change_route
status: v1.5.0
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

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Same as {{ link_ad_api('/api/routing/set_route') }}, but change the route while driving.
This API only accepts the route when the route state is SET.
In any other state, set the route first or wait for the route change to complete.
{% endblock %}
