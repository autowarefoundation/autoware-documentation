---
title: /api/remote/command/hazard_lights
status: not released
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/HazardLightsCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: command
      text: Target hazard lights status.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Send hazard lights command to this API.
This API is not available until a control mode is selected using {{ link_ad_api('/api/remote/control_mode/select') }}.
{% endblock %}
