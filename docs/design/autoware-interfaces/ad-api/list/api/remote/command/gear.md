---
title: /api/remote/command/gear
status: not released
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/GearCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: command
      text: Target gear status.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Send gear command to this API.
This API is not available until a control mode is selected using {{ link_ad_api('/api/remote/control_mode/select') }}.
{% endblock %}
