---
title: /api/remote/command/accel
status: not released
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/AccelCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: accel
      text: target acceleration [m/s^2].
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Send acceleration command to this API.
This API is not available until a control mode is selected using {{ link_ad_api('/api/remote/control_mode/select') }}.
{% endblock %}
