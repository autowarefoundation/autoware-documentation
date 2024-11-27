---
title: /api/remote/command/steer
status: not released
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/SteerCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: steer
      text: Target steering tire angle [rad].
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Send steering command to this API.
This API is not available until a control mode is selected using {{ link_ad_api('/api/remote/control_mode/select') }}.
{% endblock %}
