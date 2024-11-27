---
title: /api/remote/command/pedal
status: not released
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/PedalCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: accel
      text: Target accelerator pedal ratio.
    - name: brake
      text: Target brake pedal ratio.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Send accelerator and brake pedal command to this API. The pedal value is the ratio with the maximum pedal depression being 1.0.
This API is not available until a control mode is selected using {{ link_ad_api('/api/remote/control_mode/select') }}.
{% endblock %}
