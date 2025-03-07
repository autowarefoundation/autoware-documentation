---
title: /api/remote/command/pedals
status: not released
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/PedalsCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: accelerator
      text: Target accelerator pedal ratio.
    - name: brake
      text: Target brake pedal ratio.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Sends pedals command used in remote operation mode. The pedal value is the ratio with the maximum pedal depression being 1.0.
To use this API, select the corresponding mode as described in [manual control](../../../../features/manual-control.md).
{% endblock %}
