---
title: /api/manual/remote/command/pedals
status: v1.8.0
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/PedalsCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: throttle
      text: Target throttle pedal ratio.
    - name: brake
      text: Target brake pedal ratio.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Sends pedals command used in remote operation mode. The pedal value is the ratio with the maximum pedal depression being 1.0.
To use this API, select the corresponding mode as described in [manual control](../../../../../features/manual-control.md).
{% endblock %}
