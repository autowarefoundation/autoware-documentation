---
title: /api/control/command/pedals
status: v1.9.0
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

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
This is the target pedals that Autoware is sending to the vehicle.
The pedal value is the ratio with the maximum pedal depression being 1.0.
This API is not supported if the vehicle is not controlled by pedals.
{% endblock %}
