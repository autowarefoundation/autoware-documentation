---
title: /api/remote/command/steering
status: not released
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/SteeringCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: steering_tire_angle
      text: Target steering tire angle [rad].
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Send steering command to this API.
To use this API, select the corresponding mode as described in [manual control](../../../../features/manual-control.md).
{% endblock %}
