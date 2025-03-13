---
title: /api/vehicle/command/velocity
status: not released
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/VelocityCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: velocity
      text: Target velocity [m/s].
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
This is the target velocity that Autoware is sending to the vehicle.
{% endblock %}
