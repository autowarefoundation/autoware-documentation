---
title: /api/vehicle/command/acceleration
status: not released
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/AccelerationCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: acceleration
      text: Target acceleration [m/s^2].
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
This is the target acceleration that Autoware is sending to the vehicle.
{% endblock %}
