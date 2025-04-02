---
title: /api/manual/local/command/velocity
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
Sends velocity command used in local operation mode.
To use this API, select the corresponding mode as described in [manual control](../../../../../features/manual-control.md).
{% endblock %}
