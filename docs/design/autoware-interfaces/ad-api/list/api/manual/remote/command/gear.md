---
title: /api/manual/remote/command/gear
status: v1.8.0
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/GearCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: command
      text: Target gear status.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Sends gear command used in remote operation mode.
To use this API, select the corresponding mode as described in [manual control](../../../../../features/manual-control.md).
{% endblock %}
