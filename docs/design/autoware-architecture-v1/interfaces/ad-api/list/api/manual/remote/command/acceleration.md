---
title: /api/manual/remote/command/acceleration
status: v1.8.0
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/AccelerationCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: acceleration
      text: Target acceleration [m/s^2].
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Sends acceleration command used in remote operation mode.
To use this API, select the corresponding mode as described in [manual control](../../../../../features/manual-control.md).
{% endblock %}
