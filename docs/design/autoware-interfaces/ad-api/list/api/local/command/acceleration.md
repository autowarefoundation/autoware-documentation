---
title: /api/local/command/acceleration
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
Sends acceleration command used in local operation mode.
To use this API, select the corresponding mode as described in [manual control](../../../../features/manual-control.md).
{% endblock %}
