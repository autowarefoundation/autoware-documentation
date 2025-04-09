---
title: /api/manual/remote/control_mode/status
status: v1.8.0
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/ManualControlModeStatus
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the current manual operation mode.
{% endblock %}
