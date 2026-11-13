---
title: /api/manual/remote/control_mode/status
status: v1.8.0
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/ManualControlModeStatus
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: mode
      text: The current manual control mode.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the current manual control mode as described in [manual control](../../../../../features/manual-control.md).
{% endblock %}
