---
title: /api/manual/local/command/turn_indicators
status: not released
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/TurnIndicatorsCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: command
      text: Target turn indicators status.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Sends turn indicators command used in local operation mode.
To use this API, select the corresponding mode as described in [manual control](../../../../../features/manual-control.md).
{% endblock %}
