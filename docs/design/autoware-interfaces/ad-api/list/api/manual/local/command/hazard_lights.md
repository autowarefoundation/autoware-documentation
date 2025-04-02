---
title: /api/manual/local/command/hazard_lights
status: not released
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/HazardLightsCommand
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: command
      text: Target hazard lights status.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Sends hazard lights command used in local operation mode.
To use this API, select the corresponding mode as described in [manual control](../../../../../features/manual-control.md).
{% endblock %}
