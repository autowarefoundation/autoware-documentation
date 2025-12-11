---
title: /api/fail_safe/mrm_state
status: v1.1.0
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/MrmState
  msg:
    - name: state
      text: The state of MRM operation.
    - name: behavior
      text: The currently selected behavior of MRM.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the MRM state.
For details, see the [fail-safe](../../../features/fail-safe.md).
{% endblock %}
