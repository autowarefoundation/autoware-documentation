---
title: /api/fail_safe/mrm_state
status: not released
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/MrmState
  msg:
    - name: state
      text: The state of MRM operation.
    - name: behavior
      text: The currently selected behavior of MRM.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the MRM state.
For details, see the [fail-safe](../../../features/fail-safe.md).
{% endblock %}
