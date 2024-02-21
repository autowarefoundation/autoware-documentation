---
title: /api/fail_safe/rti_state
status: not released
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/RtiState
  msg:
    - name: state
      text: The state of RTI.
    - name: message
      text: The message such as reasons for RTI.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the RTI state.
For details, see the [fail-safe](../../../features/fail-safe.md).
{% endblock %}
