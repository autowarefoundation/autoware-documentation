---
title: /api/fail_safe/rti_state
status: not released
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/RtiState
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: request
      text: The flag for whether RTI is requested.
    - name: message
      text: The message such as reasons for RTI.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the RTI state. The default implementation does not provide this API yet.
For details, see the [fail-safe](../../../features/fail-safe.md).
{% endblock %}
