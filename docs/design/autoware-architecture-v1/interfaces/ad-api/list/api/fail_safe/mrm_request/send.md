---
title: /api/fail_safe/mrm_request/send
status: v1.9.0
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/SendMrmRequest
  req:
    - name: request.sender
      text: The sender name of the MRM request.
    - name: request.strategy
      text: The strategy of the MRM request.
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Send the MRM request.
For details, see the [fail-safe](../../../../features/fail-safe.md).
{% endblock %}
