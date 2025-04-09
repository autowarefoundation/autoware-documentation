---
title: /api/manual/remote/control_mode/select
status: v1.8.0
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/SelectManualControlMode
  req:
    - name: mode
      text: The manual control mode to be used.
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Selects the manual control mode as described in [manual control](../../../../../features/manual-control.md).
This API fails while [operation mode](../../../../../features/operation_mode.md) is remote.
{% endblock %}
