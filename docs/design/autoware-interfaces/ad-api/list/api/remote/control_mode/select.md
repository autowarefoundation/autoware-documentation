---
title: /api/remote/control_mode/select
status: not released
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
This API selects the type of command to use.
This API cannot be used while [operation mode](../../../../features/operation_mode.md) is remote.
{% endblock %}
