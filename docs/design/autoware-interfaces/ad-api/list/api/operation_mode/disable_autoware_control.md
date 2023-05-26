---
title: /api/operation_mode/disable_autoware_control
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/ChangeOperationMode
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Disable vehicle control by Autoware. For details, see the [operation mode](./index.md).
This API fails if the vehicle does not support mode change by software.
{% endblock %}
