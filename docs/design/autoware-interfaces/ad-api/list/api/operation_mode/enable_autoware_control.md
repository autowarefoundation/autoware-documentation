---
title: /api/operation_mode/enable_autoware_control
status: v1.0.0
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/ChangeOperationMode
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Enable vehicle control by Autoware.
For details, see the [operation mode](../../../features/operation_mode.md).
This API fails if the vehicle does not support mode change by software.
{% endblock %}
