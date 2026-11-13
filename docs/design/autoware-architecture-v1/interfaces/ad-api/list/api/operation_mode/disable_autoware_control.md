---
title: /api/operation_mode/disable_autoware_control
status: v1.0.0
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/ChangeOperationMode
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Disable vehicle control by Autoware.
A successful response only means that Autoware accepted the request.
The change is complete when `is_autoware_control_enabled` in {{ link_ad_api('/api/operation_mode/state') }} is false.
For details, see the [operation mode](../../../features/operation_mode.md).
This API fails if the vehicle does not support mode change by software.
{% endblock %}
