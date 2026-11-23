---
title: /api/operation_mode/change_to_autonomous
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
Change the operation mode to autonomous.
A successful response only means that Autoware accepted the request.
The mode change is complete when {{ link_ad_api('/api/operation_mode/state') }} shows the new mode and `is_in_transition` is false.
If `is_in_transition` becomes false and the mode is not the new mode, the mode change failed.
For details, see the [operation mode](../../../features/operation_mode.md).
{% endblock %}
