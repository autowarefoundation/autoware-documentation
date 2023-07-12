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

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Change the operation mode to autonomous.
For details, see the [operation mode](../../../features/operation_mode.md).
{% endblock %}
