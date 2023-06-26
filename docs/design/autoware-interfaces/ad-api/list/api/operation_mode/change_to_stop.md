---
title: /api/operation_mode/change_to_stop
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/ChangeOperationMode
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Change the operation mode to stop.
For details, see the [operation mode](../../../features/operation_mode.md).
{% endblock %}
