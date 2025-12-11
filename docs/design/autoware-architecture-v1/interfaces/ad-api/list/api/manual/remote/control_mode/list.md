---
title: /api/manual/remote/control_mode/list
status: v1.8.0
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/ListManualControlMode
  res:
    - name: status
      text: response status
    - name: modes
      text: List of available modes.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
List the available manual control modes as described in [manual control](../../../../../features/manual-control.md).
The disabled mode is not included in the available modes.
{% endblock %}
