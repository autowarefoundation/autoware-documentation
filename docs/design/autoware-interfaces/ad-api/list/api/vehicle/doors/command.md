---
title: /api/vehicle/doors/command
status: v1.2.0
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/SetDoorCommand
  req:
    - name: doors.index
      text: The index of the target door.
    - name: doors.command
      text: The command for the target door.
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Set the door command. This API is only available if the vehicle supports software door control.
This API fails if the doors cannot be opened or closed safely.
{% endblock %}
