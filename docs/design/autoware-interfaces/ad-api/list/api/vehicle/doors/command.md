---
title: /api/vehicle/doors/command
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/SetDoorCommand
  req:
    - name: doors.command
      text: The command of the door.
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Set the door command. This API is only available if the vehicle supports software door control.
The array index corresponds to the door layout.
{% endblock %}
