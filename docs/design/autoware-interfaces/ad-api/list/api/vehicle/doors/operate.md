---
title: /api/vehicle/doors/operate
status: not released
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/SetDoorOperation
  req:
    - name: doors.operation
      text: The operation of the door.
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Set the door operation. This API is only available if the vehicle supports software door control.
The array index corresponds to the door layout.
{% endblock %}
