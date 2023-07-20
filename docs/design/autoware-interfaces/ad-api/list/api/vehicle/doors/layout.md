---
title: /api/vehicle/doors/layout
status: not released
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/GetDoorLayout
  res:
    - name: status
      text: response status
    - name: doors.roles
      text: The roles of the door in the service the vehicle provides.
    - name: doors.description
      text: The description of the door for display in the interface.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the door layout. It is an array of roles and descriptions for each door.
{% endblock %}
