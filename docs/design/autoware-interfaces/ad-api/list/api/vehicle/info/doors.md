---
title: /api/vehicle/doors/layout
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/GetDoorLayout
  res:
    - name: status
      text: response status
    - name: doors.roles
      text: The roles of the door in the current operation of the vehicle.
    - name: doors.pose
      text: The pose of the door where the direction to get off is positive X-axis.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the door layout. It is an array of pose for each door. The array index corresponds to the door status.
{% endblock %}
