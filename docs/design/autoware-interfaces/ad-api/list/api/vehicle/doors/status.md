---
title: /api/vehicle/doors/status
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/DoorStatusArray
  msg:
    - name: doors.status
      text: current door status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
The status of each door such as opened or closed.
The array index corresponds to the door layout.
{% endblock %}
