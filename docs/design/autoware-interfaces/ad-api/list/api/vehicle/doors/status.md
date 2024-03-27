---
title: /api/vehicle/doors/status
status: v1.2.0
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
{% endblock %}
