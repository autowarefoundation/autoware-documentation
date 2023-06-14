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
Publish door status in array according to the door location information that received from vehicle info service.
{% endblock %}
