---
title: /api/vehicle/status
status: v1.4.0
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/VehicleStatus
  msg:
    - name: gear
      text: Gear status.
    - name: turn_indicators
      text: Turn indicators status, only either left or right will be enabled.
    - name: hazard_lights
      text: Hazard lights status.
    - name: steering_tire_angle
      text: Vehicle current tire angle in radian.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Publish vehicle state information.
{% endblock %}
