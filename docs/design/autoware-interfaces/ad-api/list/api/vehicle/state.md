---
title: /api/vehicle/dimensions
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/VehicleState
  msg:
    - name: turn_indicator
      text: Turn indicators status, only either left or right will be enabled.
    - name: hazard_light
      text: Hazard light status.
    - name: gear
      text: Gear status.
    - name: steering_tire_angle
      text: Vehicle current tire angle in radian.
    - name: energy_level
      text: Battery percentage or fuel percentage, it will depends on the vehicle.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Publish vehicle state information.
{% endblock %}
