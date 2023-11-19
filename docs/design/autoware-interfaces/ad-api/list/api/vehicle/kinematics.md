---
title: /api/vehicle/kinematics
status: not released
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/VehicleKinematics
  msg:
    - name: geographic_pose
      text: The longitude and latitude of the vehicle. If the map uses local coordinates, it will not be available.
    - name: pose
      text: The pose with covariance from the base link.
    - name: twist
      text: Vehicle current twist with covariance.
    - name: accel
      text: Vehicle current acceleration with covariance.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Publish vehicle kinematics.
{% endblock %}
