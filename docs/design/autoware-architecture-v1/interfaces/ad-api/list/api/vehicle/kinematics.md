---
title: /api/vehicle/kinematics
status: v1.1.0
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/VehicleKinematics
  msg:
    - name: geographic_pose
      text: The latitude, longitude, and altitude of the vehicle. If the map uses local coordinates, it is not available (NaN).
    - name: pose
      text: The pose of the base link with covariance in the map frame.
    - name: twist
      text: Vehicle current twist with covariance.
    - name: accel
      text: Vehicle current acceleration with covariance.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Publish vehicle kinematics.
{% endblock %}
