---
title: /api/planning/steering_factors
status: not released
method: reliable stream
type:
  name: autoware_adapi_v1_msgs/msg/SteeringFactorArray
  msg:
    - name: header
      text: Header with the timestamp and the frame ID of the poses.
    - name: factors.pose
      text: The start and end poses of the steering section, in the frame of the message header.
    - name: factors.distance
      text: The distances from the vehicle to the above poses.
    - name: factors.direction
      text: The direction of the steering factor.
    - name: factors.status
      text: The status of the steering factor.
    - name: factors.behavior
      text: The behavior type of the steering factor.
    - name: factors.sequence
      text: The sequence type of the steering factor.
    - name: factors.detail
      text: The additional information of the steering factor.
    - name: factors.cooperation
      text: The cooperation status if the module supports.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the steering factors, sorted in ascending order of distance.
For details, see the [planning factors](../../../features/planning-factors.md).
{% endblock %}
