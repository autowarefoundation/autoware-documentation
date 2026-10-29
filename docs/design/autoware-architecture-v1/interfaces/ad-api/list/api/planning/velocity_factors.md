---
title: /api/planning/velocity_factors
status: not released
method: reliable stream
type:
  name: autoware_adapi_v1_msgs/msg/VelocityFactorArray
  msg:
    - name: header
      text: Header with the timestamp and the frame ID of the poses.
    - name: factors.pose
      text: The pose related to the velocity factor, in the frame of the message header.
    - name: factors.distance
      text: The distance from the vehicle to the above pose.
    - name: factors.status
      text: The status of the velocity factor.
    - name: factors.behavior
      text: The behavior type of the velocity factor.
    - name: factors.sequence
      text: The sequence type of the velocity factor.
    - name: factors.detail
      text: The additional information of the velocity factor.
    - name: factors.cooperation
      text: The cooperation status if the module supports.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the velocity factors, sorted in ascending order of distance.
For details, see the [planning factors](../../../features/planning-factors.md).
{% endblock %}
