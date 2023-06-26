---
title: /api/planning/steering_factors
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/SteeringFactorArray
  msg:
    - name: factors.pose
      text: The base link pose related to the steering factor.
    - name: factors.distance
      text: The distance from the base link to the above pose.
    - name: factors.type
      text: The type of the steering factor.
    - name: factors.direction
      text: The direction of the steering factor.
    - name: factors.status
      text: The status of the steering factor.
    - name: factors.detail
      text: The additional information of the steering factor.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the steering factors, sorted in ascending order of distance.
For details, see the [planning factors](../../../features/planning-factors.md).
{% endblock %}
