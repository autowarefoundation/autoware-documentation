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
    - name: factors.direction
      text: The direction of the steering factor.
    - name: factors.status
      text: The status of the steering factor.
    - name: factors.module
      text: The module name of the steering factor.
    - name: factors.detail
      text: The additional information of the steering factor.
    - name: factors.cooperation
      text: The cooperation status if the module supports.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the steering factors, sorted in ascending order of distance.
For details, see the [planning](./index.md).
{% endblock %}
