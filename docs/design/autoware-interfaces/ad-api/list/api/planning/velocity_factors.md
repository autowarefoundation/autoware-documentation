---
title: /api/planning/velocity_factors
status: not released
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/VelocityFactorArray
  msg:
    - name: factors.pose
      text: The base link pose related to the velocity factor.
    - name: factors.distance
      text: The distance from the base link to the above pose.
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

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the velocity factors, sorted in ascending order of distance.
For details, see the [planning factors](../../../features/planning-factors.md).
{% endblock %}
