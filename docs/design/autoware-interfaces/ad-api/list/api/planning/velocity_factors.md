---
title: /api/planning/velocity_factors
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/VelocityFactorArray
  msg:
    - name: factors.pose
      text: The base link pose related to the velocity factor.
    - name: factors.distance
      text: The distance from the base link to the above pose.
    - name: factors.type
      text: The type of the velocity factor.
    - name: factors.status
      text: The status of the velocity factor.
    - name: factors.detail
      text: The additional information of the velocity factor.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the velocity factors, sorted in ascending order of distance.
For details, see the [planning](./index.md).
{% endblock %}
