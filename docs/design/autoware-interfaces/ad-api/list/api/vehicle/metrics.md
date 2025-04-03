---
title: /api/vehicle/metrics
status: not released
method: reliable stream
type:
  name: autoware_adapi_v1_msgs/msg/VehicleMetrics
  msg:
    - name: energy
      text: The remaining vehicle fuel or battery. Ratio with the maximum as 1.0.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Publish vehicle metrics.
{% endblock %}
