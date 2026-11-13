---
title: /api/vehicle/metrics
status: v1.9.0
method: reliable stream
type:
  name: autoware_adapi_v1_msgs/msg/VehicleMetrics
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: energy
      text: The remaining vehicle fuel or battery. Ratio with the maximum as 1.0.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Publish vehicle metrics.
The default implementation publishes this topic at the rate of its `update_rate` parameter, which is 0.1 Hz (once in 10 seconds) by default.
It calculates `energy` as the `energy_level` of `/vehicle/status/battery_charge` divided by its `max_energy_level` parameter. If it did not receive that topic, `energy` is NaN.
{% endblock %}
