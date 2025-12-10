---
title: /api/motion/state
status: not released
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/MotionState
  msg:
    - name: state
      text: A value of the motion state.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the motion state.
For details, see the [motion state](../../../features/motion.md).
{% endblock %}
