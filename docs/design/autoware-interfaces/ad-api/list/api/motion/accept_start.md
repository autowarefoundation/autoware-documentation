---
title: /api/motion/accept_start
status: not released
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/AcceptStart
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Accept the vehicle to start. This API can be used when the [motion state](../../../features/motion.md) is STARTING.
{% endblock %}
