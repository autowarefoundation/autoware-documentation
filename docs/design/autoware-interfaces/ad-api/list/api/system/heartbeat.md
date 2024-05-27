---
title: /api/system/heartbeat
status: v1.3.0
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/Heartbeat
  msg:
    - name: stamp
      text: Timestamp in Autoware for delay checking.
    - name: seq
      text: Sequence number for order verification, wraps at 65535.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
The heartbeat frequency is 10 Hz.
{% endblock %}
