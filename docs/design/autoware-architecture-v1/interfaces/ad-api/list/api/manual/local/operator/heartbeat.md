---
title: /api/manual/local/operator/heartbeat
status: v1.8.0
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/ManualOperatorHeartbeat
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: ready
      text: Whether the operator is able to continue driving.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
The application needs to determine whether the operator is able to drive and send that information via this API.
For details, see the [manual control](../../../../../features/manual-control.md).
{% endblock %}
