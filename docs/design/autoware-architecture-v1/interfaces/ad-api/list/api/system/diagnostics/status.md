---
title: /api/system/diagnostics/status
status: v1.3.0
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/DiagGraphStatus
  msg:
    - name: stamp
      text: Timestamp when this message was sent
    - name: id
      text: ID to check correspondence between struct and status.
    - name: nodes
      text: Dynamic data for nodes in diagnostic graph.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
This is the dynamic part of the diagnostics.
If static data is published with new ID, ignore dynamic data with old ID.
See [diagnostics](../../../../features/diagnostics.md) for details.
{% endblock %}
