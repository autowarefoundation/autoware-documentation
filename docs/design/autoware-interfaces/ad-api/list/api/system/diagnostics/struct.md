---
title: /api/system/diagnostics/struct
status: v1.3.0
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/DiagGraphStruct
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: id
      text: ID to check correspondence between struct and status.
    - name: nodes
      text: Static data for nodes in diagnostic graph.
    - name: links
      text: Static data for links in diagnostic graph.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
This is the static part of the diagnostics.
If static data is published with new ID, ignore dynamic data with old ID.
See [diagnostics](../../../../features/diagnostics.md) for details.
{% endblock %}
