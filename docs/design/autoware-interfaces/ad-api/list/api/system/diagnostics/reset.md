---
title: /api/system/diagnostics/reset
status: v1.9.0
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/ResetDiagGraph
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Reset the latch state of the diagnostic graph.
See [diagnostics](../../../../features/diagnostics.md) for details.
{% endblock %}
