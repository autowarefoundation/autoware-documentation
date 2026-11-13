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

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Reset the latch state of the diagnostic graph.
The default implementation accepts this request only when the MRM state is NORMAL or MRM_SUCCEEDED.
See [diagnostics](../../../../features/diagnostics.md) for details.
{% endblock %}
