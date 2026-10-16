---
title: /api/localization/initialize
status: v1.0.0
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/InitializeLocalization
  req:
    - name: pose
      text: A global pose as the initial guess. If omitted, the GNSS pose will be used.
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Request to initialize localization.
This API only guarantees that a request has been accepted.
Use [the state API](./initialization_state.md) to verify whether the request has been reflected.
For details, see the [localization](../../../features/localization.md).
{% endblock %}
