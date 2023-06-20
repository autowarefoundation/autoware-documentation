---
title: /api/localization/initialize
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

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Request to initialize localization.
For details, see the [localization](../../../features/localization.md).
{% endblock %}
