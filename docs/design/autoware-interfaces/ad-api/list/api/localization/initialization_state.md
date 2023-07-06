---
title: /api/localization/initialization_state
status: v1.0.0
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/LocalizationInitializationState
  msg:
    - name: state
      text: A value of the localization initialization state.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the initialization state of localization.
For details, see the [localization](../../../features/localization.md).
{% endblock %}
