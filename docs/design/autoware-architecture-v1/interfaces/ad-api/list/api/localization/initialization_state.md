---
title: /api/localization/initialization_state
status: v1.0.0
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/LocalizationInitializationState
  msg:
    - name: stamp
      text: Timestamp when this message was sent.
    - name: state
      text: A value of the localization initialization state.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the initialization state of localization.
For details, see the [localization](../../../features/localization.md).
{% endblock %}
