---
title: /api/planning/cooperation/get_defaults
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/GetCooperationDefaults
  res:
    - name: status
      text: response status
    - name: defaults.behavior
      text: The type of the target behavior.
    - name: defaults.cooperator
      text: The default decision of the target behavior.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the default decision that is used instead when the operator's decision is undecided. For details, see the [cooperation](../cooperation.md).
{% endblock %}
