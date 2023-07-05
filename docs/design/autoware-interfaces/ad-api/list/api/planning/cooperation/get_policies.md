---
title: /api/planning/cooperation/get_policies
status: not released
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/GetCooperationPolicies
  res:
    - name: status
      text: response status
    - name: policies.behavior
      text: The type of the target behavior.
    - name: policies.type
      text: The type of the cooporation policy.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the default decision that is used instead when the operator's decision is undecided.
For details, see the [cooperation](../../../../features/cooperation.md).
{% endblock %}
