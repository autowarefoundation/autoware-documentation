---
title: /api/planning/cooperation/set_policies
status: not released
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/SetCooperationPolicies
  req:
    - name: policies.behavior
      text: The type of the target behavior.
    - name: policies.sequence
      text: The type of the target sequence.
    - name: policies.policy
      text: The type of the cooporation policy.
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Set the default decision that is used instead when the operator's decision is undecided.
For details, see the [cooperation](../../../../features/cooperation.md).
{% endblock %}
