---
title: /api/planning/cooperation/set_decisions
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/SetCooperationDecisions
  req:
    - name: commands.uuid
      text: The ID in the cooperation status.
    - name: commands.cooperator
      text: The operator's decision.
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Set the operator's decision for cooperation. For details, see the [cooperation](../cooperation.md).
{% endblock %}
