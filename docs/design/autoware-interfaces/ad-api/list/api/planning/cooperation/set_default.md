---
title: /api/planning/cooperation/set_default
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/SetCooperationDefault
  req:
    - name: defaults.behavior
      text: The type of the target behavior.
    - name: defaults.cooperator
      text: The default decision of the target behavior.
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Set the default decision for cooperation. For details, see the [cooperation](../cooperation.md).
{% endblock %}
