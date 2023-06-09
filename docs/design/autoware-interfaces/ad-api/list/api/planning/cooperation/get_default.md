---
title: /api/planning/cooperation/get_default
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/GetCooperationDefault
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
Set the default decision for cooperation. For details, see the [cooperation](../cooperation.md).
{% endblock %}
