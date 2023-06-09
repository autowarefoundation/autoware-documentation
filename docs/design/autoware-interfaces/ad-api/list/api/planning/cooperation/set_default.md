---
title: /api/planning/cooperation/set_default
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/SetCooperationDefault
  req:
    - name: module
      text: The name of the target module.
    - name: cooperator
      text: The default decision of the target module.
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Set the default decision for cooperation. For details, see the [cooperation](../cooperation.md).
{% endblock %}
