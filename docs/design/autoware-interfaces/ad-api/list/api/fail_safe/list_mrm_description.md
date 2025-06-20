---
title: /api/fail_safe/list_mrm_description
status: v1.9.0
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/ListMrmDescription
  res:
    - name: descriptions.behavior
      text: The behavior ID of the MRM.
    - name: descriptions.name
      text: The name of the MRM.
    - name: descriptions.description
      text: The description of the MRM.
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the list of MRM description.
For details, see the [fail-safe](../../../features/fail-safe.md).
{% endblock %}
