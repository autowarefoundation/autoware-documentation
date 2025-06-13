---
title: /api/fail_safe/mrm_description
status: not released
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/ListMrmDescription
  req:
    - name: descriptions.behavior
      text: The behavior ID of the MRM.
    - name: descriptions.name
      text: The name of the MRM.
    - name: descriptions.description
      text: The description of the MRM.
  res:
    - name: status
      text: response status
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the list of MRM description.
For details, see the [fail-safe](../../../../features/fail-safe.md).
{% endblock %}
