---
title: /api/interface/list
status: not released
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/GetInterfaceList
  res:
    - name: status
      text: response status
    - name: apis.name
      text: The name of the API.
    - name: apis.available
      text: The support status of the API.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the interface list. All interfaces for that version are listed, including unsupported ones.
{% endblock %}
