---
title: /api/vehicle/specs
status: not released
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/GetVehicleSpecs
  res:
    - name: status
      text: response status
    - name: specs
      text: vehicle specifications
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the vehicle specifications. The default implementation does not provide this API yet.
{% endblock %}
