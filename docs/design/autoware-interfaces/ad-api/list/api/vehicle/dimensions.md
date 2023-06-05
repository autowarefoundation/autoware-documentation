---
title: /api/vehicle/dimensions
method: function call
type:
  name: autoware_adapi_v1_msgs/srv/GetVehicleDimensions
  res:
    - name: status
      text: response status
    - name: dimensions
      text: vehicle dimensions
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the vehicle dimensions. See [here](../../../../../components/vehicle-dimensions.md) for the definition of each value.
{% endblock %}
