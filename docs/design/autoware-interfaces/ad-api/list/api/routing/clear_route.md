---
interface:
  name: /api/routing/clear_route
  type: autoware_adapi_v1_msgs/srv/ClearRoute
  method: function call
response:
  - name: status
    text: response status
---

# /api/routing/clear_route

{% include 'design/autoware-interfaces/templates/interface-header.jinja2' %}

## Description

Clear the route.

## Request

{% include 'design/autoware-interfaces/templates/interface-request.jinja2' %}

## Response

{% include 'design/autoware-interfaces/templates/interface-response.jinja2' %}
