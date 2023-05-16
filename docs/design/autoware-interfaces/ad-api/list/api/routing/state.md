---
interface:
  name: /api/routing/state
  type: autoware_adapi_v1_msgs/msg/RouteState
  method: notification
message:
  - name: state
    text: A value of the [route state](./index.md).
---

# /api/routing/state

{% include 'design/autoware-interfaces/templates/interface-header.jinja2' %}

## Description

Get the route state. For details, see the [route state](./index.md).

## Message

{% include 'design/autoware-interfaces/templates/interface-message.jinja2' %}
