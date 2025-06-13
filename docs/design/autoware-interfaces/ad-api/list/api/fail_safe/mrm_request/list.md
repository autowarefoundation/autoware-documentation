---
title: /api/fail_safe/mrm_request/list
status: not released
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/MrmRequestList
  msg:
    - name: requests.sender
      text: The sender name of the MRM request.
    - name: requests.strategy
      text: The strategy of the MRM request.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
List the MRM requests from all senders.
For details, see the [fail-safe](../../../../features/fail-safe.md).
{% endblock %}
