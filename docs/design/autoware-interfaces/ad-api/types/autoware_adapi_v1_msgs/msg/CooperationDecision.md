---
# This file is generated by tools/autoware-interfaces/generate.py
title: autoware_adapi_v1_msgs/msg/CooperationDecision
used:
  - autoware_adapi_v1_msgs/msg/CooperationCommand
  - autoware_adapi_v1_msgs/msg/CooperationStatus
---

{% extends 'design/autoware-interfaces/templates/autoware-data-type.jinja2' %}
{% block definition %}

```txt
uint8 UNKNOWN = 0
uint8 DEACTIVATE = 1
uint8 ACTIVATE = 2
uint8 AUTONOMOUS = 3
uint8 UNDECIDED = 4

uint8 decision
```

{% endblock %}
