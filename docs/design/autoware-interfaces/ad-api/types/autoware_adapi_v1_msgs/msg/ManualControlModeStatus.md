---
# This file is generated by tools/autoware-interfaces/generate.py
title: autoware_adapi_v1_msgs/msg/ManualControlModeStatus
uses:
  - autoware_adapi_v1_msgs/msg/ManualControlMode
---

{% extends 'design/autoware-interfaces/templates/autoware-data-type.jinja2' %}
{% block definition %}

```txt
builtin_interfaces/Time stamp
autoware_adapi_v1_msgs/ManualControlMode mode
```

{% endblock %}
