---
# This file is generated by tools/autoware-interfaces/generate.py
title: autoware_adapi_v1_msgs/msg/DiagGraphStruct
uses:
  - autoware_adapi_v1_msgs/msg/DiagLeafStruct
  - autoware_adapi_v1_msgs/msg/DiagLinkStruct
  - autoware_adapi_v1_msgs/msg/DiagNodeStruct
---

{% extends 'design/autoware-interfaces/templates/autoware-data-type.jinja2' %}
{% block definition %}

```txt
builtin_interfaces/Time stamp
string id
autoware_adapi_v1_msgs/DiagNodeStruct[] nodes
autoware_adapi_v1_msgs/DiagLeafStruct[] diags
autoware_adapi_v1_msgs/DiagLinkStruct[] links
```

{% endblock %}
