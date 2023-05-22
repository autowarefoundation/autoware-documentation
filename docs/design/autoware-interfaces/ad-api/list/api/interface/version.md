---
title: /api/interface/version
method: function call
type:
  name: autoware_adapi_version_msgs/srv/InterfaceVersion
  res:
    - name: major
      text: major version
    - name: minor
      text: minor version
    - name: patch
      text: patch version
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the interface version. The version follows Semantic Versioning.
{% endblock %}
