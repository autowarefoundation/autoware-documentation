---
title: /api/perception/objects
status: not released
method: realtime stream
type:
  name: autoware_adapi_v1_msgs/msg/DynamicObjectArray
  msg:
    - name: objects.id
      text: The UUID of each object
    - name: objects.existence_probability
      text: The probability of the object exits
    - name: objects.classification
      text: The type of the object recognized and the confidence level
    - name: objects.kinematics
      text: Consist of the object pose, twist, acceleration and the predicted_paths
    - name: objects.shape
      text: escribe the shape of the object with dimension, and polygon
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the recognized objects array with label, shape, current position and predicted path
For details, see the [perception](../../../features/perception.md).
{% endblock %}
