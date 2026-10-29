---
title: /api/perception/objects
status: not released
method: reliable stream
type:
  name: autoware_adapi_v1_msgs/msg/DynamicObjectArray
  msg:
    - name: header
      text: Header with the timestamp and the frame ID of the poses.
    - name: objects.id
      text: The UUID of each object.
    - name: objects.existence_probability
      text: The probability that the object exists.
    - name: objects.classification
      text: The classes of the object and their probabilities.
    - name: objects.kinematics
      text: The pose, twist, and acceleration of the object, and its predicted paths.
    - name: objects.shape
      text: The shape of the object with its dimensions and polygon.
---

{% extends 'design/autoware-architecture-v1/interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the array of recognized objects with their classes, shapes, current poses, and predicted paths.
For details, see the [perception](../../../features/perception.md).
{% endblock %}
