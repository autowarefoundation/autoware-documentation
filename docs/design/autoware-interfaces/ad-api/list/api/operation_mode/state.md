---
title: /api/operation_mode/state
method: notification
type:
  name: autoware_adapi_v1_msgs/msg/OperationModeState
  msg:
    - name: mode
      text: The selected command for Autoware control.
    - name: is_autoware_control_enabled
      text: True if vehicle control by Autoware is enabled.
    - name: is_in_transition
      text: True if the operation mode is in transition.
    - name: is_stop_mode_available
      text: True if the operation mode can be changed to stop.
    - name: is_autonomous_mode_available
      text: True if the operation mode can be changed to autonomous.
    - name: is_local_mode_available
      text: True if the operation mode can be changed to local.
    - name: is_remote_mode_available
      text: True if the operation mode can be changed to remote.
---

{% extends 'design/autoware-interfaces/templates/autoware-interface.jinja2' %}
{% block description %}
Get the operation mode state.
For details, see the [operation mode](../../../features/operation_mode.md).
{% endblock %}
