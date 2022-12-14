---
components:
  - adapi
  - control
  - localization
  - map
  - perception
  - planning
  - sensing
  - simulator
  - system
  - vehicle

interfaces:
  - name: /vehicle/status/control_mode
    type: autoware_auto_vehicle_msgs/msg/ControlModeReport
    used: { vehicle: pub, control: sub, simulation: sub }

  - name: /control/control_mode_request
    type: autoware_auto_vehicle_msgs/srv/ControlModeCommand
    used: { vehicle: srv, control: cli }
---

# List of component interfaces (future design)

{%- for component in components %}

## {{ component }}

| interface type | interface name | data type |
| -------------- | -------------- | --------- |

{%- for interface in interfaces %}
{%- if component in interface.used %}
| {{ interface.used[component] }} | {{ interface.name }} | {{ interface.type }} |
{%- endif %}
{%- endfor %}
{%- endfor %}
