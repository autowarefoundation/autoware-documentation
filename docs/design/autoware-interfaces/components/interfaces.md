---
components:
  - adapi
  - control
  - localization
  - perception
  - planning
  - sensing
  - simulator
  - system
  - vehicle

interfaces:
  - name: /localization/initialization_state
    type: autoware_adapi_v1_msgs/msg/LocalizationInitializationState
    used: { localization: pub, adapi: sub }

  - name: /localization/initialize
    type: autoware_adapi_v1_msgs/srv/InitializeLocalization
    used: { localization: srv, adapi: cli }

  - name: /planning/mission_planning/set_route_points
    type: autoware_adapi_v1_msgs/srv/SetRoutePoints
    used: { planning: srv, adapi: cli }

  - name: /planning/mission_planning/set_route
    type: autoware_planning_msgs/srv/SetRoute
    used: { planning: srv, adapi: cli }

  - name: /planning/mission_planning/clear_route
    type: autoware_adapi_v1_msgs/msg/ClearRoute
    used: { planning: srv, adapi: cli }

  - name: /planning/mission_planning/route
    type: autoware_planning_msgs/msg/LaneletRoute
    used: { planning: pub, adapi: sub }

  - name: /planning/mission_planning/route_state
    type: autoware_adapi_v1_msgs/msg/RouteState
    used: { planning: pub, adapi: sub }

  - name: /vehicle/control_mode_report
    type: autoware_auto_vehicle_msgs/msg/ControlModeReport
    used: { vehicle: pub, control: sub, simulation: sub }

  - name: /vehicle/control_mode_request
    type: autoware_auto_vehicle_msgs/srv/ControlModeCommand
    used: { vehicle: srv, control: cli }
---

# List of component interfaces

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
