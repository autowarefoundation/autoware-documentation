---
components:
  - adapi
  - system
  - map
  - localization
  - planning
  - control
  - sensing
  - perception
  - vehicle
  - simulator

interfaces:
  - name: /autoware/engage
    type: autoware_auto_vehicle_msgs/msg/Engage
    used: { system: pub }

  - name: /autoware/state
    type: autoware_auto_system_msgs/msg/AutowareState
    used: { system: pub }

  - name: /map/pointcloud_map
    type: sensor_msgs/msg/PointCloud2
    used: { map: pub }

  - name: /map/vector_map
    type: autoware_auto_mapping_msgs/msg/HADMapBin
    used: { map: pub }

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

  - name: /vehicle/status/control_mode
    temp: /vehicle/control_mode/report
    type: autoware_auto_vehicle_msgs/msg/ControlModeReport
    used: { vehicle: pub, control: sub, simulation: sub }

  - name: /control/control_mode_request
    temp: /vehicle/control_mode/request
    type: autoware_auto_vehicle_msgs/srv/ControlModeCommand
    used: { vehicle: srv, control: cli }
---

# Interfaces

{%- for component in components %}

## {{ component }}

| interface type | interface name | data type | comments |
| -------------- | -------------- | --------- | -------- |

{%- for interface in interfaces %}
{%- if component in interface.used %}
| {{ interface.used[component] }} | {{ interface.name }} | {{ interface.type }} | {{ interface.get('note', '-') }} |
{%- endif %}
{%- endfor %}
{%- endfor %}
