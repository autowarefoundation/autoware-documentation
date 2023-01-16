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

  - name: /vehicle/status/control_mode
    type: autoware_auto_vehicle_msgs/msg/ControlModeReport
    used: { vehicle: pub, control: sub, simulation: sub }

  - name: /control/control_mode_request
    type: autoware_auto_vehicle_msgs/srv/ControlModeCommand
    used: { vehicle: srv, control: cli }
---

# List of component interfaces (current implementation)

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
