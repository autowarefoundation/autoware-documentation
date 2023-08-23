---
interfaces:
  - name: /control/command/control_cmd
    type: autoware_auto_control_msgs/msg/AckermannControlCommand
  - name: /control/command/gear_cmd
    type: autoware_auto_vehicle_msgs/msg/GearCommand
  - name: /control/command/hazard_lights_cmd
    type: autoware_auto_vehicle_msgs/msg/HazardLightsCommand
  - name: /control/command/turn_indicators_cmd
    type: autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand
  - name: /control/control_mode_request
    type: autoware_auto_vehicle_msgs/srv/ControlModeCommand
  - name: /control/current_gate_mode
    type: tier4_control_msgs/msg/GateMode
  - name: /control/vehicle_cmd_gate/is_paused
    type: tier4_control_msgs/msg/IsPaused
  - name: /control/vehicle_cmd_gate/is_start_requested
    type: tier4_control_msgs/msg/IsStartRequested
  - name: /control/vehicle_cmd_gate/set_pause
    type: tier4_control_msgs/srv/SetPause
  - name: /localization/acceleration
    type: geometry_msgs/msg/AccelWithCovarianceStamped
  - name: /localization/initialization_state
    type: autoware_adapi_v1_msgs/msg/LocalizationInitializationState
  - name: /localization/initialize
    type: tier4_localization_msgs/srv/PoseWithCovarianceStamped
  - name: /localization/kinematic_state
    type: nav_msgs/msg/Odometry
  - name: /localization/pose_estimator/pose_with_covariance
    type: geometry_msgs/msg/PoseWithCovarianceStamped
  - name: /map/get_differential_pointcloud_map
    type: autoware_map_msgs/srv/GetDifferentialPointCloudMap
  - name: /map/get_partial_pointcloud_map
    type: autoware_map_msgs/srv/GetPartialPointCloudMap
  - name: /map/map_projector_type
    type: tier4_map_msgs/msg/MapProjectorInfo
  - name: /map/vector_map
    type: autoware_auto_mapping_msgs/msg/HADMapBin
  - name: /perception/object_recognition/detection/objects
    type: autoware_auto_perception_msgs/msg/DetectedObjects
  - name: /perception/object_recognition/objects
    type: autoware_auto_perception_msgs/msg/PredictedObjects
  - name: /perception/obstacle_segmentation/pointcloud
    type: sensor_msgs/msg/PointCloud2
  - name: /perception/occupancy_grid_map/map
    type: nav_msgs/msg/OccupancyGrid
  - name: /perception/traffic_light_recognition/traffic_signals
    type: autoware_perception_msgs/msg/TrafficSignalArray
  - name: /planning/hazard_lights_cmd
    type: autoware_auto_vehicle_msgs/msg/HazardLightsCommand
  - name: /planning/mission_planning/change_route
    type: autoware_adapi_v1_msgs/srv/SetRoute
  - name: /planning/mission_planning/change_route_points
    type: autoware_adapi_v1_msgs/srv/SetRoutePoints
  - name: /planning/mission_planning/clear_route
    type: autoware_adapi_v1_msgs/srv/ClearRoute
  - name: /planning/mission_planning/route
    type: autoware_planning_msgs/msg/LaneletRoute
  - name: /planning/mission_planning/route_state
    type: autoware_adapi_v1_msgs/msg/RouteState
  - name: /planning/mission_planning/set_route
    type: autoware_adapi_v1_msgs/srv/SetRoute
  - name: /planning/mission_planning/set_route_points
    type: autoware_adapi_v1_msgs/srv/SetRoutePoints
  - name: /planning/scenario_planning/clear_velocity_limit
    type: tier4_planning_msgs/msg/VelocityLimitClearCommand
  - name: /planning/scenario_planning/max_velocity_candidates
    type: tier4_planning_msgs/msg/VelocityLimit
  - name: /planning/scenario_planning/trajectory
    type: autoware_auto_planning_msgs/msg/Trajectory
  - name: /planning/turn_indicators_cmd
    type: autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand
  - name: /sensing/gnss/pose_with_covariance
    type: geometry_msgs/msg/PoseWithCovarianceStamped
  - name: /sensing/imu/imu_data
    type: sensor_msgs/msg/Imu
  - name: /sensing/lidar/concatenated/pointcloud
    type: sensor_msgs/msg/PointCloud2
  - name: /sensing/lidar/top/outlier_filtered/pointcloud
    type: sensor_msgs/msg/PointCloud2
  - name: /sensing/vehicle_velocity_converter/twist_with_covariance
    type: geometry_msgs/msg/TwistWithCovarianceStamped
  - name: /system/emergency/control_cmd
    type: autoware_auto_control_msgs/msg/AckermannControlCommand
  - name: /system/emergency/gear_cmd
    type: autoware_auto_vehicle_msgs/msg/GearCommand
  - name: /system/emergency/hazard_lights_cmd
    type: autoware_auto_vehicle_msgs/msg/HazardLightsCommand
  - name: /system/fail_safe/mrm_state
    type: autoware_adapi_v1_msgs/msg/MrmState
  - name: /system/operation_mode/change_autoware_control
    type: tier4_system_msgs/srv/ChangeAutowareControl
  - name: /system/operation_mode/change_operation_mode
    type: tier4_system_msgs/srv/ChangeOperationMode
  - name: /system/operation_mode/state
    type: autoware_adapi_v1_msgs/msg/OperationModeState
  - name: /vehicle/status/control_mode
    type: autoware_auto_vehicle_msgs/msg/ControlModeReport
  - name: /vehicle/status/gear_status
    type: autoware_auto_vehicle_msgs/msg/GearReport
  - name: /vehicle/status/hazard_lights_status
    type: autoware_auto_vehicle_msgs/msg/HazardLightsReport
  - name: /vehicle/status/steering_status
    type: autoware_auto_vehicle_msgs/msg/SteeringReport
  - name: /vehicle/status/turn_indicators_status
    type: autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport
  - name: /vehicle/status/velocity_status
    type: autoware_auto_vehicle_msgs/msg/VelocityReport
---

# Interfaces

| interface name | data type |
| -------------- | --------- |

{%- for interface in interfaces %}
| {{ interface.name }} | {{ interface.type }} |
{%- endfor %}
