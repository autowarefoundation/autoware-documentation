# Planning component design

## Overview

Planning stack acts as the “brain” of autonomous driving. It uses all the results from Localization, Perception, and Map stacks to decide its maneuver and gives final trajectory to Control stack.

## Role

These are high-level roles of Planning stack:

- Calculates route that navigates to desired goal
- Plans trajectory to follow the route
  - Make sure that vehicle does not collide with obstacles, including pedestrians and other vehicles
  - Make sure that the vehicle follows traffic rules during the navigation. This includes following traffic light, stopping at stoplines, stopping at crosswalks, etc.
- Plan sequences of trajectories that is feasible for the vehicle. (e.g. no sharp turns that is kinematically impossible)

## Input

The table below summarizes the overall input into Planning stack:

| Input                           | Topic Name(Data Type)                                                                                               | Explanation                                                                                                                                                                                                                                                                                                         |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Vehicle Pose                    | `/tf (map->base_link)`<br>(_tf::tfMessage_)                                                                         | Planning requires vehicle pose in map frame, which is the frame where all planning takes place.                                                                                                                                                                                                                     |
| Vehicle Kinematics              | `/localization/kinematic_state`<br>(_nav_msgs/msg/Odometry_)                                                        | This includes vehicle's pose and velocity information. It is used to predict future pose on trajectory to detect collision with other objects.                                                                                                                                                                      |
| Map data                        | `/map/vector_map`<br>(_autoware_auto_mapping_msgs/msg/HADMapBin_)                                                   | This includes all static information about the environment, such as: <ul><li>Lane connection information used for route planning from starting position to goal position</li><li>Lane geometry to generate reference path used to calculate trajectory </li><li> All information related to traffic rules</li></ul> |
| Detected Object Information     | `/perception/object_recognition/objects`<br>(_autoware_auto_perception_msgs/msg/PredictedObjects_)                  | This includes information that cannot be known beforehand such as pedestrians and other vehicles. Planning stack will plan maneuvers to avoid collision with such objects.                                                                                                                                          |
| Detected Obstacle Information   | `/perception/obstacle_segmentation/pointcloud`<br>(_sensor_msgs/msg/PointCloud2_)                                   | This includes information on the location of obstacles. This is more primitive information and is used for emergency stops, etc.                                                                                                                                                                                    |
| Occupancy Map Information       | `/perception/occupancy_grid_map/map`<br>(_nav_msgs/msg/OccupancyGrid_)                                              | This includes information that cannot be known beforehand such as pedestrians and other vehicles. Planning stack will plan maneuvers to avoid collision with such objects.                                                                                                                                          |
| TrafficLight recognition result | `/perception/traffic_light_recognition/traffic_signals`<br>(_autoware_auto_perception_msgs/msg/TrafficSignalArray_) | This is the real time information about the state of each traffic light. Planning stack will extract the one that is relevant to planned path and use it to decide whether to stop at intersections.                                                                                                                |
| Goal position                   | `/planning/mission_planning/goal`<br>(_geometry_msgs::PoseStamped_)                                                 | This is the final pose that Planning stack will try to achieve.                                                                                                                                                                                                                                                     |
| Check point position            | `/planning/mission_planning/check_point`<br>(_geometry_msgs::PoseStamped_)                                          | This is the midpoint that Planning will try to go at on the way to the destination. This is used when calculating the route.                                                                                                                                                                                        |

## Output

The table below summarizes the final output from Planning stack:

| Output        | Topic(Data Type)                                                                            | Explanation                                                                                                                                                                                                                                  |
| ------------- | ------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Trajectory    | `/planning/trajectory`<br>(_autoware_auto_planning_msgs/msg/Trajectory_)                    | This is the sequence of pose, twist and acceleration that Control stack must follow. This must be smooth, and kinematically possible to follow by the Control stack. By default, the trajectory is 10 seconds long at 0.1 second resolution. |
| Turn Signal   | `/planning/turn_indicators_cmd`<br>(_autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand_) | This is the output to control turn signals of the vehicle. Planning stack will make sure that turn signal will be turned on according to planned maneuver.                                                                                   |
| Hazard Signal | `/planning/hazard_lights_cmd`<br>(_autoware_auto_vehicle_msgs/msg/HazardLightsCommand_)     | This is the output to control hazard signals of the vehicle. Planning stack will make sure that hazard signal will be turned on according to planned maneuver.                                                                               |

## Implementation

The implementation of the planning module in the latest version is shown as below.

![reference-implementation](image/planning-diagram.drawio.svg)

For more details, please refer to the design documents in each package.

- [_mission_planner_](https://autowarefoundation.github.io/autoware.universe/main/planning/mission_planner/): calculate route from start to goal based on the map information.
- [_behavior_path_planner_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/): calculates path and drivable area based on the traffic rules.
  - [_lane_following_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/#lane-following)
  - [_lane_change_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/#lane-change)
  - [_avoidance_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/#avoidance)
  - [_pull_over_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/#pull-over)
  - [_pull_out_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/#pull-out)
  - _side_shift_
- [_behavior_velocity_planner_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/): calculates max speed based on the traffic rules.
  - [_detection_area_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/detection-area-design/)
  - [_blind_spot_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/blind-spot-design/)
  - [_cross_walk_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/crosswalk-design/)
  - [_stop_line_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/stop-line-design/)
  - [_traffic_light_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/traffic-light-design/)
  - [_intersection_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/intersection-design/)
  - [_no_stopping_area_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/no-stopping-area-design/)
  - [_virtual_traffic_light_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/virtual-traffic-light-design/)
  - [_occlusion_spot_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/occlusion-spot-design/)
  - [_run_out_](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_planner/run-out-design/)
- [_obstacle_avoidance_planner_](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_avoidance_planner/): calculate path shape under obstacle and drivable area constraints
- [_surround_obstacle_checker_](https://autowarefoundation.github.io/autoware.universe/main/planning/surround_obstacle_checker/): keeps the vehicle being stopped when there are obstacles around the ego-vehicle. It works only when the vehicle is stopped.
- [_obstacle_stop_planner_](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_stop_planner/): When there are obstacles on or near the trajectory, it calculates the maximum velocity of the trajectory points depending on the situation: stopping, slowing down, or adaptive cruise (following the car).
  - [_stop_](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_stop_planner/#obstacle-stop-planner_1)
  - [_slow_down_](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_stop_planner/#slow-down-planner)
  - [_adaptive_cruise_](https://autowarefoundation.github.io/autoware.universe/main/planning/obstacle_stop_planner/#adaptive-cruise-controller)
- [_costmap_generator_](https://autowarefoundation.github.io/autoware.universe/main/planning/costmap_generator/): generates a costmap for path generation from dynamic objects and lane information.
- [_freespace_planner_](https://autowarefoundation.github.io/autoware.universe/main/planning/freespace_planner/): calculates trajectory considering the feasibility (e.g. curvature) for the freespace scene. Algorithms are described [here](https://autowarefoundation.github.io/autoware.universe/main/planning/freespace_planning_algorithms/).
- _scenario_selector_ : chooses a trajectory according to the current scenario.
- [_external_velocity_limit_selector_](https://autowarefoundation.github.io/autoware.universe/main/planning/external_velocity_limit_selector/): takes an appropriate velocity limit from multiple candidates.
- [_motion_velocity_smoother_](https://autowarefoundation.github.io/autoware.universe/main/planning/motion_velocity_smoother/): calculates final velocity considering velocity, acceleration, and jerk constraints.

## Supported Functions

![supported-functions](image/planning-functions.drawio.svg)

## Notation

### [1] self-crossing road and overlapped

To support the self-crossing road and overlapped road in the opposite direction, each planning module has to meet the [specifications](https://autowarefoundation.github.io/autoware.universe/main/common/motion_utils/)

Currently, the supported modules are as follows.

- lane_following (in behavior_path_planner)
- detection_area (in behavior_velocity_planner)
- stop_line (in behavior_velocity_planner)
- virtual_traffic_light (in behavior_velocity_planner)
- obstacle_avoidance_planner
- obstacle_stop_planner
- motion_velocity_smoother

#### [2] Size of Path Points

Some functions do not support paths with only one point. Therefore, each modules should generate the path with more than two path points.
