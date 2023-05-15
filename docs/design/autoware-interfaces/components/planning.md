# Planning

![Node diagram](images/Planning-Bus-ODD-Architecture.drawio.svg)

## Inputs

### 3D Object Predictions

set of perceived objects around ego that need to be avoided when planning a trajectory. Published by the Perception module.

- [autoware_auto_perception_msgs/msg/PredictedObjects](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/PredictedObjects.idl)
  - [std_msgs/Header](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html) header
  - sequence<[autoware_auto_perception_msgs::msg::PredictedObject](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/PredictedObject.idl)> objects
    - unique_identifier_msgs::msg::UUID uuid
    - float existence_probability
    - sequence<[autoware_auto_perception_msgs::msg::ObjectClassification](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/ObjectClassification.idl)> classification
      - uint8 classification
      - float probability
    - [autoware_auto_perception_msgs::msg::PredictedObjectKinematics](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/PredictedObjectKinematics.idl) kinematics
      - [geometry_msgs::msg::PoseWithCovariance](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html) initial_pose
      - [geometry_msgs::msg::TwistWithCovariance](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovariance.html)
      - [geometry_msgs::msg::AccelWithCovariance](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/AccelWithCovariance.html) initial_acceleration
      - sequence<[autoware_auto_perception_msgs::msg::PredictedPath](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/PredictedPath.idl), 10> predicted_paths
        - sequence<[geometry_msgs::msg::Pose](https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Pose.html), 100> path
        - builtin_interfaces::msg::Duration time_step
        - float confidence
    - sequence<[autoware_auto_perception_msgs::msg::Shape](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/Shape.idl), 5> shape
      - [geometry_msgs::msg::Polygon](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Polygon.html) polygon
      - float height

### Traffic Light Response

Service response with traffic light information. **The message definition is under discussion.**

- TrafficLightResponse
  - uint64 traffic_light_id
  - uint8 traffic_light_state

With the traffic_light_state being one of the following

- GREEN = 1
- GREEN_BLINKING = 2
- YELLOW = 3
- YELLOW_BLINKING = 4
- RED = 5
- RED_BLINKING = 6
- OFF = 7
- UNKNOWN = 8

### Vehicle kinematic state

current position and orientation of ego. Published by the Localization module.

- VehicleKinematicState
  - [nav_msgs/Odometry](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
  - std_msgs/Header header
  - string child_frame_id
  - [geometry_msgs/PoseWithCovariance pose](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html)
  - [geometry_msgs/TwistWithCovariance twist](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovariance.html)

### Lanelet2 Map

map of the environment where the planning takes place. Published by the Map Server.

- [autoware_auto_mapping_msgs/msg/HADMapBin](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_mapping_msgs/msg/HADMapBin.idl)
  - std_msgs::msg::Header header
  - uint8 map_format
  - string format_version
  - string map_version
  - sequence < uint8 > data

### Goal Pose

target pose of ego. Published by the User Interface.

- [geometry_msgs/PoseStamped](https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseStamped.html)

### Engagement Response

TBD.

**The message definition is under discussion.**

### Error status

a status corresponding to the current state of Autoware. Used by the Vehicle Interface to switch between different modes in case of emergency. Published by the Diagnostic Manager.

- [autoware_auto_system_msgs/msg/EmergencyState](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_system_msgs/msg/EmergencyState.idl)
  - builtin_interfaces::msg::Time stamp
  - uint8 state

With the state being one of the following:

- NORMAL = 1
- OVERRIDE_REQUESTING = 2
- MRM_OPERATING = 3
- MRM_SUCCEEDED = 4
- MRM_FAILED = 5

[TODO] original design for these messages: diagnostic manager also publishes an overriding emergency control command ([Add the monitoring system related messages - Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/merge_requests/38)). Possible new design: gate of the vehicle interface switches to the emergency control command (generated by another controller) when receiving an OVERRIDE_REQUESTING message.

**The message definition is under discussion.**

## Outputs

### Traffic Light Query

service request for the state of a specific traffic light. Sent to the Perception module.

- uint64 traffic_light_id

**The message definition is under discussion.**

### Trajectory

A sequence of space and velocity points to be followed by the controller.

- [autoware_auto_planning_msgs/Trajectory](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_planning_msgs/msg/Trajectory.idl)
  - [std_msgs/Header](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html) header
  - sequence<[autoware_auto_planning_msgs::msg::TrajectoryPoint](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_planning_msgs/msg/TrajectoryPoint.idl), 100> points
    - builtin_interfaces::msg::Duration time_from_start
    - [geometry_msgs::msg::Pose](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html) pose
    - float longitudinal_velocity_mps
    - float lateral_velocity_mps
    - float acceleration_mps2
    - float heading_rate_rps
    - float front_wheel_angle_rad
    - float rear_wheel_angle_rad

### Vehicle Signal Commands

Commands for various elements of the vehicle unrelated to motion. Sent to the Vehicle Interface. (For the definition, see [autoware_auto_vehicle_msgs](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/tree/master/autoware_auto_vehicle_msgs/msg).)

- HandBrake Command
- Hazard Lights Command
- Headlights Command
- Horn Command
- Stationary Locking Command
- Turn Indicator Command
- Wipers Command

### Missions Status

TBD.

**The message definition is under discussion.**

### Engagement Request

TBD,

**The message definition is under discussion.**
