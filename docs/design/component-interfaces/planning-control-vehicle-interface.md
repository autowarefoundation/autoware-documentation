# Autoware.core/universe Architecture

## Planning

![Node diagram](images/Planning-Bus-ODD-Architecture.drawio.svg)

### Inputs

#### 3D Object Predictions

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

#### (NEW) Traffic Light Response

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

#### Vehicle kinematic state

current position and orientation of ego. Published by the Localization module.

- Vehiclekinematicstate
  - [nav_msgs/Odometry](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
  - std_msgs/Header header
  - string child_frame_id
  - [geometry_msgs/PoseWithCovariance pose](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html)
  - [geometry_msgs/TwistWithCovariance twist](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovariance.html)

#### Lanelet2 Map

map of the environment where the planning takes place. Published by the Map Server.

- [autoware_auto_mapping_msgs/msg/HADMapBin](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_mapping_msgs/msg/HADMapBin.idl)
  - std_msgs::msg::Header header
  - uint8 map_format
  - string format_version
  - string map_version
  - sequence < uint8 > data

#### Goal Pose

target pose of ego. Published by the User Interface.

- [geometry_msgs/PoseStamped](https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseStamped.html)

#### (NEW) Engagement Response

TBD.

**The message definition is under discussion.**

#### (NEW) Error status

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

### Outputs

#### (NEW) Traffic Light Query

service request for the state of a specific traffic light. Sent to the Perception module.

- uint64 traffic_light_id

**The message definition is under discussion.**

#### Trajectory

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

#### Vehicle Signal Commands

Commands for various elements of the vehicle unrelated to motion. Sent to the Vehicle Interface. (See [autoware_auto_vehicle_msgs](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/tree/master/autoware_auto_vehicle_msgs/msg) for the definition.)

- HandBrake Command
- Hazard Lights Command
- Headlights Command
- Horn Command
- Stationary Locking Command
- Turn Indicator Command
- Wipers Command

#### (NEW) Missions Status

TBD.

**The message definition is under discussion.**

#### (NEW) Engagement Request

TBD,

**The message definition is under discussion.**

## Control

![Node diagram](images/Control-Bus-ODD-Architecture.drawio.svg)

### Inputs

#### Vehicle kinematic state

Current position and orientation of ego. Published by the Localization module.

- [nav_msgs/Odometry](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
  - [std_msgs/Header](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html) header
  - string child_frame_id
  - [geometry_msgs/PoseWithCovariance](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html) pose
  - [geometry_msgs/TwistWithCovariance](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovariance.html) twist

#### Trajectory

trajectory to be followed by the controller. See Outputs of Planning.

#### (NEW) Steering Status

Current steering of the ego vehicle. Published by the Vehicle Interface.

- Steering message ([github discussion](https://github.com/autowarefoundation/autoware/discussions/36)).
  - builtin_interfaces::msg::Time stamp
  - float32 steering_angle

#### (NEW) Actuation Status

Actuation status of the ego vehicle for acceleration, steering, and brake.

TODO This represents the reported physical efforts exerted by the vehicle actuators. Published by the Vehicle Interface.

- ActuationStatus ([github discussion](https://github.com/autowarefoundation/autoware/discussions/36)).
  - builtin_interfaces::msg::Time stamp
  - float32 acceleration
  - float32 steering

### Output

#### Vehicle Control Command

A motion signal to drive the vehicle, achieved by the low-level controller in the vehicle layer. Used by the Vehicle Interface.

- [autoware_auto_control_msgs/AckermannControlCommand](<https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_control_msgs/msg/AckermannControlCommand.idl>
  - builtin_interfaces::msg::Time stamp
  - [autoware_auto_control_msgs/AckermannLateralCommand](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_control_msgs/msg/AckermannLateralCommand.idl) lateral
    - builtin_interfaces::msg::Time stamp
    - float steering_tire_angle
    - float steering_tire_rotation_rate
  - [autoware_auto_control_msgs/LongitudinalCommand](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_control_msgs/msg/LongitudinalCommand.idl) longitudinal (CHANGED)
    - builtin_interfaces::msg::Time stamp
    - builtin_interfaces::msg::Duration duration
    - builtin_interfaces::msg::Duration time_step
    - float[] speeds
    - float[] accelerations
    - float[] jerks

## Vehicle Interface

![Node diagram](images/Vehicle-Interface-Bus-ODD-Architecture.drawio.svg)

The `Vehicle Interface` receives the `Vehicle Signal Commands` and `Vehicle Control Commands` and publishes the vehicle status. It also communicates with vehicle by the vehicle-specific protocol.

The `Gate` switches multiple `Vehicle Control Commands`. These signals include autonomous diving command, joystick, remote control, and emergency operation, etc.
The `Adapter` converts generalized control command (target steering, steering rate, velocity, acceleration, jerk) into vehicle-specific control values (steering-torque, wheel-turque, voltage, pressure, accel pedal position, etc).

### Inputs

#### (NEW) Error status

(See Inputs of Planning.)

#### Vehicle Control Command

(See Output of Control.)

#### Vehicle Signals Commands

Commands for various elements of the vehicle unrelated to motion. Published by the Planning module.

### Outputs

#### Vehicle Signal Reports

Reports for various elements of the vehicle unrelated to motion. Published by the Vehicle Interface.

#### Vehicle Odometry

Odometry of the vehicle. Used by the Localization module to update the pose of the vehicle in the map.

- [geometry_msgs/TwistWithCovarianceStamped](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html) odometry

#### Vehicle Communication

Vehicle specific messages. The adapter converts Autoware control and signal commands into signals that the vehicle can understand.

#### (NEW) Steering Status

Steering of the ego vehicle. Published by the Vehicle Interface.

- Steering message ([github discussion](https://github.com/autowarefoundation/autoware/discussions/36)).
  - builtin_interfaces::msg::Time stamp
  - float32 steering_angle

#### (NEW) Actuation Status

Actuation status of the ego vehicle for acceleration, steering, and brake. TODO This represents the reported physical efforts exerted by the vehicle actuators. Published by the Vehicle Interface.

- ActuationStatus ([github discussion](https://github.com/autowarefoundation/autoware/discussions/36)).
  - builtin_interfaces::msg::Time stamp
  - float32 acceleration
  - float32 steering
  - float32 brake

#### (NEW) Actuation Command

Actuation command sent to the ego vehicle. TODO This represents the requested physical efforts to be exerted by the vehicle actuators. Published by the Vehicle Interface as generated by the adapter.

- ActuationCommand ([github discussion](https://github.com/autowarefoundation/autoware/discussions/36).)
  - builtin_interfaces::msg::Time stamp
  - float32 acceleration
  - float32 steering
  - float32 brake
