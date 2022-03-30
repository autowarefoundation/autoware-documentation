# Control

![Node diagram](images/Control-Bus-ODD-Architecture.drawio.svg)

## Inputs

### Vehicle kinematic state

Current position and orientation of ego. Published by the Localization module.

- [nav_msgs/Odometry](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
  - [std_msgs/Header](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html) header
  - string child_frame_id
  - [geometry_msgs/PoseWithCovariance](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html) pose
  - [geometry_msgs/TwistWithCovariance](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovariance.html) twist

### Trajectory

trajectory to be followed by the controller. See Outputs of Planning.

### Steering Status

Current steering of the ego vehicle. Published by the Vehicle Interface.

- Steering message ([github discussion](https://github.com/autowarefoundation/autoware/discussions/36)).
  - builtin_interfaces::msg::Time stamp
  - float32 steering_angle

### Actuation Status

Actuation status of the ego vehicle for acceleration, steering, and brake.

TODO This represents the reported physical efforts exerted by the vehicle actuators. Published by the Vehicle Interface.

- ActuationStatus ([github discussion](https://github.com/autowarefoundation/autoware/discussions/36)).
  - builtin_interfaces::msg::Time stamp
  - float32 acceleration
  - float32 steering

## Output

### Vehicle Control Command

A motion signal to drive the vehicle, achieved by the low-level controller in the vehicle layer. Used by the Vehicle Interface.

- [autoware_auto_control_msgs/AckermannControlCommand](<https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_control_msgs/msg/AckermannControlCommand.idl>
  - builtin_interfaces::msg::Time stamp
  - [autoware_auto_control_msgs/AckermannLateralCommand](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_control_msgs/msg/AckermannLateralCommand.idl) lateral
    - builtin_interfaces::msg::Time stamp
    - float steering_tire_angle
    - float steering_tire_rotation_rate
  - [autoware_auto_control_msgs/LongitudinalCommand](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_control_msgs/msg/LongitudinalCommand.idl) longitudinal
    - builtin_interfaces::msg::Time stamp
    - builtin_interfaces::msg::Duration duration
    - builtin_interfaces::msg::Duration time_step
    - float[] speeds
    - float[] accelerations
    - float[] jerks
