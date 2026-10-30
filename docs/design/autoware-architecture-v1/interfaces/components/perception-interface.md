# Perception

{{ diagram("images/perception-interface.svg", "The inputs, the main parts, and the outputs of the Perception component.") }}

## Inputs

### PointCloud

PointCloud data published by Lidar.

- [sensor_msgs/msg/PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)

### Image

Image frame captured by camera.

- [sensor_msgs/msg/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)

### Vehicle kinematic state

current position of ego, used in traffic signals recognition. See [output of Localization](localization.md#vehicle-kinematic-state).

### Lanelet2 Map

map of the environment. See [outputs of Map](./map.md#outputs).

## Output

### 3D Object Predictions

3D Objects detected, tracked and predicted by sensor fusing.

- [autoware_perception_msgs/msg/PredictedObjects](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/PredictedObjects.msg)
  - [std_msgs/Header](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html) header
  - sequence<[autoware_perception_msgs::msg::PredictedObject](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/PredictedObject.msg)> objects
    - unique_identifier_msgs::msg::UUID uuid
    - float existence_probability
    - sequence<[autoware_perception_msgs::msg::ObjectClassification](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/ObjectClassification.msg)> classification
      - uint8 classification
      - float probability
    - [autoware_perception_msgs::msg::PredictedObjectKinematics](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/PredictedObjectKinematics.msg) kinematics
      - [geometry_msgs::msg::PoseWithCovariance](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html) initial_pose
      - [geometry_msgs::msg::TwistWithCovariance](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovariance.html)
      - [geometry_msgs::msg::AccelWithCovariance](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/AccelWithCovariance.html) initial_acceleration
    - sequence<[autoware_perception_msgs::msg::PredictedPath](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/PredictedPath.msg), 10> predicted_paths
      - sequence<[geometry_msgs::msg::Pose](https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Pose.html), 100> path
      - builtin_interfaces::msg::Duration time_step
      - float confidence
    - sequence<[autoware_perception_msgs::msg::Shape](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/Shape.msg), 5> shape
      - [geometry_msgs::msg::Polygon](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Polygon.html) polygon
      - float height

### Traffic Signals

traffic signals recognized by object detection model.

- [autoware_perception_msgs::msg::TrafficSignalArray](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/TrafficSignalArray.msg)
  - [autoware_perception_msgs::msg::TrafficSignal](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/TrafficSignal.msg) signals
    - [autoware_perception_msgs::msg::TrafficSignalElement](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/TrafficSignalElement.msg) elements
      - uint8 UNKNOWN = 0
      - uint8 Red = 1
      - uint8 AMBER = 2
      - uint8 WHITE = 4
      - uint8 CIRCLE = 1
      - uint8 LEFT_ARROW = 2
      - uint8 RIGHT_ARROW = 3
      - uint8 UP_ARROW = 4
      - uint8 UP_LEFT_ARROW=5
      - uint8 UP_RIGHT_ARROW=6
      - uint8 DOWN_ARROW = 7
      - uint8 DOWN_LEFT_ARROW = 8
      - uint8 DOWN_RIGHT_ARROW = 9
      - uint8 CROSS = 10
      - uint8 SOLID_OFF = 1
      - uint8 SOLID_ON = 2
      - uint8 FLASHING = 3
      - uint8 color
      - uint8 shape
      - uint8 status
      - float32 confidence
    - int64 traffic_signal_id
  - [builtin_interfaces::msg::Time](https://github.com/ros2/rcl_interfaces/blob/rolling/builtin_interfaces/msg/Time.msg) stamp
