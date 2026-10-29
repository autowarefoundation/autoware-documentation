# Perception

This page describes the interface of the Perception component. See the [Perception component design](../../components/perception/index.md) for the concepts and the [reference implementation](../../components/perception/reference_implementation.md) for the data flow inside the component.

{{ diagram("images/perception-interface.svg", "The inputs, the main parts, and the outputs of the Perception component.") }}

## Inputs

### From Sensing

| Name          | Topic                                      | Type                                                                                                                                                           | Description                                                                                                                                                                                |
| ------------- | ------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Point cloud   | `/sensing/lidar/concatenated/pointcloud`   | [sensor_msgs/msg/PointCloud2](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg)                                          | The concatenated point cloud of all LiDARs, after preprocessing.                                                                                                                           |
| Camera image  | `/sensing/camera/camera*/image_rect_color` | [sensor_msgs/msg/Image](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg)                                                      | Camera images after lens distortion correction. The camera-based object detectors use them.                                                                                                |
| Camera image  | `/sensing/camera/camera*/image_raw`        | [sensor_msgs/msg/Image](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg)                                                      | Camera images without lens distortion correction. Traffic light recognition uses them.                                                                                                     |
| Radar objects | `/sensing/radar/detected_objects`          | [autoware_perception_msgs/msg/DetectedObjects](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/DetectedObjects.msg) | The objects of all radars in one topic. The sensor kit merges the objects of each radar into it. See [Sensing](sensing.md#outputs). Only the object recognition modes with radar use them. |

### From Map

| Name            | Topic or service                                                          | Type                                                                                                                                                                       | Description                                                                         |
| --------------- | ------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------- |
| Vector map      | `/map/vector_map`                                                         | [autoware_map_msgs/msg/LaneletMapBin](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/msg/LaneletMapBin.msg)                               | The Lanelet2 map. Object filters, prediction, and traffic light recognition use it. |
| Point cloud map | `/map/get_differential_pointcloud_map` (service) or `/map/pointcloud_map` | [autoware_map_msgs/srv/GetDifferentialPointCloudMap](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/srv/GetDifferentialPointCloudMap.srv) | The map filter removes the points that are in the point cloud map.                  |

Use the service for the point cloud map when you can. With the service, the size of the map file does not limit the processing.

### From Localization

| Name             | Topic                           | Type                                                                                                      | Description                               |
| ---------------- | ------------------------------- | --------------------------------------------------------------------------------------------------------- | ----------------------------------------- |
| Vehicle odometry | `/localization/kinematic_state` | [nav_msgs/msg/Odometry](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg) | The pose and the velocity of the vehicle. |

### From Planning

| Name  | Topic                              | Type                                                                                                                                                 | Description                                                                                       |
| ----- | ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| Route | `/planning/mission_planning/route` | [autoware_planning_msgs/msg/LaneletRoute](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/LaneletRoute.msg) | Traffic light recognition uses the route to select the traffic lights that matter to the vehicle. |

### From outside Autoware

| Name                     | Topic                                                            | Type                                                                                                                                                                         | Description                                                                                                                  |
| ------------------------ | ---------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| External traffic signals | `/perception/traffic_light_recognition/external/traffic_signals` | [autoware_perception_msgs/msg/TrafficLightGroupArray](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/TrafficLightGroupArray.msg) | Traffic signals from an external system, for example V2X. The traffic light arbiter merges them with the recognized signals. |

## Outputs

See the [Perception component design](../../components/perception/index.md#high-level-architecture) for the meaning of each output.

| Name               | Topic                                                   | Type                                                                                                                                                                         | Description                                                                                                                        |
| ------------------ | ------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| Dynamic objects    | `/perception/object_recognition/objects`                | [autoware_perception_msgs/msg/PredictedObjects](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/PredictedObjects.msg)             | Objects that were not in the map when it was made, with their class, shape, and predicted paths.                                   |
| Tracked objects    | `/perception/object_recognition/tracking/objects`       | [autoware_perception_msgs/msg/TrackedObjects](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/TrackedObjects.msg)                 | The same objects before prediction. The diffusion planner uses them.                                                               |
| Obstacles          | `/perception/obstacle_segmentation/pointcloud`          | [sensor_msgs/msg/PointCloud2](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg)                                                        | The points that are not ground. They include dynamic objects and static obstacles that the vehicle must avoid or stop in front of. |
| Occupancy grid map | `/perception/occupancy_grid_map/map`                    | [nav_msgs/msg/OccupancyGrid](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg)                                                          | The probability that each cell holds an obstacle. Cells that the sensors cannot see are unknown, which shows the blind spots.      |
| Traffic signals    | `/perception/traffic_light_recognition/traffic_signals` | [autoware_perception_msgs/msg/TrafficLightGroupArray](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/TrafficLightGroupArray.msg) | The state of each traffic light group, for example the color (green, amber, red) and the arrows (left, right, straight).           |

### Predicted objects message

[autoware_perception_msgs/msg/PredictedObjects](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/PredictedObjects.msg):

- std_msgs/Header header
- [PredictedObject](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/PredictedObject.msg)[] objects
  - unique_identifier_msgs/UUID object_id
  - float32 existence_probability
  - [ObjectClassification](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/ObjectClassification.msg)[] classification
    - uint8 label: `UNKNOWN`, `CAR`, `TRUCK`, `BUS`, `TRAILER`, `MOTORCYCLE`, `BICYCLE`, `PEDESTRIAN`, `ANIMAL`, `HAZARD`, `OVER_DRIVABLE`, or `UNDER_DRIVABLE`
    - float32 probability
  - [PredictedObjectKinematics](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/PredictedObjectKinematics.msg) kinematics
    - geometry_msgs/PoseWithCovariance initial_pose_with_covariance
    - geometry_msgs/TwistWithCovariance initial_twist_with_covariance
    - geometry_msgs/AccelWithCovariance initial_acceleration_with_covariance
    - [PredictedPath](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/PredictedPath.msg)[] predicted_paths
      - geometry_msgs/Pose[] path
      - builtin_interfaces/Duration time_step
      - float32 confidence
  - [Shape](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/Shape.msg) shape
    - uint8 type: `BOUNDING_BOX`, `CYLINDER`, or `POLYGON`
    - geometry_msgs/Polygon footprint
    - geometry_msgs/Vector3 dimensions

### Traffic signals message

[autoware_perception_msgs/msg/TrafficLightGroupArray](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/TrafficLightGroupArray.msg):

- builtin_interfaces/Time stamp
- [TrafficLightGroup](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/TrafficLightGroup.msg)[] traffic_light_groups
  - int64 traffic_light_group_id
  - [TrafficLightElement](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/TrafficLightElement.msg)[] elements
    - uint8 color: `UNKNOWN`, `RED`, `AMBER`, `GREEN`, or `WHITE`
    - uint8 shape: `UNKNOWN`, `CIRCLE`, `LEFT_ARROW`, `RIGHT_ARROW`, `UP_ARROW`, `UP_LEFT_ARROW`, `UP_RIGHT_ARROW`, `DOWN_ARROW`, `DOWN_LEFT_ARROW`, `DOWN_RIGHT_ARROW`, or `CROSS`
    - uint8 status: `UNKNOWN`, `SOLID_OFF`, `SOLID_ON`, or `FLASHING`
    - float32 confidence
  - [PredictedTrafficLightState](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/PredictedTrafficLightState.msg)[] predictions
