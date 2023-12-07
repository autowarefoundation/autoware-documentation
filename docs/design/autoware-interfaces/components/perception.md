# Perception

This page provides specific specifications about the Interface of the Perception Component.  
Please refer to [the perception architecture reference implementation design](../../autoware-architecture/perception/reference_implementation.md) for concepts and data flow.

## Input

### From Map Component

| Name            | Topic / Service                     | Type                                                                                                                                                                       | Description                                  |
| --------------- | ----------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------- |
| Vector Map      | `/map/vector_map`                   | [autoware_auto_mapping_msgs/msg/HADMapBin](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_mapping_msgs/msg/HADMapBin.idl)                       | HD Map including the information about lanes |
| Point Cloud Map | `/service/get_differential_pcd_map` | [autoware_map_msgs/srv/GetDifferentialPointCloudMap](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/srv/GetDifferentialPointCloudMap.srv) | Point Cloud Map                              |

Notes:

- Point Cloud Map
  - input can be both topic or service, but we highly recommend to use service because since this interface enables processing without being constrained by map file size limits.

### From Sensing Component

| Name         | Topic                                      | Type                                                                                                                                                                                          | Description                                                            |
| ------------ | ------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------- |
| Camera Image | `/sensing/camera/camera*/image_rect_color` | [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg)                                                                                          | Camera image data, processed with Lens Distortion Correction (LDC)     |
| Camera Image | `/sensing/camera/camera*/image_raw`        | [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg)                                                                                          | Camera image data, not processed with Lens Distortion Correction (LDC) |
| Point Cloud  | `/sensing/lidar/concatenated/pointcloud`   | [sensor_msgs/PointCloud2](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/PointCloud2.msg)                                                                              | Concatenated point cloud from multiple LiDAR sources                   |
| Radar Object | `/sensing/radar/detected_objects`          | [autoware_auto_perception_msgs/msg/DetectedObject](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/DetectedObject.idl) | Radar objects                                                          |

### From Localization Component

| Name             | Topic                           | Type                                                                                                     | Description                |
| ---------------- | ------------------------------- | -------------------------------------------------------------------------------------------------------- | -------------------------- |
| Vehicle Odometry | `/localization/kinematic_state` | [nav_msgs/msg/Odometry](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg) | Ego vehicle odometry topic |

### From API

| Name                     | Topic                       | Type                                                                                                                                                                   | Description                                 |
| ------------------------ | --------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------- |
| External Traffic Signals | `/external/traffic_signals` | [autoware_perception_msgs::msg::TrafficSignalArray](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/TrafficSignalArray.msg) | The traffic signals from an external system |

## Output

### To Planning

Please refer to [the perception component design](../../autoware-architecture/perception/index.md#high-level-architecture) for detailed definitions of each output."

| Name               | Topic                                                   | Type                                                                                                                                                                     | Description                                                                                                                                                          |
| ------------------ | ------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Dynamic Objects    | `/perception/object_recognition/objects`                | [autoware_auto_perception_msgs/msg/PredictedObjects](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_perception_msgs/msg/PredictedObjects.idl) | Set of dynamic objects with information such as a object class and a shape of the objects. Dynamic objects refer to those that were not present during map creation. |
| Obstacles          | `/perception/obstacle_segmentation/pointcloud`          | [sensor_msgs/PointCloud2](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/PointCloud2.msg)                                                         | Obstacles, including both dynamic objects and static obstacles that requires the ego vehicle either steer clear of them or come to a stop in front of the obstacles. |
| Occupancy Grid Map | `/perception/occupancy_grid_map/map`                    | [nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/latest/api/nav_msgs/html/msg/OccupancyGrid.html)                                                                    | The map with the information about the presence of obstacles and blind spot                                                                                          |
| Traffic Signal     | `/perception/traffic_light_recognition/traffic_signals` | [autoware_perception_msgs::msg::TrafficSignalArray](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/TrafficSignalArray.msg)   | The traffic signal information such as a color (green, yellow, read) and an arrow (right, left, straight)                                                            |
