# Perception launch files

## Overview

The Autoware perception stack starts with `autoware.launch.xml`, as the [Launch Autoware](../index.md) page explains. If `launch_perception` is `true`, `autoware.launch.xml` includes two component launch files of the `autoware_launch` package: `component_perception.launch.xml` and `component_traffic_light.launch.xml`. These files include the launch files of the [`autoware_perception_launch`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_perception_launch) package. The following diagram shows the perception launch flow in the `autoware_launch` repository.

{{ diagram("images/perception-launch-flow.svg", "component_perception.launch.xml includes the LiDAR perception pipeline, and component_traffic_light.launch.xml includes one traffic light recognition pipeline.") }}

!!! note

    The Autoware project is a large project.
    Therefore, as we manage the Autoware project, we utilize specific
    arguments in the launch files.
    ROS 2 offers an argument-overriding feature for these launch files.
    Please refer to [the official ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameter-overrides) for further information.
    For instance,
    if we define an argument at the top-level launch,
    it will override the value on lower-level launches.

## component_perception.launch.xml

The [`component_perception.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_launch/launch/components/component_perception.launch.xml) launch file is the main perception launch file of the `autoware_launch` package. It starts the LiDAR pipeline in the `/perception` namespace: the common point cloud downsampling (optional), obstacle segmentation, the occupancy grid map, and object recognition. There is no single `perception.launch.xml` file any more. The parameter files are in the [`autoware_perception_config`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_perception_config) package. You can change the perception launch arguments in `component_perception.launch.xml`. These are some of its arguments:

- **`occupancy_grid_map_method:`** This argument selects the method of the occupancy grid map. For more information, see the [probabilistic_occupancy_grid_map](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_probabilistic_occupancy_grid_map/) package. The options are `pointcloud_based` (the default), `laserscan_based`, and `multi_lidar_pointcloud_based`. To change it to `laserscan_based`, change it here:

  ```diff
  - <arg name="occupancy_grid_map_method" default="pointcloud_based" description="options: pointcloud_based, laserscan_based, multi_lidar_pointcloud_based"/>
  + <arg name="occupancy_grid_map_method" default="laserscan_based" description="options: pointcloud_based, laserscan_based, multi_lidar_pointcloud_based"/>
  ```

- **`detected_objects_filter_method:`** This argument selects the filter method for the detected objects. For more information about the lanelet filter and the position filter, see the [detected_object_validation](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_detected_object_validation/) package. The default filter method is `lanelet_filter`. To change it to `position_filter`, change it here:

  ```diff
  - <arg name="detected_objects_filter_method" default="lanelet_filter" description="options: lanelet_filter, position_filter"/>
  + <arg name="detected_objects_filter_method" default="position_filter" description="options: lanelet_filter, position_filter"/>
  ```

- **`detected_objects_validation_method:`** This argument selects the validation method for the detected objects. For more information about the validation methods, see the [detected_object_validation](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_detected_object_validation/) package. The validator runs only if `use_object_validator` is `true`. Its default value is `false`. The default validation method is `obstacle_pointcloud`. To change it to `occupancy_grid`, enable the validator and use `laserscan_based` as `occupancy_grid_map_method`:

  ```diff
  - <arg name="use_object_validator" default="false" description="use obstacle_pointcloud based object validator"/>
  + <arg name="use_object_validator" default="true" description="use obstacle_pointcloud based object validator"/>
  - <arg name="occupancy_grid_map_method" default="pointcloud_based" description="options: pointcloud_based, laserscan_based, multi_lidar_pointcloud_based"/>
  + <arg name="occupancy_grid_map_method" default="laserscan_based" description="options: pointcloud_based, laserscan_based, multi_lidar_pointcloud_based"/>
  - <arg name="detected_objects_validation_method" default="obstacle_pointcloud" description="options: obstacle_pointcloud, occupancy_grid (must be used with laserscan_based_occupancy_grid_map)"/>
  + <arg name="detected_objects_validation_method" default="occupancy_grid" description="options: obstacle_pointcloud, occupancy_grid (must be used with laserscan_based_occupancy_grid_map)"/>
  ```

- **`lidar_detection_model:`** This argument selects the LiDAR detection model. The default value is `centerpoint`. For the other models, see the [Perception component reference implementation](../../../../design/autoware-architecture-v1/components/perception/reference_implementation.md#launch-files).

- **`use_cuda_ground_segmentation` and `use_semantic_segmentation_ptv3`:** These arguments select another ground segmentation backend: the CUDA version of the rule-based filter, or the PTv3 semantic segmentation. The default value of both is `false`. If `use_semantic_segmentation_ptv3` is `true`, object recognition also uses `object_recognition_lidar_ptv3.launch.xml`.

!!! note

    You can also use these arguments as command-line arguments:
    ```bash
    ros2 launch autoware_launch autoware.launch.xml ... detected_objects_filter_method:=lanelet_filter occupancy_grid_map_method:=laserscan_based ...
    ```

`component_perception.launch.xml` also launches the perception analytics publisher (`use_perception_analytics_publisher`, `true` by default) and the perception online evaluator (`use_perception_online_evaluator`, `false` by default) of `autoware_perception_online_evaluator`.

## Use cameras and radars

The sensor kits of `autoware_launch` have only LiDARs. So `component_perception.launch.xml` uses `object_recognition_lidar.launch.xml`, and Autoware 2.0.0 has no `perception_mode` argument. If your vehicle has cameras or radars, include another object recognition launch file of `autoware_perception_launch` in your fork of `component_perception.launch.xml`, for example `object_recognition_camera_lidar_radar.launch.xml`. For the list of these files and their detectors, see the [Perception component reference implementation](../../../../design/autoware-architecture-v1/components/perception/reference_implementation.md#launch-files).

These launch files have arguments for the camera topics. Give them the topics of your cameras, for example:

```xml
<include file="$(find-pkg-share autoware_perception_launch)/launch/object_recognition/object_recognition_camera_lidar_radar.launch.xml">
  <arg name="number_of_cameras" value="1"/>
  <arg name="input/camera0/image" value="/sensing/camera/camera0/image_rect_color"/>
  <arg name="input/camera0/info" value="/sensing/camera/camera0/camera_info"/>
  <arg name="input/camera0/rois" value="/perception/object_recognition/detection/rois0"/>
  ...
</include>
```

The parameters of the camera-LiDAR fusion nodes are in `autoware_perception_config`. For example, `remove_unknown` of [roi_cluster_fusion](https://github.com/autowarefoundation/autoware_universe/blob/0.53.1/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion.md) is in `config/object_recognition/detection/image_projection_based_fusion/roi_cluster_fusion.param.yaml`. Its value is `false`.

## component_traffic_light.launch.xml

The [`component_traffic_light.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_launch/launch/components/component_traffic_light.launch.xml) launch file starts traffic light recognition in the `/perception/traffic_light_recognition` namespace, if `use_traffic_light_recognition` is `true` (the default). It includes one of three pipelines of `autoware_perception_launch`:

| Arguments                                                                                                                       | Launch file                                        |
| ------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| `traffic_light_recognition/use_high_accuracy_detection:=true` and `high_accuracy_detection_type:=fine_detection` (the defaults) | `traffic_light_recognition_fine.launch.xml`        |
| `traffic_light_recognition/use_high_accuracy_detection:=true` and `high_accuracy_detection_type:=whole_image_detection`         | `traffic_light_recognition_whole_image.launch.xml` |
| `traffic_light_recognition/use_high_accuracy_detection:=false`                                                                  | `traffic_light_recognition_rough.launch.xml`       |

The full name of the second argument is `traffic_light_recognition/high_accuracy_detection_type`. Each pipeline includes a detector launch file and `traffic_light_fusion.launch.xml`. The fine and the rough pipelines also include `traffic_light_occlusion_predictor.launch.xml`. The camera topics are fixed inside each pipeline launch file.

You can add other arguments to `component_perception.launch.xml` and `component_traffic_light.launch.xml` like these examples.
