# Determining component dependencies

For any developers who wish to try and deploy Autoware as a microservices architecture, it is necessary to understand the software dependencies, communication, and implemented features of each ROS package/node.

As an example, the commands necessary to determine the dependencies for the Perception component are shown below. In Autoware 2.0.0, the launch files of the Perception component are in the `autoware_perception_launch` package. This package is in the [autoware_launch](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_universe_launch/autoware_perception_launch) repository.

Run the commands of this page from the root of your Autoware workspace, for example `~/autoware`.

## Perception component dependencies

### Package dependencies

To generate a graph of package dependencies, use the following `colcon` command. The `dot` command comes from the `graphviz` package (`sudo apt install graphviz`).

```bash
colcon graph --dot --legend --packages-up-to autoware_perception_launch | dot -Tpng -o graph.png
```

In the graph, a blue edge is a build dependency, a red edge is a run dependency, a tan edge is a test dependency, and a dashed edge is an indirect dependency. The following diagram groups the packages of this graph for Autoware 2.0.0.

{{ diagram("images/determining-component-dependencies/perception-launch-dependencies.svg", "The packages that autoware_perception_launch depends on in Autoware 2.0.0, grouped by function.") }}

The diagram does not show which package depends on which. To see each dependency, open the `graph.png` file from the `colcon graph` command.

To generate a list of dependencies, use:

```bash
colcon list --packages-up-to autoware_perception_launch --names-only
```

??? example "colcon list output (Autoware 2.0.0, 102 packages)"

    ```txt
    autoware_adapi_v1_msgs
    autoware_agnocast_wrapper
    autoware_bevfusion
    autoware_bytetrack
    autoware_cmake
    autoware_common_msgs
    autoware_compare_map_segmentation
    autoware_crosswalk_traffic_light_estimator
    autoware_cuda_dependency_meta
    autoware_cuda_pointcloud_preprocessor
    autoware_cuda_utils
    autoware_detected_object_feature_remover
    autoware_detected_object_validation
    autoware_detection_by_tracker
    autoware_elevation_map_loader
    autoware_euclidean_cluster
    autoware_grid_map_utils
    autoware_ground_segmentation
    autoware_ground_segmentation_cuda
    autoware_image_object_locator
    autoware_image_projection_based_fusion
    autoware_image_transport_decompressor
    autoware_internal_debug_msgs
    autoware_internal_localization_msgs
    autoware_internal_metric_msgs
    autoware_internal_msgs
    autoware_internal_perception_msgs
    autoware_internal_planning_msgs
    autoware_interpolation
    autoware_kalman_filter
    autoware_lanelet2_extension
    autoware_lanelet2_utils
    autoware_lidar_apollo_instance_segmentation
    autoware_lidar_centerpoint
    autoware_lidar_transfusion
    autoware_lint_common
    autoware_map_based_prediction
    autoware_map_msgs
    autoware_motion_utils
    autoware_multi_object_tracker
    autoware_object_merger
    autoware_object_recognition_utils
    autoware_object_sorter
    autoware_occupancy_grid_map_outlier_filter
    autoware_pcl_extensions
    autoware_perception_config
    autoware_perception_launch
    autoware_perception_msgs
    autoware_planning_msgs
    autoware_point_types
    autoware_pointcloud_preprocessor
    autoware_probabilistic_occupancy_grid_map
    autoware_ptv3
    autoware_pyplot
    autoware_qos_utils
    autoware_raindrop_cluster_filter
    autoware_sensing_msgs
    autoware_shape_estimation
    autoware_simpl_prediction
    autoware_simple_object_merger
    autoware_tensorrt_bevdet
    autoware_tensorrt_classifier
    autoware_tensorrt_common
    autoware_tensorrt_plugins
    autoware_tensorrt_yolox
    autoware_test_utils
    autoware_tracking_object_merger
    autoware_traffic_light_arbiter
    autoware_traffic_light_category_merger
    autoware_traffic_light_classifier
    autoware_traffic_light_fine_detector
    autoware_traffic_light_map_based_detector
    autoware_traffic_light_multi_camera_fusion
    autoware_traffic_light_occlusion_predictor
    autoware_traffic_light_selector
    autoware_traffic_light_utils
    autoware_traffic_light_visualization
    autoware_utils
    autoware_utils_debug
    autoware_utils_diagnostics
    autoware_utils_geometry
    autoware_utils_logging
    autoware_utils_math
    autoware_utils_pcl
    autoware_utils_rclcpp
    autoware_utils_system
    autoware_utils_tf
    autoware_utils_uuid
    autoware_utils_visualization
    autoware_vehicle_info_utils
    autoware_vehicle_msgs
    bevdet_vendor
    cuda_blackboard
    managed_transform_buffer
    mussp
    negotiated
    negotiated_interfaces
    perception_utils
    pointcloud_to_laserscan
    tier4_debug_msgs
    tier4_external_api_msgs
    tier4_perception_msgs
    ```

!!! tip

    To output a list of modules with their respective paths, run the command above without the `--names-only` parameter.

### System dependencies

The packages also depend on ROS packages and system libraries that are not in the Autoware workspace. `rosdep` lists them as rosdep keys. To list the keys of the whole Perception component, give `rosdep` the paths of all packages from the `colcon list` command:

```bash
rosdep keys --ignore-src --from-paths $(colcon list --packages-up-to autoware_perception_launch --paths-only)
```

The `--ignore-src` option removes the packages that are in the given paths. The output contains keys such as `libpcl-all-dev`, `libopencv-dev`, `tensorrt_cmake_module`, `lanelet2_core`, and `grid_map_ros`. To see the system package for one key, use `rosdep resolve`:

```bash
rosdep resolve libpcl-all-dev
```

### Unused dependencies

A `package.xml` file can declare a dependency that the code does not use. The [`autoware_dependency_checker`](https://github.com/autowarefoundation/autoware_tools/tree/main/autoware_dependency_checker) package of `autoware_tools` compares the dependencies in each `package.xml` file with the headers that the source files include. It mainly checks the packages whose names start with `autoware_`. To get it, import `repositories/tools.repos` into your workspace and build it:

```bash
vcs import src < repositories/tools.repos
colcon build --packages-up-to autoware_dependency_checker
source install/setup.bash
```

Then run the script in a package folder or in the workspace root:

```bash
cd src/universe/autoware_universe/perception/autoware_lidar_centerpoint
ros2 run autoware_dependency_checker dependency_checker.sh
```

## Perception component communication

To see which ROS topics are being subscribed and published to, launch Autoware and run `rqt_graph`. For example, start the [rosbag replay simulation](../../simulation/rosbag-replay-simulation.md), and then run:

```bash
ros2 run rqt_graph rqt_graph
```

In `rqt_graph`, enter `/perception` in the namespace filter to show only the nodes and topics of the Perception component.
