# Perception Launch Files

## Overview

The Autoware perception stacks start
launching at `autoware_launch.xml` as we mentioned at [Launch Autoware](../index.md) page.
The `autoware_launch` package includes `tier4_perception_component.launch.xml`
for starting perception launch files invocation from `autoware_launch.xml`.
This diagram describes some of the Autoware perception launch files flow at `autoware_launch` and `autoware.universe` packages.

<figure markdown>
  ![perception-launch-flow](images/perception_launch_flow.svg){ align=center }
  <figcaption>
    Autoware perception launch flow diagram
  </figcaption>
</figure>

The Autoware project is large. Therefore, as we manage the Autoware project, we utilize specific
arguments in the launch files. ROS 2 offers an argument-overriding feature for these launch files.
Please refer to [the official ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameter-overrides) for further information. For instance,
if we define an argument at the top-level launch, it will override the value on lower-level launches.

## tier4_perception_component.launch.xml

The `tier4_perception_component.launch.xml` launch file is the main perception component launch at the `autoware_launch` package.
This launch file calls `perception.launch.xml` at [tier4_perception_launch](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/tier4_perception_launch) package from `autoware.universe` repository.
We can modify perception launch arguments at tier4_perception_component.launch.xml.
Also,
we can add any other necessary arguments
that we want
to change it since `tier4_perception_component.launch.xml` is the top-level launch file of other perception launch files.
Here are some predefined perception launch arguments:

- **occupancy_grid_map_method:** This parameter determines the occupancy grid map method for perception stack. Please check [probabilistic_occupancy_grid_map](https://autowarefoundation.github.io/autoware.universe/main/perception/probabilistic_occupancy_grid_map/) package for detailed information.
  The default probabilistic occupancy grid map method is `pointcloud_based_occupancy_grid_map`.
  If you want to change it to the `laserscan_based_occupancy_grid_map`, you can change it here:

  ```diff
  - <arg name="occupancy_grid_map_method" default="pointcloud_based_occupancy_grid_map" description="options: pointcloud_based_occupancy_grid_map, laserscan_based_occupancy_grid_map"/>
  + <arg name="occupancy_grid_map_method" default="laserscan_based_occupancy_grid_map" description="options: pointcloud_based_occupancy_grid_map, laserscan_based_occupancy_grid_map"/>
  ```

- **detected_objects_filter_method:** This parameter determines the filter method for detected objects.
  Please check [detected_object_validation](https://autowarefoundation.github.io/autoware.universe/main/perception/detected_object_validation/) package for detailed information about lanelet and position filter.
  The default detected object filter method is `lanelet_filter`.
  If you want to change it to the `position_filter`, you can change it here:

  ```diff
  - <arg name="detected_objects_filter_method" default="lanelet_filter" description="options: lanelet_filter, position_filter"/>
  + <arg name="detected_objects_filter_method" default="position_filter" description="options: lanelet_filter, position_filter"/>
  ```

- **detected_objects_validation_method:** This parameter determines the validation method for detected objects.
  Please check [detected_object_validation](https://autowarefoundation.github.io/autoware.universe/main/perception/detected_object_validation/) package for detailed information about validation methods.
  The default detected object filter method is `obstacle_pointcloud`.
  If you want to change it to the `occupancy_grid`, you can change it here,
  but remember it requires `laserscan_based_occupancy_grid_map` method as `occupancy_grid_map_method`:

  ```diff
  - <arg name="occupancy_grid_map_method" default="pointcloud_based_occupancy_grid_map" description="options: pointcloud_based_occupancy_grid_map, laserscan_based_occupancy_grid_map"/>
  + <arg name="occupancy_grid_map_method" default="laserscan_based_occupancy_grid_map" description="options: pointcloud_based_occupancy_grid_map, laserscan_based_occupancy_grid_map"/>
    <arg
    name="detected_objects_validation_method"
  - default="obstacle_pointcloud"
  + default="occupancy_grid"
    description="options: obstacle_pointcloud, occupancy_grid (occupancy_grid_map_method must be laserscan_based_occupancy_grid_map)"
    />
  ```

The predefined `tier4_perception_component.launch.xml` arguments explained above,
but there is the lot of perception arguments
included in `perception.launch.xml` at [tier4_perception_launch](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/tier4_perception_launch).
Since we didn't fork `autoware.universe` repository,
we can add the necessary launch argument to tier4_perception_component.launch.xml file.
Please follow the guidelines for some examples.

## perception.launch.xml

The `perception.launch.xml` launch file is the main perception launch at the `autoware.universe`.
This launch file calls necessary perception launch files
as we mentioned [`Autoware perception launch flow diagram`](#overview) above.
The top-level launch file of `perception.launch.xml` is `tier4_perception_component.launch.xml`,
so if we want to change anything on `perception.launch.xml`,
we will apply these changes `tier4_perception_component.launch.xml` instead of `perception.launch.xml`.

Here are some example changes for the perception pipeline:

- **remove_unknown:** This parameter determines the remove unknown objects at camera-lidar fusion.
  Please check [roi_cluster_fusion](https://github.com/autowarefoundation/autoware.universe/blob/main/perception/image_projection_based_fusion/docs/roi-cluster-fusion.md) node for detailed information.
  The default value is `true`.
  If you want to change it to the `false`,
  you can add this argument to `tier4_perception_component.launch.xml`,
  so it will override the `perception.launch.xml`'s `argument`:

  ```diff
  + <arg name="remove_unknown" default="false"/>
  ```

- **camera topics:** If you are using camera-lidar fusion or camera-lidar-radar fusion as a perception_mode,
  you can add your camera and info topics on `tier4_perception_component.launch.xml` as well,
  it will override the `perception.launch.xml` launch file arguments:

  ```diff
  + <arg name="image_raw0" default="/sensing/camera/camera0/image_rect_color" description="image raw topic name"/>
  + <arg name="camera_info0" default="/sensing/camera/camera0/camera_info" description="camera info topic name"/>
  + <arg name="detection_rois0" default="/perception/object_recognition/detection/rois0" description="detection rois output topic name"/>
  ...
  ```

You can add every necessary argument
to `tier4_perception_component.launch.xml` launch file like these examples.
