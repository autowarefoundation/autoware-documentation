# Tuning perception

## Introduction

In this section, we plan to enhance our perception accuracy within the YTU Campus environment
by updating some parameters and methods.
We will enable camera-lidar
fusion as our chosen perception method. This approach holds the potential to significantly
improve our ability to accurately perceive and understand the surroundings, enabling our vehicles
to navigate more effectively and safely within the campus premises. By fine-tuning these perception
parameters, we aim to advance the capabilities of our systems and further optimize their performance
in this specific environment.

## Perception parameter tuning

### Enabling camera-lidar fusion

- To enable camera-lidar fusion, you need to first calibrate both your camera and lidar.
  Your camera driver must publish the rectified image and the camera information.
  By default, Autoware reads them from `/sensing/camera/cameraN/image_rect_color` and `/sensing/camera/cameraN/camera_info`, where `N` is the camera number.

- In Autoware 2.0, the [`component_perception.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/component_perception.launch.xml) file of `autoware_launch` launches only the lidar pipeline (`object_recognition_lidar.launch.xml`).
  The [`autoware_perception_launch`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_perception_launch) package has one object recognition launch file for each sensor mode.
  The `object_recognition_camera_lidar_radar.launch.xml` file fuses the camera, lidar, and radar detections.
  To use it, change the include in your copy of `component_perception.launch.xml`:

!!! note "[`component_perception.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/component_perception.launch.xml) launch file:"

    ```diff
    - <include file="$(find-pkg-share autoware_perception_launch)/launch/object_recognition/object_recognition_lidar.launch.xml">
    + <include file="$(find-pkg-share autoware_perception_launch)/launch/object_recognition/object_recognition_camera_lidar_radar.launch.xml">
    +   <arg name="number_of_cameras" value="<YOUR-CAMERA-NUMBER>"/>
        <arg name="lidar_detection_model" value="$(var lidar_detection_model)"/>
        ...
    ```

Please check the arguments of [`object_recognition_camera_lidar_radar.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_perception_launch/launch/object_recognition/object_recognition_camera_lidar_radar.launch.xml) before you pass them.
Some arguments of the lidar launch file do not exist in this file. For example, it always uses the lanelet filter for the objects from the clustering.
This file also launches the radar pipeline for the `/sensing/radar/detected_objects` topic.

- The camera-lidar fusion needs 2D detections (ROIs) from the cameras.
  The [TensorRT YOLOX node](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_tensorrt_yolox) makes them.
  By default, Autoware expects that YOLOX runs on another computer, and it reads the ROIs from `/perception/object_recognition/detection/roisN`.
  If you run YOLOX on the same computer, set the `enable_2d_detection` argument to `true` in [`camera_lidar_detector.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_perception_launch/launch/object_recognition/detection/detector/camera_lidar_detector.launch.xml).
  Then it launches `camera_2d_detector.launch.xml` for your cameras.
  The models must be in `$HOME/autoware_data/ml_models/tensorrt_yolox/`.

- Also, you need to update the `fusion_common.param.yaml` parameter file according to your camera number.
  Firstly,
  please refer to the [image_projection_based_fusion documentation](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/) for more information about these parameters.
  Then, you will update your camera offsets.
  For example,
  if you have four cameras for the perception detection pipeline,
  and you haven't measured their timestamps,
  you can set these camera offsets to "0" as the initial value.
  Please be careful with the array sizes. They must be equal to your camera count.

!!! note "[fusion_common.param.yaml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_perception_config/config/object_recognition/detection/image_projection_based_fusion/fusion_common.param.yaml) parameter file:"

    ```diff
    - rois_timestamp_offsets: [0.098, 0.147, 0.078, 0.062, 0.115, 0.132] # {OVERRIDE}
    + rois_timestamp_offsets: [0.0, 0.0, 0.0, 0.0] # {OVERRIDE}
    ...
    - point_project_to_unrectified_image: [false, false, false, false, false, false]
    + point_project_to_unrectified_image: [false, false, false, false]
    ...
    - approximate_camera_projection: [true, true, true, true, true, true]
    + approximate_camera_projection: [true, true, true, true]
    ...
    -   rois_timestamp_noise_window: [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
    +   rois_timestamp_noise_window: [0.01, 0.01, 0.01, 0.01]
    ```

- If you have used different namespaces for your camera and ROI topics,
  you will need to give the input topics for the image, the camera information,
  and the ROIs to `object_recognition_camera_lidar_radar.launch.xml`.

!!! note "[`object_recognition_camera_lidar_radar.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_perception_launch/launch/object_recognition/object_recognition_camera_lidar_radar.launch.xml) launch file:"

    ```diff
    - <arg name="input/camera0/image" default="/sensing/camera/camera0/image_rect_color"/>
    + <arg name="input/camera0/image" default="<YOUR-CAMERA-TOPIC-NAME>"/>
    - <arg name="input/camera0/info" default="/sensing/camera/camera0/camera_info"/>
    + <arg name="input/camera0/info" default="<YOUR-CAMERA-INFO-TOPIC-NAME>"/>
    - <arg name="input/camera0/rois" default="/perception/object_recognition/detection/rois0"/>
    + <arg name="input/camera0/rois" default="<YOUR-ROIS-TOPIC-NAME>"/>
    ```

### Tuning ground segmentation

- The ground segmentation package removes the ground points from the input point cloud for the perception pipeline.
  In our campus environment, there are a lot of high slopes and rough roads.
  Therefore, this condition makes it difficult to accurately segment ground and non-ground points.

- For example, when we pass over speed bumps,
  there are a lot of false positives (ghost points) that appear as non-ground points,
  as shown in the image below.

<figure markdown>
  ![default-ground-segmentation](images/ground-remover-ghost-points.png){ align=center }
  <figcaption>
    There are some false positive (ghost)
points on the high-slope roads with default configurations.
  </figcaption>
</figure>

- These ghost points affect the motion planner of Autoware,
  causing the vehicle to stop even though there is no obstacle on the road during autonomous driving.
  We will reduce the number of false positive non-ground points
  by fine-tuning the ground segmentation in Autoware.

- There are three different ground segmentation algorithms included in Autoware:
  `ray_ground_filter`, `scan_ground_filter`, and `ransac_ground_filter`.
  The default method is the `scan_ground_filter`.
  `ground_segmentation_rule_based.launch.xml` of `autoware_perception_launch` launches it.
  Please refer to the [`ground_segmentation` package documentation](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_ground_segmentation/)
  for more information about these methods and their parameter definitions.

- Firstly,
  we will change the `global_slope_max_angle_deg` value from 10 to 30 degrees at [`scan_ground_filter.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_perception_config/config/obstacle_segmentation/ground_segmentation/scan_ground_filter.param.yaml) parameter file.
  This change will reduce our false positive non-ground points.
  However, be cautious when increasing the threshold,
  as it may lead to an increase in the number of false negatives.

!!! note "[`scan_ground_filter.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_perception_config/config/obstacle_segmentation/ground_segmentation/scan_ground_filter.param.yaml) parameter file:"

    ```diff
    - global_slope_max_angle_deg: 10.0
    + global_slope_max_angle_deg: 30.0
    ```

- Then we will update the split_height_distance parameter from 0.2 to 0.35 meters.
  This adjustment will help in reducing false positive non-ground points,
  especially on step-like road surfaces or in cases of misaligned multiple lidar configurations.

!!! note "[`scan_ground_filter.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_perception_config/config/obstacle_segmentation/ground_segmentation/scan_ground_filter.param.yaml) parameter file:"

    ```diff
    - split_height_distance: 0.2
    + split_height_distance: 0.35
    ```

- Now, we will change the non_ground_height_threshold value from 0.2 to 0.3 meters.
  This will help us in reducing false positive non-ground points,
  but it may also decrease the number of true positive non-ground points
  that are below this threshold value.

!!! note "[`scan_ground_filter.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_perception_config/config/obstacle_segmentation/ground_segmentation/scan_ground_filter.param.yaml) parameter file:"

    ```diff
    - non_ground_height_threshold: 0.20
    + non_ground_height_threshold: 0.3
    ```

- The following image illustrates the results after these fine-tunings with the ground remover package.

<figure markdown>
  ![after-ground-segmentation](images/after-tuning-ground-segmentation.png){ align=center }
  <figcaption>
    After tuning the ground segmentation,
the false positive points will disappear from the same location.
  </figcaption>
</figure>

- You need to update the ground segmentation according to your environment.
  These examples are provided for high slopes and rough road conditions.
  If you have better conditions,
  you can adjust your parameters
  by referring to the [`ground_segmentation` package documentation page](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_ground_segmentation/).

### Tuning euclidean clustering

- The `euclidean_clustering` package applies Euclidean clustering methods
  to cluster points into smaller parts for classifying objects.
  Please refer to [`euclidean_clustering` package documentation](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_euclidean_cluster) for more information.
  This package is used in the detection pipeline of Autoware architecture.
  There are two different euclidean clustering methods included in this package:
  `euclidean_cluster` and `voxel_grid_based_euclidean_cluster`.
  In the default design of Autoware,
  the `voxel_grid_based_euclidean_cluster` method serves as the default Euclidean clustering method.

- In the YTU campus environment, there are many small objects like birds,
  dogs, cats, balls, cones, etc. To detect, track,
  and predict these small objects, we aim to assign clusters to them as small as possible.

- Firstly, we will change our object filter method from lanelet_filter to position_filter
  to detect objects that are outside the lanelet boundaries at the [`component_perception.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/component_perception.launch.xml).
  This argument is used by the lidar pipeline (`object_recognition_lidar.launch.xml`).

!!! note "[`component_perception.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/component_perception.launch.xml) launch file:"

    ```diff
    - <arg name="detected_objects_filter_method" default="lanelet_filter" description="options: lanelet_filter, position_filter"/>
    + <arg name="detected_objects_filter_method" default="position_filter" description="options: lanelet_filter, position_filter"/>
    ```

- After changing the filter method for objects,
  the output of our perception pipeline looks like the image below:

<figure markdown>
  ![default-clustering](images/initial-clusters.png){ align=center }
  <figcaption>
    The default clustering parameters with the object position filter default parameters.
  </figcaption>
</figure>

- Now, we can detect unknown objects that are outside the lanelet map,
  but we still need to update the filter range
  or disable the filter for unknown objects in the [`object_position_filter.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_perception_config/config/object_recognition/detection/object_filter/object_position_filter.param.yaml) file.

!!! note "[`object_position_filter.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_perception_config/config/object_recognition/detection/object_filter/object_position_filter.param.yaml) parameter file:"

    ```diff
        upper_bound_x: 100.0
    -   lower_bound_x: 0.0
    +   lower_bound_x: -100.0
    -   upper_bound_y: 10.0
    +   upper_bound_y: 100.0
    -   lower_bound_y: -10.0
    +   lower_bound_y: -100.0
    ```

- Also, you can simply disable the filter for unknown labeled objects.

!!! note "[`object_position_filter.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_perception_config/config/object_recognition/detection/object_filter/object_position_filter.param.yaml) parameter file:"

    ```diff
    - UNKNOWN : true
    + UNKNOWN : false
    ```

- After that,
  we can update our clustering parameters
  since we can detect all objects regardless of filtering objects with the lanelet2 map.
  As we mentioned earlier, we want to detect small objects.
  Therefore,
  we will decrease the minimum number of points in a cluster (`min_points_per_cluster`) to 1 in the `voxel_grid_based_euclidean_cluster.param.yaml` file.

!!! note "[`voxel_grid_based_euclidean_cluster.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_perception_config/config/object_recognition/detection/clustering/voxel_grid_based_euclidean_cluster.param.yaml) parameter file:"

    ```diff
    - min_points_per_cluster: 10
    + min_points_per_cluster: 1
    ```

- After making these changes, our perception output is shown in the following image:

<figure markdown>
  ![update-clustering](images/after-tuning-clustering.png){ align=center }
  <figcaption>
    The minimum number of points in a cluster is set to "1" with the unknown object filter disabled.
  </figcaption>
</figure>

If you want to use an object filter after fine-tuning clusters for unknown objects,
you can utilize either the lanelet filter or the position filter for unknown objects.
Please refer to the documentation of the [`detected_object_validation` package page](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_detected_object_validation/) for further information.
