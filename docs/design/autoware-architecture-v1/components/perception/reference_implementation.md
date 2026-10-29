# Perception component reference implementation design

## Purpose of this document

This document describes the reference implementation of the Perception component in Autoware 2.0.0. It helps developers and users to understand what the Perception component can do today, and how to use, expand, or add to its features.

## Launch files

`autoware_launch/launch/autoware.launch.xml` starts the Perception component with two component launch files of [`autoware_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/launch/components):

- `component_perception.launch.xml` starts the LiDAR pipeline: the common point cloud preprocessing, obstacle segmentation, the occupancy grid map, and object recognition.
- `component_traffic_light.launch.xml` starts traffic light recognition.

Both files include the feature launch files of [`autoware_perception_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_universe_launch/autoware_perception_launch). The parameter files are in [`autoware_perception_config`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_universe_launch/autoware_perception_config). To use a different parameter tree, set the `perception_config_pkg` launch argument.

Object recognition has one launch file for each sensor setup. `component_perception.launch.xml` uses `object_recognition_lidar.launch.xml`, or `object_recognition_lidar_ptv3.launch.xml` when `use_semantic_segmentation_ptv3` is `true`. A product launch file can include one of the other files:

| Launch file in `launch/object_recognition/`               | Detectors                                                                                                                                                     |
| :-------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `object_recognition_lidar.launch.xml`                     | LiDAR DNN detector and LiDAR clustering. This is the default.                                                                                                 |
| `object_recognition_lidar_ptv3.launch.xml`                | LiDAR DNN detector and clustering of the PTv3 semantic segmentation.                                                                                          |
| `object_recognition_lidar_radar.launch.xml`               | The PTv3 setup plus the radar faraway detector.                                                                                                               |
| `object_recognition_camera_lidar_radar.launch.xml`        | LiDAR DNN detector, camera-LiDAR fusion clustering, irregular object detector, and radar faraway detector. Camera VRU and camera-only detectors are optional. |
| `object_recognition_camera_lidar_radar_serial.launch.xml` | The same detectors, but object mergers combine them before the tracker. Radar tracks join after the tracker.                                                  |
| `object_recognition_camera.launch.xml`                    | Camera-only BEV detector (BEVDet) and an optional camera VRU detector.                                                                                        |

The `lidar_detection_model` launch argument selects the LiDAR DNN detector. Its value is `model_type/model_name`. The model types are `centerpoint` (default), `transfusion`, `bevfusion`, `pointpainting`, `apollo`, and `clustering`.

## Whole architecture

This diagram shows the architecture of the reference implementation.

{{ diagram("image/reference-implementation-perception-diagram.svg", "The sub-components of the reference implementation and their inputs and outputs.") }}

The Perception component has these sub-components:

- **Obstacle segmentation**: Finds the points that come from obstacles that the ego vehicle must avoid. These are dynamic objects and also static obstacles, for example construction cones. By default, a crop box filter and the scan ground filter of [`autoware_ground_segmentation`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_ground_segmentation) remove the ground. Two other backends are available: a CUDA version ([`autoware_ground_segmentation_cuda`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_ground_segmentation_cuda)) and the PTv3 semantic segmentation ([`autoware_ptv3`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_ptv3)).
- **Occupancy grid map**: Finds blind spots. In a blind spot, no information is available and dynamic objects can jump out. [`autoware_probabilistic_occupancy_grid_map`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_probabilistic_occupancy_grid_map) makes the map from the raw point cloud and the obstacle point cloud. The `occupancy_grid_map_method` launch argument selects `pointcloud_based` (default), `laserscan_based`, or `multi_lidar_pointcloud_based`.
- **Object recognition**: Recognizes dynamic objects around the ego vehicle in the current frame and predicts their future trajectories.
  - **Detection**: Finds the pose, shape, and class of dynamic objects such as vehicles and pedestrians.
    - **Detector**: Processes each frame and outputs detected objects.
    - **Interpolator**: Keeps the detection stable. If the output of a detector stops for a short time, the interpolator uses the tracking output to keep the objects. [`autoware_detection_by_tracker`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_detection_by_tracker) does this in the serial launch file. In the other launch files, the multi-object tracker keeps its tracks when one detector misses an object.
  - **Tracking**: Associates the detected objects across frames, gives each object an ID, and estimates its velocity.
  - **Prediction**: Predicts the trajectories of dynamic objects.
- **Traffic light recognition**: Recognizes the colors of traffic lights and the directions of arrow signals.

### Outputs to Planning

| Output             | Topic                                                   | Message type                                          |
| :----------------- | :------------------------------------------------------ | :---------------------------------------------------- |
| Dynamic objects    | `/perception/object_recognition/objects`                | `autoware_perception_msgs/msg/PredictedObjects`       |
| Obstacle points    | `/perception/obstacle_segmentation/pointcloud`          | `sensor_msgs/msg/PointCloud2`                         |
| Occupancy grid map | `/perception/occupancy_grid_map/map`                    | `nav_msgs/msg/OccupancyGrid`                          |
| Traffic signals    | `/perception/traffic_light_recognition/traffic_signals` | `autoware_perception_msgs/msg/TrafficLightGroupArray` |

### Internal interface in the perception component

- **Obstacle segmentation to object recognition**
  - Obstacle point cloud: the point cloud of the current frame without the ground. The pointcloud map filter of object recognition then removes the points that match the point cloud map. LiDAR clustering uses the result.
- **Obstacle segmentation to occupancy grid map**
  - Obstacle point cloud: the same point cloud. The occupancy grid map also uses the raw point cloud to find free space.
- **Occupancy grid map to obstacle segmentation and object recognition**
  - Occupancy grid map: [`autoware_occupancy_grid_map_outlier_filter`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_occupancy_grid_map_outlier_filter) can use the map to remove outlier points, but the launch files of Autoware 2.0.0 do not start it. The object validator of object recognition can also use the map (`objects_validation_method:=occupancy_grid`).

## Traffic light recognition

{{ diagram("image/traffic-light-recognition.svg", "The traffic light recognition pipeline with fine detection.") }}

By default, `component_traffic_light.launch.xml` runs `traffic_light_recognition_fine.launch.xml` for the cameras `camera6` and `camera7`. For each camera, these nodes run:

1. [`autoware_traffic_light_map_based_detector`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_map_based_detector) projects the traffic lights of the vector map on the route into the image, and outputs rough regions of interest (ROIs).
2. [`autoware_traffic_light_fine_detector`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_fine_detector) finds the exact traffic light ROIs in the rough ROIs with a YOLOX model.
3. Two [`autoware_traffic_light_classifier`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_classifier) nodes find the colors and arrows of the car signals and of the pedestrian signals.
4. [`autoware_traffic_light_occlusion_predictor`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_occlusion_predictor) uses the LiDAR point cloud to mark the signals that an object hides from the camera.

Then the results of all cameras go through these nodes:

1. [`autoware_traffic_light_multi_camera_fusion`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_multi_camera_fusion) combines the results of the cameras.
2. [`autoware_traffic_light_arbiter`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_arbiter) combines the camera result with the V2X signals on `/perception/traffic_light_recognition/external/traffic_signals`.
3. [`autoware_crosswalk_traffic_light_estimator`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_crosswalk_traffic_light_estimator) estimates the states of pedestrian signals that the cameras cannot see. It publishes the final result.

Two other detection modes are available. With `traffic_light_recognition/use_high_accuracy_detection:=false`, the classifiers use the rough ROIs directly. With `traffic_light_recognition/high_accuracy_detection_type:=whole_image_detection`, a YOLOX model finds the traffic lights in the whole image.

## Architecture for object recognition

{{ diagram("image/new_autoware_design.svg", "The detectors feed the multi-object tracker through separate channels.") }}

Each detector publishes its own `DetectedObjects` topic. The multi-object tracker takes each topic as one input channel. The tracker combines the channels, so most launch files do not need an object merger. The channel name tells the tracker how much it can trust the channel (see [Multi-object tracking](#multi-object-tracking)).

### Base 3D detection

Autoware primarily uses ML-based methods for 3D detection, referred to as **Base 3D Detection**.
Available methods include:

- [CenterPoint](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_lidar_centerpoint) (default)
- [TransFusion-L](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_lidar_transfusion)
- [BEVFusion-L](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_bevfusion)
- [Apollo instance segmentation](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_lidar_apollo_instance_segmentation) + [shape estimation](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_shape_estimation)

The detection range depends on the model. The default model parameters cover a square of ±76.8 m around the vehicle for CenterPoint, ±92.16 m for TransFusion, and ±122.4 m for BEVFusion.

If you want to use camera-LiDAR fusion, you can use the camera-LiDAR model of BEVFusion (`bevfusion_camera_lidar`) or [PointPainting](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_image_projection_based_fusion) (`lidar_detection_model:=pointpainting`, in the camera-LiDAR launch files).
`Base 3D Detection` is a critical component of the architecture, so it needs stable performance.
For this reason, the design recommends against camera-LiDAR fusion methods in environments where the camera data often drops. With these methods, a camera drop affects the base detection:

- The camera-LiDAR model of BEVFusion skips the detection until each camera sent one image. After that, it uses the last image of each camera, also when the image is old ([`bevfusion_node.cpp`](https://github.com/autowarefoundation/autoware_universe/blob/main/perception/autoware_bevfusion/src/bevfusion_node.cpp)).
- The fusion node of PointPainting waits for the 2D detections of the cameras until a timer expires. Then it fuses the point cloud with the detections that arrived ([`autoware_image_projection_based_fusion`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_image_projection_based_fusion#inner-workings--algorithms)).

The output topic is `/perception/object_recognition/detection/<model_type>/objects`. The tracker channel is `lidar_<model_type>`, for example `lidar_centerpoint`. If `use_validator` is `true`, an object validator of [`autoware_detected_object_validation`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_detected_object_validation) removes the objects that have no obstacle points.

### Near-object 3D detection

As optional method for detection to enhance detection of nearby objects, especially pedestrians and cyclists, we have introduced **Near-Object 3D Detection**.
This can serve as a supplementary detection method alongside `Base 3D Detection`.

To enable it, set `use_short_range_detection:=true`. Autoware then starts a second CenterPoint node (`lidar_short_range_centerpoint`) with the short-range model. Its tracker channel is `lidar_centerpoint_short_range`.
The short-range model uses a higher-resolution voxel grid to detect small objects better. In the `lidar_centerpoint` model bundle v4.1, the short-range model uses 0.16 m voxels and covers a square of ±51.2 m around the vehicle. The base model uses 0.32 m voxels and covers ±76.8 m. The `README.md` of the [model bundle on Hugging Face](https://huggingface.co/AutowareFoundation/lidar_centerpoint) lists the parameters of each model.

### Camera-only 3D detection

**Camera-only 3D detection** finds objects that LiDAR-based methods detect badly, for example objects behind trees and objects far away.

- `object_recognition_camera.launch.xml` runs [BEVDet](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_tensorrt_bevdet) on six cameras. Its output goes to the tracker channel `camera_streampetr`.
- In `object_recognition_camera_lidar_radar.launch.xml`, set `use_camera_only_detection:=true` to give the tracker the objects of an external camera-only detector. By default, the topic is `/perception/object_recognition/detection/camera_only/objects` and the channel is `camera_streampetr`.
- Other camera-only detectors are available as packages: [StreamPETR](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_camera_streampetr) and [BEVFormer](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_tensorrt_bevformer).

The design plans a high-confidence threshold for `Camera-Only 3D Detection`, to reduce the effect of false positives. Autoware 2.0.0 does not apply such a threshold. BEVDet keeps the objects with a score of at least 0.2 (`score_threshold` in `bevdet.param.yaml` of `autoware_tensorrt_bevdet`). The base CenterPoint model uses 0.35. The tracker can start new tracks from the `camera_streampetr` channel and trusts its existence probability (`input_channels.param.yaml`).

The **camera VRU detector** (`use_camera_vru_detector:=true`) is a related option. VRU means vulnerable road users, for example pedestrians and cyclists. [`autoware_image_object_locator`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_image_object_locator) makes 3D objects from the 2D boxes of the camera detector. Its tracker channel is `camera_vru`.

### Radar-only faraway object 3D detection

For enhanced detection of distant objects, we use **Radar-Only 3D Detection**. The radar launch files filter the radar objects with [`autoware_object_sorter`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_object_sorter) and the object lanelet filter. They publish the result on `/perception/object_recognition/detection/radar/far_objects`, and the tracker reads it on the `radar` channel.
For more details, see [the document on faraway radar object detection](reference-implementations/radar-based-3d-detector/faraway-object-detection.md).

### 3D semantic segmentation

**3D semantic segmentation** improves the detection of objects that 3D detectors find badly, especially vegetation and traffic cones. It gives a class label to each point. Then it outputs the non-ground point cloud and the labeled point cloud.

Available methods include:

- [PTv3](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_ptv3): set `use_semantic_segmentation_ptv3:=true`. PTv3 then also replaces the ground removal of obstacle segmentation.
- [FRNet](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_lidar_frnet): available as a package. The launch files of Autoware 2.0.0 do not start it.

To make Autoware objects from the labeled points, the label-based Euclidean clustering of [`autoware_euclidean_cluster`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_euclidean_cluster) groups the points of each label. Its tracker channel is `lidar_clustering`.

### Cluster-based 3D detection

To enhance detection of objects that LiDAR-based methods detect badly, we offer **Cluster-Based 3D Detection**.
`Cluster-Based 3D Detection` consists of many nodes. The pipeline is as follows.

{{ diagram("image/clustering_based_detection.svg", "LiDAR clustering and camera-LiDAR fusion clustering.") }}

`Cluster-Based 3D Detection` is based on Euclidean clustering of the non-ground LiDAR points:

- **LiDAR clustering** (`object_recognition_lidar.launch.xml`): The clusters become objects with a convex hull shape. The object lanelet filter keeps the objects on the lanes. The tracker channel is `lidar_clustering`.
- **Camera-LiDAR fusion clustering** (camera-LiDAR launch files): [ROI cluster fusion](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_image_projection_based_fusion) gives each cluster the class of the 2D detection (YOLOX) that it falls into. The low-intensity cluster filter of [`autoware_raindrop_cluster_filter`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_raindrop_cluster_filter) removes clusters from raindrops. An optional image segmentation filter removes points of classes such as the sky before the clustering. The tracker channel is `camera_lidar_fusion`.
- **Irregular object detector** (`use_irregular_object_detector`, `true` by default in the camera-LiDAR launch files): ROI point cloud fusion finds small unknown objects in the 2D detections. The tracker channel is `camera_lidar_fusion_irregular`.

This can be used as supplementary detection alongside `Base 3D Detection`.
In the serial launch file, [`autoware_detection_by_tracker`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_detection_by_tracker) also uses the clusters to keep the tracked objects, and [`autoware_object_merger`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_object_merger) combines the results.

The processing time of this pipeline increases with the number of points and images.
For this reason, the design recommends that you do not use this pipeline when the processing time is critical, or that you use it only for a narrow detection range. The default launch files do not narrow the range of the clustering. Its crop box is off by default (`use_low_height_cropbox:=false`). When it is on, it covers ±200 m in x and y (`voxel_grid_based_euclidean_cluster.param.yaml` of `autoware_perception_config`).

### Multi-object tracking

[`autoware_multi_object_tracker`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_multi_object_tracker) gives each object an ID and estimates its velocity. It works as follows:

{{ diagram("image/multi_object_tracking.svg", "The processing steps of the multi-object tracker.") }}

The key features are as follows.

- **Multiple input channels**: The tracker takes up to 12 detection topics. The parameter file `input_channels.param.yaml` gives each channel flags: whether the channel can start new tracks, and whether the tracker can trust its existence probability, size, class, and orientation. For example, the `lidar_clustering` channel can start tracks, but the tracker does not trust its size and orientation.
- **No waiting for synchronization**: The tracker does not use a message filter to synchronize its inputs. The input manager keeps a queue for each channel. It uses the slowest channel that still publishes as the time reference, and gives the tracker the objects of all channels in time order. If a channel stops, the tracker uses the other channels.
- **Periodic output**: With the default parameters, the tracker publishes on a timer at 10 Hz (`publish_on_timer`, `publish_rate`). It predicts the tracks to the publish time (`delay_compensation`).

An earlier version of this design placed a separate priority object merger in front of the tracker. The multi-channel input of the tracker now covers this role. The old [`object_merger`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_object_merger) uses approximate time synchronization of two inputs. As a result, it waits for the slower input, and it does not output a frame if one input is missing:

{{ diagram("image/priority_merger_1.svg", "A synchronized merger waits for the slower detector.") }}

The multi-channel tracker uses each detection result when it arrives. A late or missing channel does not stop the output:

{{ diagram("image/priority_merger_2.svg", "The multi-channel tracker does not wait for all detectors.") }}

The earlier design also proposed a stationary object detection step to reduce the processing time. The tracker of Autoware 2.0.0 does not have this step. It has a static tracker type, which it uses for objects of the `HAZARD` class.

### Prediction

[`autoware_map_based_prediction`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_map_based_prediction) predicts the paths of the tracked objects from the lanes of the vector map. It publishes `/perception/object_recognition/objects`. The prediction launch file also supports [SIMPL](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_simpl_prediction), an ML-based prediction model (`prediction_model_type:=simpl`). Prediction needs the vector map. Without it (`use_vector_map:=false`), the launch file does not start prediction.
