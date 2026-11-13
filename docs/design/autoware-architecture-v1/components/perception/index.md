# Perception component design

## Purpose of this document

This document outlines the high-level design strategies, goals and related rationales in the development of the Perception Component. Through this document, it is expected that all OSS developers will comprehend the design philosophy, goals and constraints under which the Perception Component is designed, and participate seamlessly in the development.

## Overview

The Perception component receives inputs from the Sensing, Localization, and Map components. It adds semantic information, for example object recognition, obstacle segmentation, traffic light recognition, and an occupancy grid map. Then it sends this information to the Planning component. This component design follows the overarching philosophy of Autoware, defined as the [microautonomy concept](../../../autoware-concepts/index.md#microautonomy-architecture).

## Goals and non-goals

The role of the Perception Component is to recognize the surrounding environment based on the data obtained through Sensing and acquire sufficient information (such as the presence of dynamic objects, stationary obstacles, blind spots, and traffic signal information) to enable autonomous driving.

In our overall design, we emphasize the concept of [microautonomy architecture](../../../autoware-concepts/index.md#microautonomy-architecture). This term refers to a design approach that focuses on the proper modularization of functions, clear definition of interfaces between these modules, and as a result, high expandability of the system. Given this context, the goal of the Perception Component is set not to solve every conceivable complex use case (although we do aim to support basic ones), but rather to provide a platform that can be customized to the user's needs and can facilitate the development of additional features.

To clarify the design concepts, the following points are listed as goals and non-goals.

### Goals

- To provide the basic functions so that a simple ODD can be defined.
- To achieve a design that can provide perception functionality to every autonomous vehicle.
- To be extensible with the third-party components.
- To provide a platform that enables Autoware users to develop the complete functionality and capability.
- To provide a platform that enables Autoware users to develop the autonomous driving system which always outperforms human drivers.
- To provide a platform that enables Autoware users to develop the autonomous driving system achieving "100% accuracy" or "error-free recognition".

### Non-goals

- To develop the perception component architecture specialized for specific / limited ODDs.
- To achieve the complete functionality and capability.
- To outperform the recognition capability of human drivers.
- To achieve "100% accuracy" or "error-free recognition".

## High-level architecture

This diagram shows the high-level architecture of the Perception component.

{{ diagram("image/high-level-perception-diagram.svg", "The Perception component, its inputs, and its output to Planning.") }}

The Perception component has these sub-components:

- **Object recognition**: Recognizes dynamic objects around the ego vehicle in the current frame, and objects that were not present when the map was made. It also predicts their future trajectories. The objects include:
  - Pedestrians
  - Cars
  - Trucks and buses
  - Bicycles
  - Motorcycles
  - Animals
  - Traffic cones
  - Road debris: items such as cardboard, oil drums, trash cans, and wood, either dropped on the road or floating in the air
- **Obstacle segmentation**: Finds the points of the point cloud that come from obstacles. These are dynamic objects and static obstacles. The ego vehicle must steer clear of them or stop in front of them.
  - This includes:
    - All dynamic objects (as listed above)
    - Curbs and bollards
    - Barriers
    - Trees
    - Walls and buildings
  - This does not include:
    - Grass
    - Water splashes
    - Smoke and vapor
    - Newspapers
    - Plastic bags
- **Occupancy grid map**: Finds blind spots. In a blind spot, no information is available and dynamic objects can jump out.
- **Traffic light recognition**: Recognizes the colors of traffic lights and the directions of arrow signals.

The [`ObjectClassification`](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/ObjectClassification.msg) message of `autoware_perception_msgs` defines the object classes that the detectors can output: `UNKNOWN`, `CAR`, `TRUCK`, `BUS`, `TRAILER`, `MOTORCYCLE`, `BICYCLE`, `PEDESTRIAN`, `ANIMAL`, `HAZARD`, `OVER_DRIVABLE`, and `UNDER_DRIVABLE`.

## Component interface

The following describes the input and output concept between the Perception component and other components. See [the Perception component interface](../../interfaces/components/perception.md) page for the topics and message types.

### Input to the Perception component

- **From Sensing**: This input provides real-time information about the environment.
  - Camera image: image data from the cameras.
  - Point cloud: point cloud data from the LiDARs.
  - Radar object: object data from the radars.
- **From Localization**: This input provides real-time information about the ego vehicle.
  - Vehicle motion information: includes the position of the ego vehicle.
- **From Map**: This input provides static information about the environment.
  - Vector map: contains all static information about the environment, including lane area information.
  - Point cloud map: contains static point cloud maps. These maps must not include dynamic objects.
- **From API**:
  - V2X information: the information from V2X modules, for example the states of traffic signals.

### Output from the Perception component

- **To Planning**
  - Dynamic objects: real-time information about objects that cannot be known in advance, such as pedestrians and other vehicles.
  - Obstacle segmentation: real-time information about the location of obstacles. This information is more primitive than detected objects.
  - Occupancy grid map: real-time information about occluded areas.
  - Traffic light recognition result: the current state of each traffic light in real time.

## How to add new modules

As mentioned in the goals, third-party components can extend the Perception component. The object recognition pipeline is the most common place to add a module:

<div class="aw-steps" markdown>

1. **Publish detections.** Make your detector publish [`autoware_perception_msgs/msg/DetectedObjects`](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_perception_msgs/msg/DetectedObjects.msg) for example in the `/perception/object_recognition/detection` namespace.
2. **Add a tracker channel.** The [multi-object tracker](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_multi_object_tracker) takes up to 12 detection inputs (`input/detection01` to `input/detection12`). Connect your topic to a free input in the object recognition launch file, and give it a channel name.
3. **Set the channel flags.** In `input_channels.param.yaml` of [`autoware_perception_config`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_universe_launch/autoware_perception_config/config/object_recognition/tracking/multi_object_tracker), tell the tracker what it can trust from the channel: existence probability, size, class, and orientation, and whether the channel can start new tracks.

</div>

The [reference implementation](reference_implementation.md#launch-files) page describes the launch files of each sub-component.

## Supported functions

| Function                           | Description                                                                                                                                                            | Inputs                                            | Packages                                                                                                                                                                                                                                                                                                                                                                                                                |
| ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| LiDAR DNN-based 3D detector        | Takes point clouds and detects objects such as vehicles, trucks, buses, pedestrians, and bicycles.                                                                     | Point clouds                                      | [`autoware_lidar_centerpoint`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_lidar_centerpoint), [`autoware_lidar_transfusion`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_lidar_transfusion), [`autoware_bevfusion`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_bevfusion)               |
| Camera-LiDAR DNN-based 3D detector | Takes point clouds and camera images, and detects objects in 3D.                                                                                                       | Point clouds, camera images                       | [`autoware_bevfusion`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_bevfusion) (camera-LiDAR model), [`autoware_image_projection_based_fusion`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_image_projection_based_fusion) (PointPainting)                                                                                           |
| Camera DNN-based 2D detector       | Takes camera images and detects objects such as vehicles, trucks, buses, pedestrians, and bicycles in the image. It does not have to give 3D coordinates.              | Camera images                                     | [`autoware_tensorrt_yolox`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_tensorrt_yolox)                                                                                                                                                                                                                                                                                       |
| Camera DNN-based 3D detector       | Takes the images of several cameras and detects objects in 3D without LiDAR.                                                                                           | Camera images                                     | [`autoware_tensorrt_bevdet`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_tensorrt_bevdet), [`autoware_camera_streampetr`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_camera_streampetr), [`autoware_tensorrt_bevformer`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_tensorrt_bevformer) |
| LiDAR clustering                   | Groups the points of a point cloud into clusters and gives each cluster a shape. It detects objects without labels.                                                    | Point clouds                                      | [`autoware_euclidean_cluster`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_euclidean_cluster), [`autoware_detected_object_feature_remover`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_detected_object_feature_remover)                                                                                                            |
| Semi-rule-based detector           | Detects objects with images and point clouds. It combines LiDAR clustering and the camera DNN-based 2D detector.                                                       | Output of the 2D detector and of LiDAR clustering | [`autoware_image_projection_based_fusion`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_image_projection_based_fusion)                                                                                                                                                                                                                                                         |
| LiDAR semantic segmentation        | Gives each point a class label. A label-based clustering step makes objects from the labeled points.                                                                   | Point clouds                                      | [`autoware_ptv3`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_ptv3), [`autoware_lidar_frnet`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_lidar_frnet)                                                                                                                                                                              |
| Radar-based 3D detector            | Takes radar data and detects dynamic 3D objects. See the [radar-based 3D detector](reference-implementations/radar-based-3d-detector/radar-based-3d-detector.md) page. | Radar data                                        | [`autoware_object_sorter`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_object_sorter), [`autoware_radar_fusion_to_detected_object`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_radar_fusion_to_detected_object)                                                                                                                    |
| Object merger                      | Combines the results of several detectors.                                                                                                                             | Detected objects                                  | [`autoware_object_merger`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_object_merger), [`autoware_simple_object_merger`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_simple_object_merger)                                                                                                                                          |
| Interpolator                       | Keeps detection results stable over time. It uses the tracking results to keep objects that a detector misses for a short time.                                        | Detected objects, tracked objects                 | [`autoware_detection_by_tracker`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_detection_by_tracker)                                                                                                                                                                                                                                                                           |
| Tracking                           | Gives an ID to each detected object and estimates its velocity.                                                                                                        | Detected objects                                  | [`autoware_multi_object_tracker`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_multi_object_tracker)                                                                                                                                                                                                                                                                           |
| Prediction                         | Predicts the future paths of dynamic objects, and their probabilities, from the shape of the map and the surrounding environment.                                      | Tracked objects, vector map                       | [`autoware_map_based_prediction`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_map_based_prediction), [`autoware_simpl_prediction`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_simpl_prediction)                                                                                                                                    |
| Obstacle segmentation              | Finds the points that come from obstacles that the ego vehicle must avoid.                                                                                             | Point clouds, point cloud map                     | [`autoware_ground_segmentation`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_ground_segmentation), [`autoware_compare_map_segmentation`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_compare_map_segmentation)                                                                                                                      |
| Occupancy grid map                 | Finds blind spots. In a blind spot, no information is available and dynamic objects can jump out.                                                                      | Point clouds                                      | [`autoware_probabilistic_occupancy_grid_map`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_probabilistic_occupancy_grid_map)                                                                                                                                                                                                                                                   |
| Traffic light recognition          | Finds the position and the state of traffic signals.                                                                                                                   | Camera images, vector map                         | [`autoware_traffic_light_fine_detector`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_fine_detector), [`autoware_traffic_light_classifier`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_classifier), and other `autoware_traffic_light_*` packages                                                       |

## Reference implementation

When you launch Autoware, it loads the default parameters and starts the reference implementation.

<div class="grid cards aw-cards" markdown>

- :fa-cl-s fa-diagram-project: **[Reference implementation](reference_implementation.md)**

  The launch files, the data flow, and the detection, tracking, and prediction pipelines.

- :fa-cl-s fa-satellite-dish: **[Radar-based 3D detector](reference-implementations/radar-based-3d-detector/radar-based-3d-detector.md)**

  How radar data adds faraway objects and velocities to the object recognition.

</div>
