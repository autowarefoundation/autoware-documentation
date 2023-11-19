# Perception component Reference Implementation Design

## Purpose of this document

This document outlines detailed design of the reference imprementations. This allows developers and users to understand what is currently available with the Perception Component, how to utilize, expand, or add to its features.

## Architecture

This diagram describes the architecture of the reference implementation.

![overall-perception-architecture](image/reference-implementaion-perception-diagram.drawio.svg)

The Perception component consists of the following sub-components:

- **Obstacle Segmentation**: Identifies point clouds originating from obstacles(not only dynamic objects but also static obstacles that should be avoided, such as stationary obstacles) that the ego vehicle should avoid. For example, construction cones are recognized using this module.
- **Occupancy Grid Map**: Detects blind spots (areas where no information is available and where dynamic objects may jump out).
- **Object Recognition**: Recognizes dynamic objects surrounding the ego vehicle in the current frame and predicts their future trajectories.
  - **Detection**: Detects the pose and velocity of dynamic objects such as vehicles and pedestrians.
    - **Detector**: Triggers object detection processing frame by frame.
    - **Interpolator**: Maintains stable object detection. Even if the output from Detector suddenly becomes unavailable, Interpolator uses the output from the Tracking module to maintain the detection results without missing any objects.
  - **Tracking**: Associates detected results across multiple frames.
  - **Prediction**: Predicts trajectories of dynamic objects.
- **Traffic Light Recognition**: Recognizes the colors of traffic lights and the directions of arrow signals.

The following diagram describes the reference implementation of the Perception component. By adding new modules or extending the functionalities, various ODDs can be supported.

### Remarkable points

#### Obstacle Segmentation

- Ground removal employs a simple method.
  - Assuming dense point clouds with 64 or 128 lines of LiDAR, this approach yields sufficient performance.
  - However, when using sparse LiDAR with 16 lines, False Positives may occur. In such cases, a more sophisticated ground removal method is needed, but this comes with an increased computational load.

#### Object Recognition

- Detects Unknown Objects by LiDAR clustering
  - This minimizes False Negatives to avoid contact with obstacles like tree branches and objects on the road.
  - However, as False Negatives decrease, there is a trade-off with an increase in False Positives due to the misidentification of vegetation, leading to a decrease in system availability.
- In lidar clustering, an upper limit is imposed on the number of points within each cluster.
  - Not setting an upper limit can lead to the formation of enormous clusters, resulting in False Positives.
  - While reducing False Positives, there is a trade-off: the failure to recognize large nearby objects increases, leading to more instances of False Negatives.
- In the DNN-based 3D Detector, an upper limit is imposed on the number of points within a unit region of the input point cloud.
  - Without such an upper limit, there is a potential for increased processing time and memory usage, leading to the destabilization and delays of the entire autonomous driving system.
  - The trade-off involved in setting the upper limit is that, while it reduces computational load, recognition becomes impossible if the number of points within the unit region exceeds the limit. This trade-off can be addressed by incorporating a downsample filter in the preprocessing stage, but it comes at the cost of increased computational load.
- A map-dependent detection validator is being utilized.
  - This allows the removal of instances where buildings are falsely detected as dynamic objects.
  - Since it relies on the map, if the map is incorrect, objects may disappear. There is a trade-off where reducing False Positives may increase the likelihood of False Negatives.
- A Projection-based method is used in the Camera-LiDAR pipeline.
  - Even if the sensor configuration changes, fusion is possible with calibration information, reducing the need for retraining.
  - On the flip side, there is a trade-off where objects detected by the camera may be incorrectly assigned to the wrong clusters. While it adapts well to various sensor configuration differences, it comes with the trade-off of potentially having more False Positives.
- In the Radar Pipeline, only objects at a distant range and with significant speed are detected.
  - Considering the current standard performance of radar systems, not limiting detection to distant and fast-moving objects could result in numerous False Positives that could interfere with vehicle movement. Instead, this approach may lead to False Negatives for distant stationary objects.
  - With advancements in radar performance and a reduction in false detections expected, it is anticipated that the system will become capable of detecting both close-range and stationary objects.
- Interpolator
  - The use of an interpolator allows for the detection of unknown objects in clustering and, if successfully tracked, helps prevent False Negatives.
  - While reducing , there is a trade-off where issues such as inducing vehicle rotation or persistently holding falsely detected objects at a distant range may occur.

#### Traffic Light Recognition

- Rough detection of signal locations is conducted using a map.
  - This reduces the computational load for signal recognition detections.
  - However, there is a trade-off, as there is a potential for false detections when the map is inaccurate or when pitch estimation is misaligned.

### Internal interface in the perception component

- **Obstacle Segmentation to Object Recognition**
  - Point Cloud: A Point Cloud observed in the current frame, where the ground and outliers are removed.
- **Obstacle Segmentation to Occupancy Grid Map**
  - Ground filtered Point Cloud: A Point Cloud observed in the current frame, where the ground is removed.
- **Occupancy Grid Map to Obstacle Segmentation**
  - Occupancy Grid Map: This is used for filtering outlier.
