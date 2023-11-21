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

### Internal interface in the perception component

- **Obstacle Segmentation to Object Recognition**
  - Point Cloud: A Point Cloud observed in the current frame, where the ground and outliers are removed.
- **Obstacle Segmentation to Occupancy Grid Map**
  - Ground filtered Point Cloud: A Point Cloud observed in the current frame, where the ground is removed.
- **Occupancy Grid Map to Obstacle Segmentation**
  - Occupancy Grid Map: This is used for filtering outlier.
