# Perception Component Design

## Purpose of this document

This document outlines the high-level design strategies, goals and related rationales in the development of the Perception Component. Through this document, it is expected that all OSS developers will comprehend the design philosophy, goals and constraints under which the Perception Component is designed, and participate seamlessly in the development.

## Overview

The Perception Component receives inputs from Sensing, Localization, and Map components, and adds semantic information (e.g., Object Recognition, Obstacle Segmentation, Traffic Light Recognition, Occupancy Grid Map), which is then passed on to Planning Component. This component design follows the overarching philosophy of Autoware, defined as the [microautonomy concept](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-concepts/).

## Goals and non-goals

The role of the Perception Component is to recognize the surrounding environment based on the data obtained through Sensing and acquire sufficient information (such as the presence of dynamic objects, stationary obstacles, blind spots, and traffic signal information) to enable autonomous driving.

In our overall design, we emphasize the concept of [microautonomy architecture](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-concepts). This term refers to a design approach that focuses on the proper modularization of functions, clear definition of interfaces between these modules, and as a result, high expandability of the system. Given this context, the goal of the Perception Component is set not to solve every conceivable complex use case (although we do aim to support basic ones), but rather to provide a platform that can be customized to the user's needs and can facilitate the development of additional features.

To clarify the design concepts, the following points are listed as goals and non-goals.

**Goals:**

- To provide the basic functions so that a simple ODD can be defined.
- To achieve a design that can provide perception functionality to every autonomous vehicle.
- To be extensible with the third-party components.
- To provide a platform that enables Autoware users to develop the complete functionality and capability.
- To provide a platform that enables Autoware users to develop the autonomous driving system which always outperforms human drivers.
- To provide a platform that enables Autoware users to develop the autonomous driving system achieving "100% accuracy" or "error-free recognition".

**Non-goals:**

- To develop the perception component architecture specialized for specific / limited ODDs.
- To achieve the complete functionality and capability.
- To outperform the recognition capability of human drivers.
- To achieve "100% accuracy" or "error-free recognition".

## High-level architecture

This diagram describes the high-level architecture of the Perception Component.

![overall-perception-architecture](image/high-level-perception-diagram.drawio.svg)

The Perception Component consists of the following sub-components:

- **Object Recognition**: Recognizes dynamic objects surrounding the ego vehicle in the current frame, objects that were not present during map creation, and predicts their future trajectories. This includes:
  - Pedestrians
  - Cars
  - Trucks/Buses
  - Bicycles
  - Motorcycles
  - Animals
  - Traffic Cones
  - Fallen Objects, Flying Objects, etc: Items such as cardboard, drums, wood, etc., either falling or airborne.
- **Obstacle Segmentation**: Identifies point clouds originating from obstacles, including both dynamic objects and static obstacles that requires the ego vehicle either steer clear of them or come to a stop in front of the obstacles.
  - This includes:
    - all dynamic objects (as listed above)
    - curbs/bollards
    - barriers
    - trees
    - walls/buildings
    - etc
  - This does not include:
    - grass
    - water splashes
    - smoke/vapor
    - newspapers
    - plastic bags
    - etc
- **Occupancy Grid Map**: Detects blind spots (areas where no information is available and where dynamic objects may jump out).
- **Traffic Light Recognition**: Recognizes the colors of traffic lights and the directions of arrow signals.

## Component interface

The following describes the input/output concept between Perception Component and other components. See [the Perception Component Interface](../../autoware-interfaces/components/perception.md) page for the current implementation.

### Input to the Perception Component

- **From Sensing**: This input should provide real-time information about the environment.
  - Camera Image: Image data obtained from the camera.
  - Point Cloud: Point Cloud data obtained from LiDAR.
  - Radar Object: Object data obtained from radar.
- **From Localization**: This input should provide real-time information about the ego vehicle.
  - Vehicle motion information: Includes the ego vehicle's position.
- **From Map**: This input should provide real-time information about the static information about the environment.
  - Vector Map: Contains all static information about the environment, including lane aria information.
  - Point Cloud Map: Contains static point cloud maps, which should not include information about the dynamic objects.
- **From API**:
  - V2X information: The information from V2X modules. For example, the information from traffic signals.

### Output from the Perception Component

- **To Planning**
  - Dynamic Objects: Provides real-time information about objects that cannot be known in advance, such as pedestrians and other vehicles.
  - Obstacle Segmentation: Supplies real-time information about the location of obstacles, which is more primitive than Detected Object.
  - Occupancy Grid Map: Offers real-time information about the presence of occluded area information.
  - Traffic Light Recognition result: Provides the current state of each traffic light in real time.

## How to add new modules (WIP)

As mentioned in the goal session, this perception module is designed to be extensible by third-party components. For specific instructions on how to add new modules and expand its functionality, please refer to the provided documentation or guidelines (WIP).

## Supported Functions

| Feature                      | Description                                                                                                                                                                                                                                                       | Requirements                                                    |
| ---------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------- |
| LiDAR DNN based 3D detector  | This module takes point clouds as input and detects objects such as vehicles, trucks, buses, pedestrians, and bicycles.                                                                                                                                           | - Point Clouds                                                  |
| Camera DNN based 2D detector | This module takes camera images as input and detects objects such as vehicles, trucks, buses, pedestrians, and bicycles in the two-dimensional image space. It detects objects within image coordinates and providing 3D coordinate information is not mandatory. | - Camera Images                                                 |
| LiDAR Clustering             | This module performs clustering of point clouds and shape estimation to achieve object detection without labels.                                                                                                                                                  | - Point Clouds                                                  |
| Semi-rule based detector     | This module detects objects using information from both images and point clouds, and it consists of two components: LiDAR Clustering and Camera DNN based 2D detector.                                                                                            | - Output from Camera DNN based 2D detector and LiDAR Clustering |
| Object Merger                | This module integrates results from various detectors.                                                                                                                                                                                                            | - Detected Objects                                              |
| Interpolator                 | This module stabilizes the object detection results by maintaining long-term detection results using Tracking results.                                                                                                                                            | - Detected Objects <br> - Tracked Objects                       |
| Tracking                     | This module gives ID and estimate velocity to the detection results.                                                                                                                                                                                              | - Detected Objects                                              |
| Prediction                   | This module predicts the future paths (and their probabilities) of dynamic objects according to the shape of the map and the surrounding environment.                                                                                                             | - Tracked Objects <br> - Vector Map                             |
| Obstacle Segmentation        | This module identifies point clouds originating from obstacles that the ego vehicle should avoid.                                                                                                                                                                 | - Point Clouds <br> - Point Cloud Map                           |
| Occupancy Grid Map           | This module detects blind spots (areas where no information is available and where dynamic objects may jump out).                                                                                                                                                 | - Point Clouds <br> - Point Cloud Map                           |
| Traffic Light Recognition    | This module detects the position and state of traffic signals.                                                                                                                                                                                                    | - Camera Images <br> - Vector Map                               |

## Reference Implementation

When Autoware is launched, the default parameters are loaded, and the Reference Implementation is started. For more details, please refer to [the Reference Implementation](./reference_implementation.md).
