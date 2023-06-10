# Perception component design

!!! warning

    Under Construction

## Purpose of this document

TODO: 背景と目的の認識合意、誰にどのように使われるかを明確に記載する。

この文書は、Perception Componentの開発における目標やハイレベルな設計戦略、およびそれに関連する意思決定とその理由を説明します。このドキュメントを通じて、すべてのOSS開発者は、Perception Componentがどのような設計思想や制約のもとで設計され、どのような目標を達成するために開発が行われているのかを理解することができます。これにより、円滑な開発参加が可能となります。

さらに、（これらの情報は将来的に分離して管理されるかもしれませんが、）具体的なリファレンス実装や提供される機能の一覧も後半に記載されています。これにより、開発者やユーザーは、Perception Componentを使用することで現在何が可能なのか、機能をどのように活用したり、拡張したり、追加したりすることができるのかを理解することができます。

## Overview

The Perception Component receives inputs from Sensing, Localization, and Map components, and adds semantic information (e.g., Object Recognition, Obstacle Segmentation, Traffic Light Recognition, Occupancy Grid Map), which is then passed on to Planning Component. This component design follows the overarching philosophy of Autoware, defined as the [microautonomy concept](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-concepts/).

## Requirements

The goal of the Perception Component is to provide functionality and APIs that enable the development of various Autonomous Driving Vehicles, such as trucks driving on freeways, robotaxis navigating urban areas, Autonomous Mobile Robots traversing inside buildings, self-driving vehicles conducting transport within factories, or rovers exploring the lunar surface. This aims to offer flexibility in creating such vehicles.

In order to achieve this, it is necessary to design the Perception component with attention to Policy/Mechanism separation.
In other words, it means the following:

- Should not only consider how to achieve the best possible Perception functionality for autonomous driving. Should consider how to design a system that can enable a platform capable of achieving even the best possible Perception functionality for autonomous driving.
- In order to be able to develop any type of autonomous vehicle, we must design the Perception Component in a way that allows its implementation. For example, while it is possible to add features for a rover to traverse the lunar surface, it is not acceptable to design the Perception Component to work exclusively for the lunar rover, limiting its functionality to that specific purpose.

<!-- to accurately perform Object Recognition, Traffic Light Recognition, and Blind-spot Detection from sensor information. -->

**Goals:**

- To achieve a design that can provide Perception functionality to every autonomous vehicle.
- The capability is extensible with the third-party components.
- The Perception component is designed to provide a platform that enables autoware users to develop the complete functionality and capability.
- The Perception component is designed to provide a platform that enables autoware users to develop the autonomous driving system which always outperforms human drivers.
- The Perception component is designed to provide a platform  that enables autoware users to develop the autonomous driving system achiving "zero overlooks" or "error-free recognition".
- The basic functions are provided so that a simple ODD can be defined.

**Non-goals:**

- The Perception Component should not operate perfectly only in specific environments and be completely useless in other environments.
- The Perception component is not self-contained but can be extended with third parties.
- The Perception component is not aimed at the complete functionality and capability.
- The Perception component is not designed to always outperform human drivers.
- The Perception component is not capable of achieving "zero overlooks" or "error-free recognition".

## High-level architecture

This diagram describes the high-level architecture of the Perception Component.

![overall-perception-architecture](image/high-level-perception-diagram.drawio.svg)

The Perception component consists of the following sub-components:

- **Object Recognition**: Recognizes dynamic objects surrounding the ego vehicle in the current frame and predicts their future trajectories.
  - **Detection**: Detects the pose and velocity of dynamic objects such as vehicles and pedestrians.
    - **Detector**: Triggers object detection processing frame by frame.
    - **Interpolator**: Maintains stable object detection. Even if the output from Detector suddenly becomes unavailable, Interpolator uses the output from the Tracking module to maintain the detection results without missing any objects.
  - **Tracking**: Associates detected results across multiple frames.
  - **Prediction**: Predicts trajectories of dynamic objects.
- **Obstacle Segmentation**: Detects not only dynamic objects but also static obstacles that should be avoided, such as stationary obstacles. For example, construction cones are recognized using this module.
- **Occupancy Grid Map**: Detects blind spots (areas where no information is available and where dynamic objects may jump out).
- **Traffic Light Recognition**: Recognizes the colors of traffic lights and the directions of arrow signals.

## Component interface

The following describes the input/output concept between Perception Component and other components. See the [Perception Component Interface (WIP)](../../autoware-interfaces/components/perception.md) page for the current implementation.

### Input to the perception component

- **From Sensing**
  - Camera: Image data obtained from the camera. The Perception component utilizes it for Traffic Light Recognition and Object Recognition.
  - Point Cloud: Point Cloud data obtained from LiDAR. The Perception component utilizes it for Object Recognition and blind spot detection.
  - RadarTrack: RadarTrack data obtained from radar. The Perception component utilizes it for Object Recognition.
- **From Localization**
  - Vehicle motion information: Includes the ego vehicle's position.
- **From Map**
  - Vector Map: Contains all static information about the environment, including lane aria information for filtering unkown objects outside lane and the locations of traffic lights.
  - Point Cloud Map: The Perception Component uses Point Cloud Map for compare map filter.

### Output from the perception component

- **To Planning**
  - Dynamic Objects: Provides real-time information about objects that cannot be known in advance, such as pedestrians and other vehicles.
  - Obstacle Segmentation: Supplies real-time information about the location of obstacles, which is more primitive than Detected Object.
  - Occupancy Grid Map: Offers real-time information about the presence of occluded area information.
  - Traffic Light Recognition result: Provides the current state of each traffic light in real time.

### Internal interface in the perception component

- **Obstacle Segmentation to Object Recognition**
  - Point Cloud: A Point Cloud observed in the current frame, where the ground and outliers are removed.
- **Obstacle Segmentation to Occupancy Grid Map**
  - Ground filtered Point Cloud: A Point Cloud observed in the current frame, where the ground is removed.
- **Occupancy Grid Map to Obstacle Segmentation**
  - Occupancy Grid Map: This is used for filtering outlier.

## How to add new modules (WIP)

As mentioned in the goal session, this perception module is designed to be extensible by third-party components. For specific instructions on how to add new modules and expand its functionality, please refer to the provided documentation or guidelines (WIP).

## Supported Functions

## Reference Implementation

The following diagram describes the reference implementation of the Perception component. By adding new modules or extending the functionalities, various ODDs can be supported.

_Note that some implementation does not adhere to the high-level architecture design and require updating._

![reference-implementation](image/perception-diagram.png)

For more details, please refer to the design documents in each package.

### Important Parameters

| Package           | Parameter               | Type   | Description                                                                                                                                                                                                                                            |
| ----------------- | ----------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| perception launch | `mode`                  | string | detectionのmode. options: `camera_lidar_radar_fusion`, `camera_lidar_fusion`, `lidar_radar_fusion`, `lidar` or `radar` <https://github.com/autowarefoundation/autoware.universe/blob/main/launch/tier4_perception_launch/launch/perception.launch.xml> |
| perception launch | `lidar_detection_model` | string | lidar detection modelのmodel. options: `centerpoint`, `apollo`, `pointpainting`, `clustering`                                                                                                                                                          |
| perception launch | `use_object_filter`     | string | Whether to use filters for detected objects. They can filter objects bu using either vector maps or relative positions.                                                                                                                                |

### Notation
