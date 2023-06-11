# Perception component design

!!! warning

    Under Construction

## Purpose of this document

TODO: 背景と目的の認識合意、誰にどのように使われるかを明確に記載する。

この文書は、Perception Componentの開発における目標やハイレベルな設計戦略、およびそれに関連する意思決定とその理由を説明します。このドキュメントを通じて、すべてのOSS開発者は、Perception Componentがどのような設計思想や制約のもとで設計され、どのような目標を達成するために開発が行われているのかを理解することができます。これにより、円滑な開発参加が可能となります。

さらに、（これらの情報は将来的に分離して管理されるかもしれませんが、）具体的なリファレンス実装や提供される機能の一覧も後半に記載されています。これにより、開発者やユーザーは、Perception Componentを使用することで現在何が可能なのか、機能をどのように活用したり、拡張したり、追加したりすることができるのかを理解することができます。

## Overview

The Perception Component receives inputs from Sensing, Localization, and Map components, and adds semantic information (e.g., Object Recognition, Obstacle Segmentation, Traffic Light Recognition, Occupancy Grid Map), which is then passed on to Planning Component. This component design follows the overarching philosophy of Autoware, defined as the [microautonomy concept](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-concepts/).

<!-- ## Requirements -->

## Goals and non-goals

<!-- 最強のPerception Systemをdesignしたいわけではない。最強のPerception Systemでさえ実現可能な、platformをdesignしたい。 -->
<!-- ある一つの尖ったODDに特化しているようなdesignは望ましくない。一方、ある一つの尖ったODDであっても実現可能であるようにはdesignしたい。 -->

The goal of the Perception Component is to provide functionality and APIs that enable the development of various Autonomous Driving Vehicles, such as trucks driving on freeways, robotaxis navigating urban areas, Autonomous Mobile Robots traversing inside buildings, self-driving vehicles conducting transport within factories, or rovers exploring the lunar surface. This aims to offer flexibility in creating such vehicles.

In other words, it means the following:

- Should not only consider how to achieve the best possible Perception functionality for autonomous driving. Should consider how to design a system that can enable a platform capable of achieving even the best possible Perception functionality for autonomous driving.
- In order to be able to develop any type of autonomous vehicle, we must design the Perception Component in a way that allows its implementation. For example, while it is possible to add features for a rover to traverse the lunar surface, it is not acceptable to design the Perception Component to work exclusively for the lunar rover, limiting its functionality to that specific purpose.

**Goals:**

- To achieve a design that can provide Perception functionality to every autonomous vehicle.
- The capability is extensible with the third-party components.
- The Perception component is designed to provide a platform that enables autoware users to develop the complete functionality and capability.
- The Perception component is designed to provide a platform that enables autoware users to develop the autonomous driving system which always outperforms human drivers.
- The Perception component is designed to provide a platform that enables autoware users to develop the autonomous driving system achiving "zero overlooks" or "error-free recognition".
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
- **Obstacle Segmentation**: Detects not only dynamic objects but also static obstacles that should be avoided, such as stationary obstacles. For example, construction cones are recognized using this module.
- **Occupancy Grid Map**: Detects blind spots (areas where no information is available and where dynamic objects may jump out).
- **Traffic Light Recognition**: Recognizes the colors of traffic lights and the directions of arrow signals.

## Component interface

The following describes the input/output concept between Perception Component and other components. See the [Perception Component Interface (WIP)](../../autoware-interfaces/components/perception.md) page for the current implementation.

### Input to the perception component

- **From Sensing**
  - Camera: Image data obtained from the camera. The Perception component can utilize it for Traffic Light Recognition and Object Recognition.
  - Point Cloud: Point Cloud data obtained from LiDAR. The Perception component can utilize it for Object Recognition and blind spot detection.
  - RadarTrack: RadarTrack data obtained from radar. The Perception component can utilize it for Object Recognition.
- **From Localization**
  - Vehicle motion information: Includes the ego vehicle's position.
- **From Map**
  - Vector Map: Contains all static information about the environment, including lane aria information.
  <!-- for filtering unkown objects outside lane and the locations of traffic lights. -->
- Point Cloud Map: The Perception Component can utilize it for compare map filter.

### Output from the perception component

- **To Planning**
  - Dynamic Objects: Provides real-time information about objects that cannot be known in advance, such as pedestrians and other vehicles.
  - Obstacle Segmentation: Supplies real-time information about the location of obstacles, which is more primitive than Detected Object.
  - Occupancy Grid Map: Offers real-time information about the presence of occluded area information.
  - Traffic Light Recognition result: Provides the current state of each traffic light in real time.

## How to add new modules (WIP)

As mentioned in the goal session, this perception module is designed to be extensible by third-party components. For specific instructions on how to add new modules and expand its functionality, please refer to the provided documentation or guidelines (WIP).

## Supported Functions

<!-- linuxが様々な機能を柔軟に追加できるように、autowareも、様々な機能を柔軟に追加できるようにしている。 -->
<!-- 逆に言えば、これらの機能は、linuxのように、柔軟に様々なPolicyを達成できるように設計されているとより望ましい。 -->

- Perception ComponentのFuncionsは、様々なODDにおける自動運転システム開発を実現できるようにdesignされています。
- TODO: 残りも記載する

| Feature                      | Description                                                                                                                                                                                   | Requirements                                                    | Packages                                                                                                                                                                                                                           |
| ---------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| LiDAR DNN based 3D detector  | This module takes point clouds as input and performs detection of objects such as vehicles, trucks, buses, pedestrians, and bicycles.                                                         | - Point Cloud                                                   | - [lidar_centerpoint](https://github.com/autowarefoundation/autoware.universe/blob/main/perception/lidar_centerpoint/README.md)                                                                                                    |
| Camera DNN based 2D detector | This module takes camera image as input and performs detection of objects such as vehicles, trucks, buses, pedestrians, and bicycles.                                                         | - Camera Image                                                  | - [tensorrt_yolox](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/tensorrt_yolox) <br> - [tensorrt_yolo](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/tensorrt_yolo) |
| LiDAR Clustering             | LiDAR clustering module performs clustering of point clouds and shape estimation to achieve object detection without labels.                                                                  | - Point Cloud                                                   | - [euclidean_cluster](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/euclidean_cluster)                                                                                                              |
| Semi-rule based detector     | Semi-rule based detector performs object detection using information from both images and point clouds, and it consists of two components: LiDAR Clustering and Camera DNN based 2D detector. | - Output from Camera DNN based 2D detector and LiDAR Clustering | - [image_projection_based_fusion](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/image_projection_based_fusion)                                                                                      |
| Object Merger                | Object Merger integrates results from various detectors.                                                                                                                                      | - Detected Object                                               | - [object_merger](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/object_merger)                                                                                                                      |
| Interpolator                 | Interpolator stabilizes the object detection results by maintaining long-term detection results using Tracking results.                                                                       | - Detected Object <br> - Tracked Object                         | - [detection_by_tracker](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/detection_by_tracker)                                                                                                        |

## Reference Implementation

autowareを起動すると、default parameterが読み込まれ、Reference Implementationが起動される。詳細については、[Reference Implementation](./reference_implementation.md)を参照。

See [Reference Implementation](./reference_implementation.md)
