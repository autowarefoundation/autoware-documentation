# Autoware's Design

## Autoware concepts

The [Autoware concepts page](autoware-concepts/index.md) describes the design philosophy of Autoware. Readers (service providers and all Autoware users) will learn the basic concepts underlying Autoware development, such as microautonomy and the Core/Universe architecture.

## Architecture

The [Autoware architecture page](autoware-architecture-v1/index.md) describes an overview of each module that makes up Autoware. Readers (all Autoware users) will gain a high-level picture of how each module that composes Autoware works.

Core and Universe.

Autoware provides the runtimes and technology components by open-source software. The runtimes are based on the Robot Operating System (ROS). The technology components are provided by contributors, which include, but are not limited to:

- Sensing
  - Camera Component
  - LiDAR Component
  - RADAR Component
  - GNSS Component
- Computing
  - Localization Component
  - Perception Component
  - Planning Component
  - Control Component
  - Logging Component
  - System Monitoring Component
- Actuation
  - DBW Component
- Tools
  - Simulator Component
  - Mapping Component
  - Remote Component
  - ML Component
  - Annotation Component
  - Calibration Component

### Autoware interfaces

The [Autoware interfaces page](autoware-architecture-v1/interfaces/index.md) describes in detail the interface of each module that makes up Autoware. Readers (intermediate developers) will learn how to add new functionality to Autoware and how to integrate their own modules with Autoware.

### Concern, Assumption, and Limitation

The downside of the microautonomy architecture is that the computational performance of end applications is sacrificed due to its data path overhead attributed to functional modularity. In other words, the trade-off characteristic of the microautonomy architecture exists between computational performance and functional modularity. This trade-off problem can be solved technically by introducing real-time capability. This is because autonomous driving systems are not really designed to be real-fast, that is, low-latency computing is nice-to-have but not must-have. The must-have feature for autonomous driving systems is that the latency of computing is predictable, that is, the systems are real-time. As a whole, we can compromise computational performance to an extent that is predictable enough to meet the given timing constraints of autonomous driving systems, often referred to as deadlines of computation.
