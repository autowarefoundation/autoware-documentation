# Autoware's Design

## Architecture

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

## Concern, Assumption, and Limitation

The downside of the microautonomy architecture is that the computational performance of end applications is sacrificed due to its data path overhead attributed to functional modularity. In other words, the trade-off characteristic of the microautonomy architecture exists between computational performance and functional modularity. This trade-off problem can be solved technically by introducing real-time capability. This is because autonomous driving systems are not really designed to be real-fast, that is, low-latency computing is nice-to-have but not must-have. The must-have feature for autonomous driving systems is that the latency of computing is predictable, that is, the systems are real-time. As a whole, we can compromise computational performance to an extent that is predictable enough to meet the given timing constraints of autonomous driving systems, often referred to as deadlines of computation.

## Design

!!! warning

    Under Construction

### Autoware concepts

### Autoware architecture

The overall architecture of Autoware and the detailed design of each component are available [here](autoware-architecture/index.md).

### Autoware interfaces

### Configuration management

## Conclusion
