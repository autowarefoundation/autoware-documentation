# Architecture 1.0

Architecture 1.0 is the layered architecture of the rule-based Autoware stack.
In [Architecture 2.0](../autoware-architecture-v2/index.md), this stack is still the default stack, and its planning acts as the rule-based generator of trajectories.

## Introduction

Architecture 1.0 is a layered architecture that clarifies the role of each module and simplifies the interfaces between them. By doing so:

- Autoware's internal processing becomes more transparent.
- Collaborative development is made easier because of the reduced interdependency between modules.
- Users can easily replace an existing module (for example, localization) with their own software component by wrapping their software to fit in with Autoware's interface.

The initial focus of this architecture design was solely on driving capability. The following features were left as future work at first. Later work added packages for each of them:

| Feature                 | Examples of the packages in Autoware 2.0.0                                        |
| :---------------------- | :-------------------------------------------------------------------------------- |
| Fail safe               | `autoware_mrm_handler` and the minimal risk maneuver (MRM) operators              |
| Human machine interface | The [AD API](interfaces/ad-api/index.md)                                          |
| Real-time processing    | `agnocast`, a zero-copy inter-process communication middleware                    |
| Redundant system        | `autoware_redundancy_switcher_interface` and `autoware_redundancy_adapi_switcher` |
| State monitoring system | `autoware_diagnostic_graph_aggregator` and `autoware_system_monitor`              |

## High-level architecture design

{{ diagram("image/autoware-architecture-overview.svg", "The stacks of the rule-based architecture and the data flow between them.") }}

Autoware's architecture consists of the following seven stacks. Each linked page contains a more detailed set of requirements and use cases specific to that stack:

<div class="grid cards aw-cards aw-cards--small" markdown>

- :fa-cl-s fa-satellite-dish: **[Sensing](components/sensing/index.md)**

  Sensor drivers and the preprocessing of sensor data.

- :fa-cl-s fa-map: **[Map](components/map/index.md)**

  The point cloud map and the vector map.

- :fa-cl-s fa-location-crosshairs: **[Localization](components/localization/index.md)**

  The pose and the velocity of the vehicle.

- :fa-cl-s fa-eye: **[Perception](components/perception/index.md)**

  Objects, obstacles, and traffic lights around the vehicle.

- :fa-cl-s fa-route: **[Planning](components/planning/index.md)**

  The route and the trajectory of the vehicle.

- :fa-cl-s fa-gauge-high: **[Control](components/control/index.md)**

  The commands that make the vehicle follow the trajectory.

- :fa-cl-s fa-car: **[Vehicle interface](components/vehicle/index.md)**

  The connection between Autoware and the vehicle.

</div>

## Node diagram

The [node diagram](node-diagram/index.md) page shows Autoware's nodes in the default configuration. Detailed documents for each node are available in the [Autoware Universe docs](https://autowarefoundation.github.io/autoware_universe/main/).

Autoware configurations are scalable and selectable, and they vary depending on the environment and the required use cases.

## References

- [The architecture presentation given to the AWF Technical Steering Committee, March 2020](https://discourse.ros.org/uploads/short-url/woUU7TGLPXFCTJLtht11rJ0SqCL.pdf)
