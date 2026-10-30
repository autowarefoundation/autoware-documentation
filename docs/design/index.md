# Architecture design

This section describes how Autoware is designed: its concepts, its architecture, its components and interfaces, and how its repositories are organized and released.
Autoware 2.0 uses Architecture 2.0, which adds AI planning next to the rule-based stack.

## Start here

<div class="grid cards aw-cards" markdown>

- :fa-cl-s fa-lightbulb: **[Concepts](autoware-concepts/index.md)**

  The design philosophy of Autoware: the microautonomy architecture, and the Core and Universe repository model.

- :fa-cl-s fa-diagram-project: **[Architecture 2.0](autoware-architecture-v2/index.md)**

  The current architecture: generators make candidate trajectories, and a selector checks them and picks one.

- :fa-cl-s fa-cubes: **[Components](autoware-architecture-v1/components/sensing/index.md)**

  The stacks of Autoware: sensing, map, localization, perception, planning, control, and the vehicle interface.

- :fa-cl-s fa-plug: **[Interfaces](autoware-architecture-v1/interfaces/index.md)**

  The interfaces between the components, and the AD API for systems outside Autoware.

- :fa-cl-s fa-circle-nodes: **[Node diagram](autoware-architecture-v1/node-diagram/index.md)**

  The nodes of Autoware in the default configuration, and how they connect.

- :fa-cl-s fa-sitemap: **[Repositories and releases](repository-structure.md)**

  The repositories of Autoware, the `.repos` files, and the versioning and release process.

</div>

## Concern, assumption, and limitation

The downside of the microautonomy architecture is that the computational performance of end applications is sacrificed due to its data path overhead attributed to functional modularity. In other words, the trade-off characteristic of the microautonomy architecture exists between computational performance and functional modularity. This trade-off problem can be solved technically by introducing real-time capability. This is because autonomous driving systems are not really designed to be real-fast, that is, low-latency computing is nice-to-have but not must-have. The must-have feature for autonomous driving systems is that the latency of computing is predictable, that is, the systems are real-time. As a whole, we can compromise computational performance to an extent that is predictable enough to meet the given timing constraints of autonomous driving systems, often referred to as deadlines of computation.
