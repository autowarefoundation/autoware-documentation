# Node diagram

This page shows the main nodes of Autoware and the topics between the components. It shows the default setup of [`autoware_launch`](https://github.com/autowarefoundation/autoware_launch), with the rule-based planning stack. For the inputs and outputs of each component, see [Component interfaces](../interfaces/components/index.md).

{{ diagram("node-diagram.svg", "The main nodes of each component and the main topics between the components.") }}

The diagram is an overview. A running Autoware has many more nodes, and many of them are optional or depend on launch arguments. For example, the diagram does not show the parking scenario, the planning modules inside the behavior and motion planners, the control checkers, or the debug and diagnostic topics.

## See the full graph

Use one of these tools to see every node and topic of your setup.

<div class="aw-features" markdown>

- :fa-cl-s fa-diagram-project: **rqt_graph**

  Launch Autoware, for example with the [planning simulation](../../../simulation/planning-simulation/index.md). Then run `rqt_graph` in another terminal. It shows the nodes and topics that run now.

- :fa-cl-s fa-sitemap: **Autoware System Designer**

  [`autoware_system_designer`](https://github.com/autowarefoundation/autoware_system_designer) generates a node diagram from the YAML design of a system. Build the [`autoware_sample_designs`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_sample_designs) package, and then open `install/systems.html` in a web browser.

</div>

To build the sample system design in a workspace where Autoware is already built, run:

```bash
colcon build --symlink-install --packages-select autoware_sample_designs --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The `systems.html` page shows every node, module, and topic connection of the `AutowareSample` system, for each of its modes.
