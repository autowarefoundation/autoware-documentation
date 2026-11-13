# System launch files

## Overview

The Autoware system stack starts with `autoware.launch.xml`, as the [Launch Autoware](../index.md) page explains. If `launch_system` is `true`, `autoware.launch.xml` includes `tier4_system_component.launch.xml` of the `autoware_launch` package. This file includes `system.launch.xml` of the [`tier4_system_launch`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/tier4_universe_launch/tier4_system_launch) package. The following diagram shows the system launch flow in the `autoware_launch` repository.

{{ diagram("images/system-launch-flow.svg", "tier4_system_component.launch.xml includes system.launch.xml, which launches the system packages that its launch arguments enable.") }}

!!! note

    The Autoware project is a large project.
    Therefore, as we manage the Autoware project, we utilize specific
    arguments in the launch files.
    ROS 2 offers an argument-overriding feature for these launch files.
    Please refer to [the official ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameter-overrides) for further information.
    For instance,
    if we define an argument at the top-level launch,
    it will override the value on lower-level launches.

As the flow diagram shows, `system.launch.xml` of `tier4_system_launch` always launches these packages:

- [duplicated_node_checker](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_duplicated_node_checker/)
- [processing_time_checker](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_processing_time_checker/)
- [pipeline_latency_monitor](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_pipeline_latency_monitor/)
- [component_state_monitor](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_component_state_monitor/)
- [mrm_emergency_stop_operator](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_mrm_emergency_stop_operator/)
- [diagnostic_graph_aggregator](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_diagnostic_graph_aggregator/)
- [hazard_status_converter](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/system/autoware_hazard_status_converter)

It launches these packages when their launch arguments allow it:

- [system_monitor](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_system_monitor/), if `launch_system_monitor` is `true` (the default).
- [mrm_comfortable_stop_operator](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_mrm_comfortable_stop_operator/) and [mrm_handler](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_mrm_handler/), if `use_control_command_gate` is `false` (the default).
- [command_mode_switcher](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_command_mode_switcher/) and [command_mode_decider](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_command_mode_decider/), if `use_control_command_gate` is `true`.
- [dummy_diag_publisher](https://autowarefoundation.github.io/autoware_universe/main/simulator/autoware_dummy_diag_publisher/), if `launch_dummy_diag_publisher` is `true`. The default is `false`.

`tier4_system_component.launch.xml` also includes `logging.launch.xml` of [diagnostic_graph_utils](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_diagnostic_graph_utils/). It logs the errors of the diagnostic graph in the terminal.

`autoware.launch.xml` has these system arguments:

| Argument                                 | Default value                                                                    | Description                                                                                        |
| ---------------------------------------- | -------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------- |
| `system_run_mode`                        | `online`                                                                         | The run mode of the system. The options are `online`, `logging_simulation`, `planning_simulation`. |
| `launch_system_monitor`                  | `true`                                                                           | Launch the system monitor.                                                                         |
| `launch_dummy_diag_publisher`            | `false`                                                                          | Launch the dummy diagnostic publisher.                                                             |
| `diagnostic_graph_aggregator_graph_path` | `$(find-pkg-share autoware_launch)/config/system/diagnostics/autoware-main.yaml` | The graph file of the diagnostic graph aggregator.                                                 |

For example, to use your own diagnostic graph, launch Autoware with this command-line argument:

```bash
ros2 launch autoware_launch autoware.launch.xml ... diagnostic_graph_aggregator_graph_path:=<YOUR-DIAGNOSTIC-GRAPH-PATH> ...
```

The parameter files of the system packages are in the `config/system/` directory of the `autoware_launch` package. `tier4_system_component.launch.xml` gives their paths to `system.launch.xml` as fixed values, so a command-line argument cannot change them. To use another parameter file, change the path in your `tier4_system_component.launch.xml`. For example, for the parameter file of `mrm_handler`:

```diff
- <arg name="mrm_handler_param_path" value="$(find-pkg-share autoware_launch)/config/system/mrm_handler/mrm_handler.param.yaml"/>
+ <arg name="mrm_handler_param_path" value="<YOUR-MRM-HANDLER-PARAM-PATH>"/>
```
