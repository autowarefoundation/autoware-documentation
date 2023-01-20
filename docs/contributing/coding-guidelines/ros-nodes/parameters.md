# Parameters

## ROS parameters

The ROS packages in Autoware have ROS parameters. You need to customize the parameters depending on your applications.
It is recommended not to set default values when you declare ROS parameters to avoid unintended behaviors.

## Parameter files

Autoware has the following two types of parameter files for ROS packages:

- **Node parameter file**
  - Node parameter files store the default parameters provided for each package in Autoware.
    - For example, [the parameter of `behavior_path_planner`](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/behavior_path_planner/config)
  - All the nodes in Autoware **must** have the node parameter file.
  - For `FOO_package`, the parameter is expected to be stored in `FOO_package/config`.
  - The launch file for individual packages must load node parameter by default:

```xml
<launch>
  <arg name="foo_node_param_path" default="$(find-pkg-share FOO_package)/config/foo_node.param.yaml" />

  <node ...>
    <param from="$(var foo_node_param_path)" />
  </node>
</launch>
```

- **Launch parameter file**
  - Launch parameter files store the customized parameters for user's vehicle.
    - For example, [the customized parameter of `behavior_path_planner` stored under `autoware_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config)
  - Launch parameter files are stored under `autoware_launch`.

All the parameter files should have the `.param.yaml` suffix so that the auto-format can be applied properly.
