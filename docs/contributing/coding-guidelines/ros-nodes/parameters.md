# Parameters

The ROS packages in Autoware have ROS parameters. You need to customize the parameters depending on your applications.
It is recommended not to set default values when declaring ROS parameters to avoid unintended behaviors due to accidental use of default values.
Instead, set parameters from configuration files named `*.param.yaml`.

For understanding ROS 2 parameters, also check out the official documentation [Understanding parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html).

## Parameter files

Autoware has the following two types of parameter files for ROS packages:

- **Node parameter file**
  - Node parameter files store the default parameters provided for each package in Autoware.
    - For example, [the parameter of `behavior_path_planner`](https://github.com/autowarefoundation/autoware.universe/tree/245242cee866de2d113e89c562353c5fc17f1f98/planning/behavior_path_planner/config)
  - All nodes in Autoware must have a parameter file if one or more parameters that can be customized by the user are defined.
  - For `FOO_package`, the parameter is expected to be stored in `FOO_package/config`.
  - The launch file for individual packages must load node parameter by default:

```xml
<launch>
  <arg name="foo_node_param_path" default="$(find-pkg-share FOO_package)/config/foo_node.param.yaml" />

  <node pkg="FOO_package" exec="foo_node">
    ...
    <param from="$(var foo_node_param_path)" />
  </node>
</launch>
```

- **Launch parameter file**
  - Launch parameter files store the customized parameters for user's vehicle.
    - For example, [the customized parameter of `behavior_path_planner` stored under `autoware_launch`](https://github.com/autowarefoundation/autoware_launch/tree/5fa613b9d80bf4f0db77efde03a43f7ede6bac86/autoware_launch/config)
  - Launch parameter files are stored under `autoware_launch`.

All the parameter files should have the `.param.yaml` suffix so that the auto-format can be applied properly.
