# Parameters

## ROS parameters

The ROS packages in Autoware may have ROS parameter(s) for setting some variables that should be customized depending on the applications.
It is recommended not to set default values when you declare ROS parameters to avoid unintended behaviors.

## ROS Parameter files

Autoware has the following two types of parameter files for ROS packages:

- Node parameter file: the reference parameter for the node package
  - e.g., [the parameter for the `behavior_path_planner` package](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/behavior_path_planner/config)
- Launch parameter file: [the reference parameter for the `autoware_launch` package](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config)

All the parameter files should have the `.param.yaml` suffix so that the auto-format can be applied properly.

### Node parameter

Node parameter files store the default parameters provided for each package in Autoware Universe.
All the nodes in Autoware **must** have the node parameter file.
For `FOO_package`, the parameter is expected to be stored in `FOO_package/config`.
Also, the launch file for individual packages must load node parameter by default.

### Launch parameter

Launch parameter files store the parameters that are used when launching Autoware, which are stored under `autoware_launch` repository. Basically, launch parameters are the subset of node parameters. It is expected that the users tune launch parameters in `autoware_launch`, and thus, a node parameter file should also be added to launch parameter if the parameter may be tuned depending on the vehicle.
