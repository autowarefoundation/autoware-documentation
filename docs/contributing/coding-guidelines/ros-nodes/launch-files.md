# Launch files

## Overview

Autoware uses the ROS 2 launch system to start the software. Please see the [official documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html) to get a basic understanding about ROS 2 Launch system if you are not familiar with it.

## Guideline

### The organization of launch files in Autoware

Autoware distinguishes between the reusable node implementations and their example integration, including system-specific configurations, pipelines, and system topologies.

- Reusable node implementations can be found in [`autoware_core`](https://github.com/autowarefoundation/autoware_core) and [`autoware_universe`](https://github.com/autowarefoundation/autoware_universe)
  - `autoware_core` repository itself provides a minimal integration in the package named `autoware_core`.
- An integrated system example orchestrated from these nodes can be found in [`autoware_launch`](https://github.com/autowarefoundation/autoware_launch).
  - There are many possible ways to construct a full autonomous driving system, and `autoware_launch` provides one, highly configurable reference integration.

The `autoware_launch` repository (version 0.53.0 in Autoware 2.0.0) contains these packages:

- `autoware_launch`: the entry points and the component launch files in `launch/components`.
- `autoware_universe_launch/autoware_<component>_launch`: the launch files of each component, for example `autoware_localization_launch`.
- `autoware_universe_launch/autoware_<component>_config`: the parameter files of each component, for example `autoware_localization_config`.
- `tier4_universe_launch/tier4_<name>_launch`: the launch files of the system, the API, the simulator, and the vehicle.
- `sensor_kit/<name>_sensor_kit_launch`: the launch and description packages of each sensor kit, for example `sample_sensor_kit_launch` and `sample_sensor_kit_description`.
- `vehicle/<name>_vehicle_launch`: the launch and description packages of each vehicle, for example `sample_vehicle_launch` and `sample_vehicle_description`.
- `autoware_sample_designs`: an example that defines the system with [Autoware System Designer](https://github.com/autowarefoundation/autoware_system_designer/tree/v0.4.2) and generates the launch files from it.

The package `autoware_launch` itself provides the general entrypoint to call other modularized launch files and start the Autoware nodes.

- The `autoware.launch.xml` is the basic launch file for road driving scenarios.

  This launch file loads other launch files for different modules, including _Vehicle_, _System_, _Map_, _Sensing_, _Localization_, _Perception_, _Planning_, _Control_, etc. By setting the `launch_*` argument to either `true` or `false` , the users can selectively load a subset of the system.

- The `logging_simulator.launch.xml` is often used together with the recorded ROS bag to debug if the target module (e.g, _Sensing_, _Localization_ or _Perception_) functions normally.

- The `planning_simulator.launch.xml` is based on the Planning Simulator tool, mainly used for testing/validation of _Planning_ module by simulating traffic rules, interactions with dynamic objects and control commands to the ego vehicle.

- The `e2e_simulator.launch.xml` is the launcher for digital twin simulation environment. Its `simulator_type` argument selects AWSIM (`awsim`, the default) or CARLA (`carla`).

The next diagram shows how these launch files include each other, with the localization component as an example. The names are from `autoware_launch` 0.53.0.

{{ diagram("images/launch-file-tree.svg", "The simulator launch files include autoware.launch.xml. It includes the component launch files. These include the launch files of autoware_localization_launch, which include the launch files of the node packages.") }}

### Add a new package in Autoware

If a newly created package has executable node, we expect example launch files and configurations within the package, just like the recommended structure shown in the previous [directory structure](directory-structure.md) page.

### Integrate a new package in `autoware_launch`

In order to automatically load the newly added package (in `autoware_core` or `autoware_universe`) when starting Autoware, you need to make some necessary changes to the corresponding launch file.

For example, to add a node to the localization component:

1. Include the launch file of your package from the correct launch file in `autoware_universe_launch/autoware_localization_launch/launch`. Some node packages are included directly from `autoware_launch/launch/components/component_localization.launch.xml`, for example `autoware_pose_estimator_arbiter`.
2. Add the launch parameter file of your node to `autoware_universe_launch/autoware_localization_config/config`, and pass its path to your launch file.
3. Register the pair of the node parameter file and the launch parameter file in the sync-params configuration. See [sync-params](parameters.md#sync-params).

## Parameter and system topology management

Another purpose of introducing the `autoware_launch` repository is to facilitate the parameter and system topology management of Autoware.

Suppose that we want to integrate Autoware based on `autoware_launch` to a specific vehicle, and that we are only interested in different parameters and possibly different node configurations, without rewriting the existing node implementations.
In such case, we can **fork only `autoware_launch`** to customize the parameters or the pipelines, without modifying the official `autoware_universe`.

Taking the localization module as an example, in the `autoware_launch` repository:

1. all the launch parameter files for the localization component are in [`autoware_universe_launch/autoware_localization_config/config`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_localization_config).
2. [`autoware_launch/launch/components/component_localization.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_launch/launch/components/component_localization.launch.xml) has the `localization_config_pkg` argument. It names the package that holds the parameter files. The default is `autoware_localization_config`.
3. in [`autoware_universe_launch/autoware_localization_launch/launch`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_localization_launch/launch), the launch files make the parameter file paths from `localization_config_pkg`, for example the `ekf_localizer_param_path` argument of `pose_twist_fusion_filter.launch.xml`. They give these paths to the launch files of the node packages. You can still use the default parameters in each package, because the launch file of each node package has a default parameter file.

To use your own parameter files for all the localization nodes, set `localization_config_pkg` to your own package with the same `config/` layout. To change only a few files, override the `*_param_path` arguments.

See [sync-params](parameters.md#sync-params) to learn how these launch parameter files can be updated.
