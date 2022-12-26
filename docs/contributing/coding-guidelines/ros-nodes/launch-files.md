# Launch files

## Overview

Autoware use ROS2 launch system to startup the software. Please see the [official documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html) to get a basic understanding about ROS 2 Launch system if you are not familiar with it.

## Guideline

### The organization of launch files in Autoware

The [Autoware launch repository](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch) is the general entrance to start the Autoware software stacks. It is responsible for calling the launch file of each module to load the functions of corresponding module.

- The `autoware.launch.xml` is the basic launch file for road driving scenarios.

  As can be seen from the content, the entire launch file is divided into several different modules, including _Vehicle_, _System_, _Map_, _Sensing_, _Localization_, _Perception_, _Planning_, _Control_, etc. By setting `true` or `false`, we can determine which modules to be loaded.

- The `logging_simulator.launch.xml` is often used together with the recorded ROS bag to debug if the target module (e.g. _Sensing_, _Localization_ or _Perception_) functions normally.

- The `planning_simulator.launch.xml` is based on the Planning Simulator tool, mainly used for testing/validation of _Planning_ module by simulating traffic rules, interactions with dynamic objects and control commands to the ego vehicle.

In Autoware, the nodes of each module is organized based on the [architecture](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/#high-level-architecture-design). So you may find that we try to match [launch structure](https://github.com/autowarefoundation/autoware.universe/tree/main/launch) similar to the architecture (splitting of files, namespace).

```txt
                                                        autoware_launch
                                                                |
─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
|                       |                           |                       |                       |                       |
tier4_vehicle_launch    tier4_localization_launch   tier4_system_launch tier4_sensing_launch    tier4_perception_launch    ...
                                |
─────────────────────────────────────────────────────────────────
|               |                   |                           |
pose_estimator twist_estimator    pose_twist_fusion_filter     ...
                                        |
                    ─────────────────────────────────────────
                    |                   |                   |
                    ekf_localizer    stop_filter        twist2accel
```

### Create or add a new package in Autoware

If a newly created package has executable node, we expect sample launch file and configuration within the package, just like the recommended structure shown in previous [directory structure](https://autowarefoundation.github.io/autoware-documentation/main/contributing/coding-guidelines/ros-nodes/directory-structure/) page.

In order to automatically load the newly added package when starting Autoware, you may choose to call its launch file at certain place. For example, if using ICP instead of NDT as the pointcloud registration algorithm, you can modify the `tier4_localization_launch/launch/pose_estimator/pose_estimator.launch.xml` file to load the newly added ICP package.

## Future updates

How we organize configuration files.
