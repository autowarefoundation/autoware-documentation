# Map launch files

## Overview

The Autoware map stack starts with `autoware.launch.xml`, as the [Launch Autoware](../index.md) page explains. If `launch_map` is `true`, `autoware.launch.xml` includes [`component_map.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_launch/launch/components/component_map.launch.xml) of the `autoware_launch` package. This file includes the launch files of the [`autoware_map_launch`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_map_launch) package. The following diagram shows the map launch flow in the `autoware_launch` repository.

{{ diagram("images/map-launch-flow.svg", "component_map.launch.xml includes five launch files of autoware_map_launch in the map namespace, and each one starts its map nodes.") }}

!!! note

    The Autoware project is a large project.
    Therefore, as we manage the Autoware project, we utilize specific
    arguments in the launch files.
    ROS 2 offers an argument-overriding feature for these launch files.
    Please refer to [the official ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameter-overrides) for further information.
    For instance,
    if we define an argument at the top-level launch,
    it will override the value on lower-level launches.

`component_map.launch.xml` makes the paths of the map files from `map_path`, `pointcloud_map_file`, and `lanelet2_map_file`. It also gives the nodes the paths of `pointcloud_map_metadata.yaml`, `lanelet2_map_metadata.yaml`, and `map_projector_info.yaml` in `map_path`. Then, in the `/map` namespace, it starts these nodes:

| Argument                 | Launch file                          | Nodes                                                                                                                  |
| ------------------------ | ------------------------------------ | ---------------------------------------------------------------------------------------------------------------------- |
| `use_pointcloud_map`     | `pointcloud_map_loader.launch.xml`   | `pointcloud_map_loader` of `autoware_map_loader`                                                                       |
| `use_lanelet2_map`       | `lanelet2_map_loader.launch.xml`     | `lanelet2_map_loader` of `autoware_map_loader`, and `lanelet2_map_visualization` of `autoware_lanelet2_map_visualizer` |
| `use_map_tf_generator`   | `vector_map_tf_generator.launch.xml` | `vector_map_tf_generator` of `autoware_map_tf_generator`                                                               |
| `use_map_hash_generator` | `map_hash_generator.launch.xml`      | `map_hash_generator` of `autoware_map_loader`                                                                          |
| `use_map_projection`     | `map_projection_loader.launch.xml`   | The map projection loader of `autoware_map_projection_loader`                                                          |

All these arguments are `true` by default. In Autoware 2.0.0, the map nodes run as separate processes. They are not in a `map_container` any more. `component_map.launch.xml` also includes `agnocast_env.launch.xml` of `autoware_agnocast_wrapper`, which selects the executor of the nodes. The parameter files of the map nodes are in the [`autoware_map_config`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_map_config) package.

You do not have many configuration choices in the map launch files, because the parameters are in the configuration files. But you can set the names of your point cloud map and your Lanelet2 map when you launch Autoware. The default values are `pointcloud_map.pcd` and `lanelet2_map.osm`. For example, to change the names of your map files, launch Autoware with these command-line arguments:

```bash
ros2 launch autoware_launch autoware.launch.xml ... pointcloud_map_file:=<YOUR-PCD-FILE-NAME> lanelet2_map_file:=<YOUR-LANELET2-MAP-NAME> ...
```

Or you can change them in your `autoware.launch.xml` launch file:

```diff
- <arg name="lanelet2_map_file" default="lanelet2_map.osm" description="lanelet2 map file name"/>
+ <arg name="lanelet2_map_file" default="<YOUR-LANELET2-MAP-NAME>" description="lanelet2 map file name"/>
- <arg name="pointcloud_map_file" default="pointcloud_map.pcd" description="pointcloud map file name"/>
+ <arg name="pointcloud_map_file" default="<YOUR-PCD-FILE-NAME>" description="pointcloud map file name"/>
```
