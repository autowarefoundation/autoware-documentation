# Using divided pointcloud map

Divided pointcloud map is necessary when handling large pointcloud map, in which case Autoware may not be capable of sending the whole map via ROS 2 topic or loading the whole map into memory. By using the pre-divided map, Autoware will dynamically load the pointcloud map according to the vehicle's position.

## Tutorial

In this tutorial, you divide the pointcloud map of the rosbag replay demo into a grid of small PCD files. Then you start the logging simulator with the divided map.

### Download the sample map

Use the `demo_artifacts` Ansible role to download the sample map, as in [the demo for rosbag replay simulation](../../demos/rosbag-replay-simulation.md):

```bash
cd ~/autoware
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
ansible-playbook autoware.dev_env.install_dev_env --tags demo_artifacts --ask-become-pass
```

The map is then in `~/autoware_data/maps/sample-map-rosbag/`. It contains `lanelet2_map.osm`, `pointcloud_map.pcd`, and `map_projector_info.yaml`.

### Build the pointcloud divider

The [`autoware_pointcloud_divider`](https://github.com/autowarefoundation/autoware_tools/tree/main/map/autoware_pointcloud_divider) package is in the `autoware_tools` repository. Import `repositories/tools.repos` into your workspace and build the package:

```bash
cd ~/autoware
vcs import src < repositories/tools.repos
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to autoware_pointcloud_divider
```

### Divide the map

Run the divider. It reads `pointcloud_map.pcd` of the sample map and writes the new map to `output_pcd_dir`. The paths must be absolute. A negative `leaf_size` keeps all points (no downsampling). Each output file covers a grid cell of 20 m by 20 m by default.

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_pointcloud_divider pointcloud_divider.launch.xml \
  input_pcd_or_dir:=$HOME/autoware_data/maps/sample-map-rosbag/pointcloud_map.pcd \
  output_pcd_dir:=$HOME/autoware_data/maps/sample-map-rosbag-divided \
  prefix:=pointcloud_map \
  leaf_size:=-1.0
```

!!! warning

    The divider deletes the folder `output_pcd_dir` before it starts, if the folder exists. Do not give it a folder that holds files you want to keep.

The divider creates a folder named `pointcloud_map.pcd` that holds the PCD files, and a `pointcloud_map_metadata.yaml` file next to it. Copy the other map files into the new map folder:

```bash
cp ~/autoware_data/maps/sample-map-rosbag/{lanelet2_map.osm,map_projector_info.yaml} ~/autoware_data/maps/sample-map-rosbag-divided/
```

The new map folder has this structure:

```text
sample-map-rosbag-divided
├── lanelet2_map.osm
├── map_projector_info.yaml
├── pointcloud_map.pcd
│   ├── pointcloud_map_<ix>_<iy>.pcd
│   └── ...
└── pointcloud_map_metadata.yaml
```

The numbers `<ix>` and `<iy>` are the indices of the grid cell. The metadata file gives the size of the grid cells and the position of each PCD file.

### Launch Autoware with the divided map

The `pointcloud_map_file` argument names the pointcloud map in `map_path`. It can be one PCD file or a folder of PCD files, and its default value is `pointcloud_map.pcd`. If your folder of PCD files has another name, set this argument to that name.

Launch the logging simulator with the divided map:

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:=$HOME/autoware_data/maps/sample-map-rosbag-divided pointcloud_map_file:=pointcloud_map.pcd \
  vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

For playing rosbag to simulate Autoware, please refer to the instruction in [the demo for rosbag replay simulation](../../demos/rosbag-replay-simulation.md).

## Related links

- For specific format definition of the divided map, please refer to [Map component design page](../../design/autoware-architecture-v1/components/map/index.md)
- [The Readme of map_loader](https://github.com/autowarefoundation/autoware_core/tree/main/map/autoware_map_loader) explains the metadata file and the partial and differential loading of the pointcloud map
- [The Readme of pointcloud_divider](https://github.com/autowarefoundation/autoware_tools/tree/main/map/autoware_pointcloud_divider) explains all options of the divider, for example downsampling and larger grid cells
