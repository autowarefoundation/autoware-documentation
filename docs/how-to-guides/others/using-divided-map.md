# Using divided pointcloud map

Divided pointcloud map is necessary when handling large pointcloud map, in which case Autoware may not be capable of sending the whole map via ROS 2 topic or loading the whole map into memory. By using the pre-divided map, Autoware will dynamically load the pointcloud map according to the vehicle's position. For specific format definition of the divided map, please refer to [Map component design page](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/map/).

## Tutorial

Download the [sample-map-rosbag_split](TODO) and locate the map under `$HOME/autoware_map/`.

```bash
gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1VnwJx9tI3kI_cTLzP61ktuAJ1ChgygpG' # TODO
unzip -d ~/autoware_map/ ~/autoware_map/sample-rosbag_split.zip
```

Then, you may launch logging_simulator with the following command to load the divided map.
Note that you need to specify the `map_path` and `pointcloud_map_file` arguments.

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-rosbag pointcloud_map_file:=pointcloud_map \
  vehicle_model:=sample_vehicle_split sensor_model:=sample_sensor_kit
```
