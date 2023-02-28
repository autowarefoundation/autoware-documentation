# Map

![Node diagram](./images/Map-Bus-ODD-Architecture.drawio.svg)

## Inputs

Autoware relies on high-definition point cloud maps and vector maps of the driving environment to perform various tasks. Before launch Autoware, you need to load the pre-created map files.

How to create maps refference [Creating maps for Autoware](../../../how-to-guides/creating-maps-for-autoware.md).
 
## Outputs

### Point cloud map

Loads pointcloud file and publishes the maps to the other Autoware nodes in various configurations. Currently, it supports the following types:

- Raw pointcloud map (sensor_msgs/msg/PointCloud2)
- Downsampled pointcloud map (sensor_msgs/msg/PointCloud2)
- Partial pointcloud map loading via ROS 2 service (autoware_map_msgs/srv/GetPartialPointCloudMap) 
- Differential pointcloud map loading via ROS 2 service (autoware_map_msgs/srv/GetDifferentialPointCloudMap)

### Lanlet2 map

Loads Lanelet2 file and publishes the map data as `autoware_auto_mapping_msgs/msg/HADMapBin` message. The lan/lon coordinates is projected into the MGRS coordinates.

- autoware_auto_mapping_msgs/msg/HADMapBin
    - std_msgs/Header header
    - string version_map_format
    - string version_map
    - string name_map
    - uint8[] data

 
### Lanlet2 map visualization

visualize `autoware_auto_mapping_msgs/HADMapBin` messages in `Rviz`.

- visualization_msgs/msg/MarkerArray