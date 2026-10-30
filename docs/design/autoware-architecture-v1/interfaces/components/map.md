# Map

This page describes the interface of the Map component. See the [Map component design](../../components/map/index.md) for the concepts.

Autoware uses high-definition point cloud maps and vector maps of the driving environment. Before you launch Autoware, you must create the map files. See [Creating maps](../../../../tutorials/integrating-autoware/creating-maps/index.md).

{{ diagram("images/Map-Bus-ODD-Architecture.svg", "The main nodes of the Map component and the components that use the maps.") }}

## Inputs

The Map component reads files. It does not subscribe to topics from other components.

| Name                     | File                               | Description                                                                                                                                                                                                                                              |
| ------------------------ | ---------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Point cloud map          | `*.pcd` files, or a folder of them | The 3D point cloud map. Localization and Perception use it.                                                                                                                                                                                              |
| Point cloud map metadata | `pointcloud_map_metadata.yaml`     | The grid cells of a divided point cloud map. Partial and differential loading use it.                                                                                                                                                                    |
| Lanelet2 map             | `*.osm` file, or a folder of them  | The vector map in the [Autoware Lanelet2 format](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/docs/lanelet2_format_extension.md).                                                             |
| Lanelet2 map metadata    | `lanelet2_map_metadata.yaml`       | The grid cells of a divided Lanelet2 map. A Lanelet2 map in more than one file must have it. For a map in one file, the loader calculates the cell from the map if the file does not exist.                                                              |
| Map projector info       | `map_projector_info.yaml`          | The projection from latitude and longitude to the local `map` frame. If the file does not exist, the projection loader reads the Lanelet2 map. It uses `MGRS` if a point of the map has latitude and longitude values, and `Local` if no point has them. |

## Outputs

### Map projector info

The `autoware_map_projection_loader` node publishes the projection of the map.

| Name               | Topic                     | Type                                                                                                                                               | Description                                                                                                                            |
| ------------------ | ------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| Map projector info | `/map/map_projector_info` | [autoware_map_msgs/msg/MapProjectorInfo](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/msg/MapProjectorInfo.msg) | The projector type and its parameters. The types are `MGRS`, `LocalCartesianUTM`, `LocalCartesian`, `TransverseMercator`, and `Local`. |

### Point cloud map

The `autoware_pointcloud_map_loader` node of the [`autoware_map_loader`](https://github.com/autowarefoundation/autoware_core/tree/main/map/autoware_map_loader) package loads the point cloud files. It sends the map to the other nodes in the configurations below. The parameter file of the node turns each configuration on or off. The metadata topic and the differential loading service are always on.

| Name                                 | Topic or service                                 | Type                                                                                                                                                                       | Default | Description                                                                                       |
| ------------------------------------ | ------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | ------------------------------------------------------------------------------------------------- |
| Whole point cloud map                | `/map/pointcloud_map`                            | [sensor_msgs/msg/PointCloud2](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg)                                                      | On      | The whole map in one message.                                                                     |
| Point cloud map metadata             | `/map/pointcloud_map_metadata`                   | [autoware_map_msgs/msg/PointCloudMapMetaData](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/msg/PointCloudMapMetaData.msg)               | Always  | The cells of the map and their areas.                                                             |
| Partial point cloud map loading      | `/map/get_partial_pointcloud_map` (service)      | [autoware_map_msgs/srv/GetPartialPointCloudMap](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/srv/GetPartialPointCloudMap.srv)           | On      | Returns the cells that overlap a requested area.                                                  |
| Differential point cloud map loading | `/map/get_differential_pointcloud_map` (service) | [autoware_map_msgs/srv/GetDifferentialPointCloudMap](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/srv/GetDifferentialPointCloudMap.srv) | Always  | Returns the cells in a requested area that the client does not have yet, and the cells to remove. |
| Selected point cloud map loading     | `/map/get_selected_pointcloud_map` (service)     | [autoware_map_msgs/srv/GetSelectedPointCloudMap](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/srv/GetSelectedPointCloudMap.srv)         | Off     | Returns the cells with the requested IDs.                                                         |
| Downsampled point cloud map          | `/map/output/debug/downsampled_pointcloud_map`   | [sensor_msgs/msg/PointCloud2](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg)                                                      | Off     | The whole map after a voxel grid filter, for visualization.                                       |

See the [README of autoware_map_msgs](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/README.md) for the area queries of these services.

### Lanelet2 map

The `autoware_lanelet2_map_loader` node of the `autoware_map_loader` package loads the Lanelet2 file and publishes it as one binary message. It projects the latitude and longitude of each point to the `map` frame with the map projector info.

| Name                          | Topic or service                         | Type                                                                                                                                                           | Description                                                                  |
| ----------------------------- | ---------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------- |
| Lanelet2 map                  | `/map/vector_map`                        | [autoware_map_msgs/msg/LaneletMapBin](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/msg/LaneletMapBin.msg)                   | The whole Lanelet2 map. The QoS durability is `transient_local`.             |
| Lanelet2 map metadata         | `/map/vector_map_metadata`               | [autoware_map_msgs/msg/LaneletMapMetaData](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/msg/LaneletMapMetaData.msg)         | The cells of a divided Lanelet2 map. Only with selected loading.             |
| Selected Lanelet2 map loading | `/map/get_selected_vector_map` (service) | [autoware_map_msgs/srv/GetSelectedLanelet2Map](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/srv/GetSelectedLanelet2Map.srv) | Returns the Lanelet2 map of the requested cells. Only with selected loading. |

The `LaneletMapBin` message has these fields:

- std_msgs/Header header
- string version_map_format
- string version_map
- string name_map
- uint8[] data

The `GetSelectedLanelet2Map` service has this request and response:

- Request:
  - string[] cell_ids
- Response:
  - std_msgs/Header header
  - autoware_map_msgs/LaneletMapBin lanelet2_cells

!!! note

    Selected Lanelet2 map loading is off by default. To turn it on, set `enable_selected_map_loading` to `true` in the parameter file of the Lanelet2 map loader.

### Lanelet2 map visualization

The `autoware_lanelet2_map_visualizer` node turns `/map/vector_map` into markers for RViz.

| Name                 | Topic                    | Type                                                                                                                                | Description                             |
| -------------------- | ------------------------ | ----------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------- |
| Lanelet2 map markers | `/map/vector_map_marker` | [visualization_msgs/msg/MarkerArray](https://github.com/ros2/common_interfaces/blob/rolling/visualization_msgs/msg/MarkerArray.msg) | The lanes, lines, and areas of the map. |

### Viewer frame

The `autoware_vector_map_tf_generator` node publishes a static transform from `map` to `viewer` at the mean position of the points of the Lanelet2 map. RViz uses the `viewer` frame to show the map.

### Map hash

The `map_hash_generator` node of the `autoware_map_loader` package identifies the loaded map for applications outside Autoware. It calculates the SHA-256 hash of the Lanelet2 map file and of the point cloud map files once, when it starts.

| Name             | Topic or service                              | Type                                                                                                                                                    | Description                                                           |
| ---------------- | --------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------- |
| Map hash         | `/api/autoware/get/map/info/hash`             | [tier4_external_api_msgs/msg/MapHash](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/msg/MapHash.msg)         | The hashes of the map files. The QoS durability is `transient_local`. |
| Lanelet2 map XML | `/api/autoware/get/map/lanelet/xml` (service) | [tier4_external_api_msgs/srv/GetTextFile](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/srv/GetTextFile.srv) | Returns the content of the Lanelet2 map file as text.                 |
