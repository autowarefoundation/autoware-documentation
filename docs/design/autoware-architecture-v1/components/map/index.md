# Map component design

## Overview

Autoware uses high-definition maps of the driving environment for localization, route planning, traffic light detection, and the prediction of the paths of pedestrians and other vehicles.

This page describes the map component of Autoware: its requirements, architecture, interfaces, and the formats of the map files.

## Requirements

The map component gives two types of information to the rest of the stack:

- Semantic information about the roads, as a vector map.
- Geometric information about the environment, as a point cloud map. This map is optional.

A vector map contains accurate information about the road network, the lane geometry, and the traffic lights. Route planning, traffic light detection, and the prediction of the paths of other vehicles and pedestrians need the vector map.

Autoware uses the 3D point cloud map for LiDAR-based localization and for some perception functions. To find the position and orientation of the vehicle, the localization matches a live scan from one or more LiDARs against the point cloud map. As a result, the accuracy of the point cloud map controls the accuracy of the localization. If the vehicle has a different localization method with enough accuracy, for example camera-based localization, Autoware can run without a point cloud map.

Autoware also needs a projection information file. This file defines how the local map coordinates relate to the geodetic coordinates (latitude, longitude, and altitude).

## Architecture

{{ diagram("image/high-level-map-diagram.svg", "The loader nodes read the map files and publish the maps to the other components.") }}

The map component runs these nodes in the `/map` namespace:

| Node                         | Package                                                                                                                                  | Role                                                                                                                                            |
| :--------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------------------- |
| `map_projection_loader`      | [`autoware_map_projection_loader`](https://github.com/autowarefoundation/autoware_core/tree/main/map/autoware_map_projection_loader)     | Loads the projection information and publishes it. The other nodes use it to convert between local and geodetic coordinates.                    |
| `lanelet2_map_loader`        | [`autoware_map_loader`](https://github.com/autowarefoundation/autoware_core/tree/main/map/autoware_map_loader)                           | Loads one Lanelet2 file, or all the Lanelet2 files of a directory, projects them with the projection information, and publishes the vector map. |
| `pointcloud_map_loader`      | [`autoware_map_loader`](https://github.com/autowarefoundation/autoware_core/tree/main/map/autoware_map_loader)                           | Loads one or more PCD files, publishes the point cloud map, and serves parts of the map through services.                                       |
| `lanelet2_map_visualization` | [`autoware_lanelet2_map_visualizer`](https://github.com/autowarefoundation/autoware_core/tree/main/map/autoware_lanelet2_map_visualizer) | Converts the vector map into markers for RViz.                                                                                                  |
| `vector_map_tf_generator`    | [`autoware_map_tf_generator`](https://github.com/autowarefoundation/autoware_universe/tree/main/map/autoware_map_tf_generator)           | Broadcasts the `viewer` frame at the center of the vector map. Only RViz uses this frame.                                                       |
| `map_hash_generator`         | [`autoware_map_loader`](https://github.com/autowarefoundation/autoware_core/tree/main/map/autoware_map_loader)                           | Publishes the hashes of the map files and serves the Lanelet2 file as text for the map API.                                                     |

All the packages are in Autoware Core, except `autoware_map_tf_generator`, which is in Autoware Universe.

The launch file [`component_map.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/component_map.launch.xml) of `autoware_launch` starts the map component. It includes one launch file of the `autoware_map_launch` package for each feature. The arguments `use_pointcloud_map`, `use_lanelet2_map`, `use_map_projection`, `use_map_tf_generator`, and `use_map_hash_generator` turn each feature on or off. All of them are `true` by default.

The launch file reads the map files from the directory in the `map_path` argument:

```bash
map_path
├── lanelet2_map.osm               # the vector map (the lanelet2_map_file argument)
├── pointcloud_map.pcd             # the point cloud map (the pointcloud_map_file argument), a file or a directory
├── pointcloud_map_metadata.yaml   # needed for a divided point cloud map
├── lanelet2_map_metadata.yaml     # optional, for a divided vector map
└── map_projector_info.yaml        # the projection information
```

## Component interface

### Input to the map component

The map component reads only files:

- The point cloud map and its metadata file
- The vector map
- The projection information

### Output from the map component

| Output                 | Topic or service                                                                                              | Type                                                                                                                                                    | Users                                                                                                                                                                                                                                                                                                       |
| :--------------------- | :------------------------------------------------------------------------------------------------------------ | :------------------------------------------------------------------------------------------------------------------------------------------------------ | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Projection information | `/map/map_projector_info`                                                                                     | `autoware_map_msgs/msg/MapProjectorInfo`                                                                                                                | Sensing: `autoware_gnss_poser` converts the GNSS position into local coordinates. Localization: `autoware_geo_pose_projector`. AD API: `autoware_default_adapi_universe` converts the vehicle position into geodetic coordinates. The Lanelet2 map loader also uses it.                                     |
| Vector map             | `/map/vector_map`                                                                                             | `autoware_map_msgs/msg/LaneletMapBin`                                                                                                                   | Localization: the pose initializer, YabLoc, and the landmark-based localizers. Perception: `autoware_map_based_prediction` predicts the paths of vehicles and pedestrians, and the traffic light recognition finds the traffic lights in the camera images. Planning: route and behavior planning. Control. |
| Point cloud map        | `/map/pointcloud_map`                                                                                         | `sensor_msgs/msg/PointCloud2`                                                                                                                           | Perception: for example, obstacle segmentation that compares the LiDAR scan with the map. The loader publishes the whole map only when its parameter `enable_whole_load` is `true` (the default in `autoware_launch`).                                                                                      |
| Point cloud map parts  | `/map/get_partial_pointcloud_map`, `/map/get_differential_pointcloud_map`, `/map/get_selected_pointcloud_map` | `autoware_map_msgs/srv/GetPartialPointCloudMap`, `autoware_map_msgs/srv/GetDifferentialPointCloudMap`, `autoware_map_msgs/srv/GetSelectedPointCloudMap` | Localization: `autoware_ndt_scan_matcher` loads the map around the vehicle, and the pose initializer fits the initial pose to the ground. Perception: the compare map filter of the obstacle segmentation, when it loads the map dynamically.                                                               |
| Point cloud metadata   | `/map/pointcloud_map_metadata`                                                                                | `autoware_map_msgs/msg/PointCloudMapMetaData`                                                                                                           | Nodes that use the services. The loader always publishes it.                                                                                                                                                                                                                                                |
| Map hash               | `/api/autoware/get/map/info/hash`                                                                             | `tier4_external_api_msgs/msg/MapHash`                                                                                                                   | The external API, to identify the loaded map.                                                                                                                                                                                                                                                               |

## Map specification

### Point cloud map

The point cloud map must meet these requirements:

- The point cloud map must use the same projection as the one that `map_projection_loader` publishes. Then it agrees with the Lanelet2 map and with the packages that convert between local and geodetic coordinates. For more information, see [the README of `autoware_map_projection_loader`](https://github.com/autowarefoundation/autoware_core/tree/main/map/autoware_map_projection_loader/README.md).
- It must be in the [PCD (Point Cloud Data) file format](https://pointclouds.org/documentation/tutorials/pcd_file_format.html). It can be one PCD file or many PCD files (a divided map).
- Each point in the map must contain X, Y, and Z coordinates.
- Each point can also contain an intensity or an RGB value.
- It must cover the entire operational area of the vehicle. Add a buffer zone around this area that matches the detection range of the sensors on the vehicle.
- Its resolution must be at least 0.2 m to give reliable localization results.
- It can be in local or global coordinates. To use GNSS data for localization, it must be in global coordinates (georeferenced).

For a large map, use a divided map. The point cloud map loader can then send only the parts of the map near the vehicle (partial and differential loading). A divided map must be cut along lines parallel to the x-axis and the y-axis into cells of the same size, about 20 m by 20 m. It also needs a metadata file. [`autoware_pointcloud_divider`](https://github.com/autowarefoundation/autoware_tools/tree/main/map/autoware_pointcloud_divider) can divide a map and write the metadata file. For the details of the divided map format, see [the README of `autoware_map_loader`](https://github.com/autowarefoundation/autoware_core/blob/main/map/autoware_map_loader/README.md).

### Vector map

The vector map must meet these requirements:

- It must be in [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) format, with the [additional rules of Autoware](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/docs/lanelet2_format_extension.md).
- It must contain the shape and position of the lanes, traffic lights, stop lines, crosswalks, parking spaces, and parking lots.
- Except at the start or end of a road, each lanelet must have correct connections to its predecessor, successors, left neighbor, and right neighbor.
- Each lanelet must contain its traffic rules: the speed limit, the right of way, the traffic direction, and the related traffic lights, stop lines, and traffic signs.
- It must cover the entire operational area of the vehicle.

The vector map can be one `.osm` file or a directory of `.osm` files. The Lanelet2 map loader merges all the files of a directory into one map. A map with more than one file needs a `lanelet2_map_metadata.yaml` file.

For the detailed rules to create a vector map, see the [vector map creation requirement specifications](map-requirements/vector-map-requirements-overview/index.md).

### Projection information

The projection information is a YAML file, `map_projector_info.yaml`, that `map_projection_loader` reads. It contains these fields:

| Field            | Use                                                                                                                                                                  |
| :--------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `projector_type` | The projection between local and geodetic coordinates: `Local`, `MGRS`, `LocalCartesianUTM`, `LocalCartesian`, or `TransverseMercator`. The value is case-sensitive. |
| `vertical_datum` | The height reference of the map: `WGS84` (the ellipsoid) or `EGM2008` (the geoid). All types except `Local` need it.                                                 |
| `mgrs_grid`      | The MGRS grid of the map, for example `54SUE`. Only for `MGRS`.                                                                                                      |
| `map_origin`     | The latitude and longitude of the map origin. For `LocalCartesianUTM`, `LocalCartesian`, and `TransverseMercator`.                                                   |
| `scale_factor`   | The scale factor of `TransverseMercator` (0.9996 by default). The loader sets a fixed value for the other types.                                                     |

!!! note

    With `Local`, the map has no geodetic reference. GNSS localization and the position in latitude and longitude of the AD API do not work.

    With `MGRS`, the X and Y coordinates of each point are its position in the 100 km square of the MGRS grid, and the Z coordinate is its height. The map must stay in one MGRS grid square.

    If the file does not exist, `map_projection_loader` uses `MGRS` and reads the grid from the Lanelet2 map. This fallback is deprecated.

For more information, see [the README of `autoware_map_projection_loader`](https://github.com/autowarefoundation/autoware_core/tree/main/map/autoware_map_projection_loader/README.md).
