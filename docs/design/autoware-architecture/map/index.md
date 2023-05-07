# Map component design

## 1. Overview
Autoware relies on high-definition point cloud maps and vector maps of the driving environment to perform various tasks such as localization, route planning, traffic light detection, and predicting the trajectories of pedestrians and other vehicles.

This document describes the design of map component of Autoware, including its requirements, architecture design, features, data formats, and interface to distribute map information to the rest of autonomous driving stack.

## 2. Requirements

Map should provide two types of information to the rest of the stack:
* Semantic information about roads as a vector map
* Geometric information about the environment as a point cloud map (optional)

A vector map contains highly accurate information about a road network, lane geometry, and traffic lights. It is required for route planning, traffic light detection, and predicting the trajectories of other vehicles and pedestrians.

A 3D point cloud map is primarily used for LiDAR-based localization and part of perception in Autoware. In order to determine the current position and orientation of the vehicle, a live scan captured from one or more LiDAR units is matched against a pre-generated 3D point cloud map. Therefore, an accurate point cloud map is crucial for good localization results. However, if the vehicle has an alternate localization method with enough accuracy, for example using camera-based localization, point cloud map may not be required to use Autoware.

## 3. Architecture

!!! warning

    Under Construction

## 4. Features

!!! warning

    Under Construction

## 5. Map Specification

### Point Cloud Map
The point cloud map must be supplied as a file with the following requirements:
- It must be in the [PCD (Point Cloud Data) file format](https://pointclouds.org/documentation/tutorials/pcd_file_format.html), but can be a single PCD file or divided into multiple PCD files.
- Each point in the map must contain X, Y, and Z coordinates.
- An intensity or RGB value for each point may be optionally included.
- It must cover the entire operational area of the vehicle. It is also recommended to include an additional buffer zone according to the detection range of sensors attached to the vehicle.
- Its resolution should be at least 0.2 m to yield reliable localization results.
- It can be in either local or global coordinates, but must be in global coordinates (georeferenced) to use GNSS data for localization.

!!! note

    Three global coordinate systems are currently supported by Autoware, including [Military Grid Reference System (MGRS)](https://en.wikipedia.org/wiki/Military_Grid_Reference_System), [Universal Transverse Mercator (UTM)](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system), and [Japan Rectangular Coordinate System](https://ja.wikipedia.org/wiki/%E5%B9%B3%E9%9D%A2%E7%9B%B4%E8%A7%92%E5%BA%A7%E6%A8%99%E7%B3%BB).
    However, MGRS is a preferred coordinate system for georeferenced maps.
    In a map with MGRS coordinate system, the X and Y coordinates of each point represent the point's location within the 100,000-meter square, while the Z coordinate represents the point's elevation.

If it is split into a single file, Autoware assumes the following directory structure by default.

```bash
sample-map-rosbag
├── lanelet2_map.osm
├── pointcloud_map.pcd
```

If it is split into multiple files, Autoware assumes the following directory structure by default.

```bash
sample-map-rosbag
├── lanelet2_map.osm
├── pointcloud_map
├── pcd_00.pcd
├── pcd_01.pcd
├── pcd_02.pcd
├── ...
└── pointcloud_map_metadata.yaml
```

Note that, if you split the map into multiple files, you must meet the following additional conditions:

- It must be split by lines parallel to the x-y axis.
- Additional metadata must be provided as well.

Metadata should look like as follows:

```yaml
x_resolution: 100.0
y_resolution: 150.0
A.pcd: [1200, 2500] # -> 1200 < x < 1300, 2500 < y < 2650
B.pcd: [1300, 2500] # -> 1300 < x < 1400, 2500 < y < 2650
C.pcd: [1200, 2650] # -> 1200 < x < 1300, 2650 < y < 2800
D.pcd: [1400, 2650] # -> 1400 < x < 1500, 2650 < y < 2800
```

TODO(yukkysaito): move to [here](../../how-to-guides/creating-maps-for-autoware/index.md)?
You may use [pointcloud_divider](https://github.com/MapIV/pointcloud_divider) from MAP IV for dividing pointcloud map as well as generating the compatible metadata.yaml.

#### Vector Map
The vector cloud map must be supplied as a file with the following requirements:
- It must be in [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) format, with [additional modifications required by Autoware](https://github.com/autowarefoundation/autoware_common/blob/main/tmp/lanelet2_extension/docs/lanelet2_format_extension.md).
- It must contain the shape and position information of lanes, traffic lights, stop lines, crosswalks, parking spaces, and parking lots.
- Except at the beginning or end of a road, each lanelet in the map must be correctly connected to its predecessor, successors, left neighbor, and right neighbor.
- Each lanelet in the map must contain traffic rule information including its speed limit, right of way, traffic direction, associated traffic lights, stop lines, and traffic signs.
- It must cover the entire operational area of the vehicle.

!!! warning

    Under Construction
