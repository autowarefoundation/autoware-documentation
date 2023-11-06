# Map component design

## 1. Overview

Autoware relies on high-definition point cloud maps and vector maps of the driving environment to perform various tasks such as localization, route planning, traffic light detection, and predicting the trajectories of pedestrians and other vehicles.

This document describes the design of map component of Autoware, including its requirements, architecture design, features, data formats, and interface to distribute map information to the rest of autonomous driving stack.

## 2. Requirements

Map should provide two types of information to the rest of the stack:

- Semantic information about roads as a vector map
- Geometric information about the environment as a point cloud map (optional)

A vector map contains highly accurate information about a road network, lane geometry, and traffic lights. It is required for route planning, traffic light detection, and predicting the trajectories of other vehicles and pedestrians.

A 3D point cloud map is primarily used for LiDAR-based localization and part of perception in Autoware. In order to determine the current position and orientation of the vehicle, a live scan captured from one or more LiDAR units is matched against a pre-generated 3D point cloud map. Therefore, an accurate point cloud map is crucial for good localization results. However, if the vehicle has an alternate localization method with enough accuracy, for example using camera-based localization, point cloud map may not be required to use Autoware.

In addition to above two types of maps, Autoware also requires a supplemental file for specifying the coordinate system of the map in geodetic system.

## 3. Architecture

This diagram describes the high-level architecture of Map component in Autoware.

![map component architecture](image/high-level-map-diagram.drawio.svg){width="800"}

The Map component consists of the following sub-components:

- **Point Cloud Map Loading**: Load and publish point cloud map
- **Vector Map Loading**: Load and publish vector map
- **Projection Loading**: Load and publish projection information for conversion between local coordinate (x, y, z) and geodetic coordinate (latitude, longitude, altitude)

## 4. Component interface

### Input to the map component

- **From file system**
  - Point cloud map and its metadata file
  - Vector map
  - Projection information

### Output from the map component

- **To Sensing**
  - Projection information: Used to convert GNSS data from geodetic coordinate system to local coordinate system
- **To Localization**
  - Point cloud map: Used for LiDAR-based localization
  - Vector map: Used for localization methods based on road markings, etc
- **To Perception**
  - Point cloud map: Used for obstacle segmentation by comparing LiDAR and point cloud map
  - Vector map: Used for vehicle trajectory prediction
- **To Planning**
  - Vector map: Used for behavior planning
- **To API layer**
  - Projection information: Used to convert localization results from local coordinate system to geodetic coordinate system

## 5. Map Specification

### Point Cloud Map

The point cloud map must be supplied as a file with the following requirements:

#### Basic requirements

1. The point cloud map must be projected on the same coordinate defined in `map_projection_loader` in order to be consistent with the lanelet2 map and other packages that converts between local and geodetic coordinates. For more information, please refer to [the readme of `map_projection_loader`](https://github.com/autowarefoundation/autoware.universe/tree/main/map/map_projection_loader/README.md).
2. It must be in the [PCD (Point Cloud Data) file format](https://pointclouds.org/documentation/tutorials/pcd_file_format.html), but can be a single PCD file or divided into multiple PCD files.
3. Each point in the map must contain X, Y, and Z coordinates.
4. An intensity or RGB value for each point may be optionally included.
5. It must cover the entire operational area of the vehicle. It is also recommended to include an additional buffer zone according to the detection range of sensors attached to the vehicle.
6. Its resolution should be at least 0.2 m to yield reliable localization results.
7. It can be in either local or global coordinates, but must be in global coordinates (georeferenced) to use GNSS data for localization.

#### Additional requirements for divided map

Furthermore, when using divided map, the following requirements must be met so that the dynamic map loading features work properly:

8. It must be divided by straight lines parallel to the x-axis and y-axis. The system does not support division by diagonal lines or curved lines.
9. The division size along each axis should be equal.
10. The division size should be about 20m x 20m. Particularly, care should be taken as using too large division size (for example, more than 100m) may have adverse effects on dynamic map loading features.
11. All the split maps should not overlap with each other.
12. Metadata file should also be provided e.g. as `pointcloud_map_metadata.yaml`.

The metadata should look like this:

```yaml
x_resolution: 20.0
y_resolution: 20.0
A.pcd: [1200, 2500] # -> 1200 < x < 1220, 2500 < y < 2520
B.pcd: [1220, 2500] # -> 1220 < x < 1240, 2500 < y < 2520
C.pcd: [1200, 2520] # -> 1200 < x < 1220, 2520 < y < 2540
D.pcd: [1240, 2520] # -> 1240 < x < 1260, 2520 < y < 2540
```

where,

- x_resolution and y_resolution
- A.pcd, B.pcd, etc, are the names of PCD files.
- List such as [1200, 2500] are the values indicate that for this PCD file, x coordinates are between 1200 and 1220 (x_resolution + x_coordinate) and y coordinates are between 2500 and 2520 (y_resolution + y_coordinate).
  You may use [pointcloud_divider](https://github.com/MapIV/pointcloud_divider) from MAP IV for dividing pointcloud map as well as generating the compatible metadata.yaml.

For more details on divided map format, please refer to [the readme of `map_loader` in Autoware Universe](https://github.com/autowarefoundation/autoware.universe/blob/main/map/map_loader/README.md).

### Vector Map

The vector cloud map must be supplied as a file with the following requirements:

1. It must be in [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) format, with [additional modifications required by Autoware](https://github.com/autowarefoundation/autoware_common/blob/main/tmp/lanelet2_extension/docs/lanelet2_format_extension.md).
2. It must contain the shape and position information of lanes, traffic lights, stop lines, crosswalks, parking spaces, and parking lots.
3. The position information should be expressed in correct global coordinate in latitude / longitude unless you are using local coordinate. Please see the next section for projection information.
4. Except at the beginning or end of a road, each lanelet in the map must be correctly connected to its predecessor, successors, left neighbor, and right neighbor.
5. Each lanelet in the map must contain traffic rule information including its speed limit, right of way, traffic direction, associated traffic lights, stop lines, and traffic signs.
6. It must cover the entire operational area of the vehicle.

!!! warning

    Under Construction

### Projection Information

The projection information must be supplied as a file with the following requirements:

1. It must be in YAML format, provided into `map_projection_loader` in current Autoware Universe implementation.
2. The file must contain the following information:

- The name of the projection method used to convert between local and global coordinates
- The parameters of the projection method (depending on the projection method)

For further information, please refer to [the readme of `map_projection_loader` in Autoware Universe](https://github.com/autowarefoundation/autoware.universe/tree/main/map/map_projection_loader/README.md).

!!! note

    Three global coordinate systems are currently supported by Autoware, including [Military Grid Reference System (MGRS)](https://en.wikipedia.org/wiki/Military_Grid_Reference_System), [Universal Transverse Mercator (UTM)](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system), and [Transverse Mercator](https://en.wikipedia.org/wiki/Transverse_Mercator_projection).
    However, MGRS is a preferred coordinate system for georeferenced maps.
    In a map with MGRS coordinate system, the X and Y coordinates of each point represent the point's location within the 100,000-meter square, while the Z coordinate represents the point's elevation.

    Local coordinate can also be used, but you cannot use GNSS data in this case.
