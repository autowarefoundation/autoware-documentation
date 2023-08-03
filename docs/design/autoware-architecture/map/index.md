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

- It must be in the [PCD (Point Cloud Data) file format](https://pointclouds.org/documentation/tutorials/pcd_file_format.html), but can be a single PCD file or divided into multiple PCD files.
- Each point in the map must contain X, Y, and Z coordinates.
- An intensity or RGB value for each point may be optionally included.
- It must cover the entire operational area of the vehicle. It is also recommended to include an additional buffer zone according to the detection range of sensors attached to the vehicle.
- Its resolution should be at least 0.2 m to yield reliable localization results.
- It can be in either local or global coordinates, but must be in global coordinates (georeferenced) to use GNSS data for localization.

For more details on divided map format, please refer to [the Readme of `map_loader` in Autoware Universe](https://github.com/autowarefoundation/autoware.universe/blob/main/map/map_loader/README.md).

!!! note

    Three global coordinate systems are currently supported by Autoware, including [Military Grid Reference System (MGRS)](https://en.wikipedia.org/wiki/Military_Grid_Reference_System), [Universal Transverse Mercator (UTM)](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system), and [Japan Rectangular Coordinate System](https://ja.wikipedia.org/wiki/%E5%B9%B3%E9%9D%A2%E7%9B%B4%E8%A7%92%E5%BA%A7%E6%A8%99%E7%B3%BB).
    However, MGRS is a preferred coordinate system for georeferenced maps.
    In a map with MGRS coordinate system, the X and Y coordinates of each point represent the point's location within the 100,000-meter square, while the Z coordinate represents the point's elevation.

### Vector Map

The vector cloud map must be supplied as a file with the following requirements:

- It must be in [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) format, with [additional modifications required by Autoware](https://github.com/autowarefoundation/autoware_common/blob/main/tmp/lanelet2_extension/docs/lanelet2_format_extension.md).
- It must contain the shape and position information of lanes, traffic lights, stop lines, crosswalks, parking spaces, and parking lots.
- Except at the beginning or end of a road, each lanelet in the map must be correctly connected to its predecessor, successors, left neighbor, and right neighbor.
- Each lanelet in the map must contain traffic rule information including its speed limit, right of way, traffic direction, associated traffic lights, stop lines, and traffic signs.
- It must cover the entire operational area of the vehicle.

!!! warning

    Under Construction
