# How to create maps for Autoware

Autoware relies on high-definition point cloud maps and vector maps of the driving environment to perform various tasks such as localization, route planning, traffic light detection, and predicting the trajectories of pedestrians and other vehicles.

The specifications for point cloud and vector maps required by Autoware are given below, along with examples of both open-source and proprietary software that you can use to create them.

## Point cloud maps

A 3D point cloud map is primarily used for LiDAR-based localization in Autoware. In order to determine the current position and orientation of the vehicle, a live scan captured from one or more LiDAR units is matched against a pre-generated 3D point cloud map. Therefore, an accurate point cloud map is crucial for good localization results.

### Point cloud map specifications

- It must cover the entire operational area of the vehicle and should include an additional buffer zone of at least 200 m in all directions.
- It must be saved using the [PCD (Point Cloud Data) file format](https://pointclouds.org/documentation/tutorials/pcd_file_format.html), but can be a single PCD file or divided into multiple PCD files.
- Each point in the map must contain X, Y, and Z coordinates.
- An intensity or RGB value for each point may be optionally included.
- Its file size must be smaller than 1 GB, [as per the current ROS message size limit](https://github.com/ros/ros_comm/issues/902).
- Its resolution should be at least 0.2 m to yield reliable localization results.
- It can be in either local or global coordinates, but must be in global coordinates (georeferenced) to use GNSS data for localization.

!!! note

    Three global coordinate systems are supported by Autoware, including [Military Grid Reference System (MGRS)](https://en.wikipedia.org/wiki/Military_Grid_Reference_System), [Universal Transverse Mercator (UTM)](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system), and [Japan Rectangular Coordinate System](https://ja.wikipedia.org/wiki/%E5%B9%B3%E9%9D%A2%E7%9B%B4%E8%A7%92%E5%BA%A7%E6%A8%99%E7%B3%BB).
    However, MGRS is a preferred coordinate system for georeferenced maps.
    In a map with MGRS coordinate system, the X and Y coordinates of each point represent the point's location within the 100,000-meter square, while the Z coordinate represents the point's elevation.

### Creating a point cloud map

Traditionally, a Mobile Mapping System (MMS) is used in order to create highly accurate large-scale point cloud maps. However, since a MMS requires high-end sensors for precise positioning, its operational cost can be very expensive and may not be suitable for a relatively small driving environment. Alternatively, a Simultaneous Localization And Mapping (SLAM) algorithm can be used to create a point cloud map from recorded LiDAR scans.

Commonly used open-source SLAM implementations are [lidarslam-ros2](https://github.com/rsasaki0109/lidarslam_ros2) (LiDAR, IMU\*) and [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/ros2) (LiDAR, IMU, GNSS). The required sensor data for each algorithm is specified in the parentheses, where an asterisk (\*) indicates that such sensor data is optional. For supported LiDAR models, please check the Github repository of each algorithm. While these ROS 2-based SLAM implementations can be easily installed and used directly on the same machine that runs Autoware, it is important to note that they may not be as well-tested or as mature as ROS 1-based alternatives.

The notable open-source SLAM implementations that are based on ROS 1 include [hdl-graph-slam](https://github.com/koide3/hdl_graph_slam) (LiDAR, IMU\*, GNSS\*), [LeGO-LOAM](https://github.com/facontidavide/LeGO-LOAM-BOR) (LiDAR, IMU\*), [LeGO-LOAM-BOR](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) (LiDAR), and [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) (LiDAR, IMU, GNSS).
Most of these algorithms already have a built-in loop-closure and pose graph optimization. However, if the built-in, automatic loop-closure fails or does not work correctly, you can use [Interactive SLAM](https://github.com/SMRT-AIST/interactive_slam) to adjust and optimize a pose graph manually.

Since Autoware is based on ROS 2, it could be problematic if you want to install ROS 1-based SLAM implementations on the same machine. To avoid this problem, you can use [Docker](https://www.docker.com/) or simply install them on a different machine. Another problem is the ROSBAG version; these SLAM implementations require ROSBAG 1 instead of ROSBAG 2 used by Autoware. For the ROSBAG version problem, you may use this [stand-alone converter](https://gitlab.com/MapIV/rosbags) to convert a ROSBAG 2 file to a ROSBAG 1 file and vice versa.

If you prefer proprietary software that is easy to use, you can try a fully automatic mapping tool from [MAP IV, Inc.](https://www.map4.jp/), [_MapIV Engine_](https://www.map4.jp/map4_engine_en). They currently provide a trial license for Autoware users free of charge.

## Vector maps

A vector map contains highly accurate information about a road network, lane geometry, and traffic lights. It is required for route planning, traffic light detection, and predicting the trajectories of other vehicles and pedestrians.

### Vector map specifications

- It must cover the entire operational area of the vehicle and should include an additional buffer zone of at least 200 m in all directions.
- It must be in [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) format, with [additional modifications required by Autoware](https://github.com/autowarefoundation/autoware.universe/blob/main/map/lanelet2_extension/docs/lanelet2_format_extension.md).
- It must contain the shape and position information of lanes, traffic lights, stop lines, crosswalks, parking spaces, and parking lots.
- Each lanelet in the map must contain information regarding its right of way, speed limit, traffic direction, associated traffic lights, stop lines, and traffic signs.
- Except at the beginning or end of a road, each lanelet in the map must be correctly connected to its predecessor, successors, left neighbor, and right neighbor.

### Creating a vector map

The easiest way to create an Autoware-compatible vector map is to use [Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/), a free web-based tool provided by [TIER IV, Inc.](https://www.tier4.jp/).
Vector Map Builder allows you to create lanes and add additional regulatory elements such as stop signs or traffic lights using a point cloud map as a reference.

For open-source software options, [MapToolbox](https://github.com/autocore-ai/MapToolbox) is a plugin for [Unity](https://unity.com/) specifically designed to create Lanelet2 maps for Autoware.
Although [JOSM](https://josm.openstreetmap.de/) is another open-source tool that can be used to create Lanelet2 maps, be aware that a number of modifications must be done manually to make the map compatible with Autoware. This process can be tedious and time-consuming, so the use of JOSM is not recommended.

## Autoware-compatible map providers

If it is not possible to create HD maps yourself, you can use a mapping service from the following Autoware-compatible map providers instead:

- [MAP IV, Inc.](https://www.map4.jp/)
- [AISAN TECHNOLOGY CO., LTD.](https://www.aisantec.co.jp/)
- [TomTom](https://www.tomtom.com/)

The table below shows each company's mapping technology and the types of HD maps they support.

| **Company**                                               | **Mapping technology** | **Available maps**          |
| --------------------------------------------------------- | ---------------------- | --------------------------- |
| [MAP IV, Inc.](https://www.map4.jp/)                      | SLAM                   | Point cloud and vector maps |
| [AISAN TECHNOLOGY CO., LTD.](https://www.aisantec.co.jp/) | MMS                    | Point cloud and vector maps |
| [TomTom](https://www.tomtom.com/)                         | MMS                    | Vector map\*                |

!!! note

    Maps provided by TomTom use their proprietary AutoStream format, not Lanelet2.
    The open-source [AutostreamForAutoware tool](https://github.com/tomtom-international/AutoStreamForAutoware) can be used to convert an AutoStream map to a Lanelet2 map.
    However, the converter is still in its early stages and has some [known limitations](https://github.com/tomtom-international/AutoStreamForAutoware/blob/main/docs/known-issues.md).
