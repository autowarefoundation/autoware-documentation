# How to create maps for Autoware 

Autoware relies on high-definition maps (HD maps), both point cloud maps and vector maps, to perform various tasks such as localization, route planning, traffic light detection, and predicting the trajectories of other vehicles and pedestrians. In order to use these functionalities in Autoware, compatible HD maps of the driving environment need to be created.
To create point cloud and vector maps you can use various software, both open-source and proprietary. However, you need to ensure that the created maps are compatible with Autoware. Specifications of point cloud and vector maps that Autoware requires are given below, along with examples of software you can use to create them.

### Creating a point cloud map

A 3D point cloud map is primarily used for LiDAR-based localization in Autoware. In order to determine the current position and orientation of the vehicle, a live scan captured from one or more LiDAR units is matched against a pre-generated 3D point cloud map. Therefore, an accurate point cloud map is crucial for good localization results. 

The specifications of an Autoware-compatible point cloud map are as follows:

- A map must cover the entire operational area of the vehicle and should include an additional buffer zone of at least 200 m in all directions.
- A map is saved using [PCD (Point Cloud Data) file format](https://pointclouds.org/documentation/tutorials/pcd_file_format.html).
- A map can be saved as a single PCD file or divided into multiple PCD files.
- Each point in a map must contain X, Y, and Z coordinates.
- An intensity or RGB value of each point can be included in the PCD file but is optional.
- A map must be smaller than 1 GB, [as per the current ROS message size limit](https://github.com/ros/ros_comm/issues/902). 
- A map resolution should be at least 0.2 m to yield reliable localization results.
- A map can be in either local or global coordinates, but must be in global coordinates (georeferenced) to use GNSS data for localization.
- [Military Grid Reference System (MGRS)](https://en.wikipedia.org/wiki/Military_Grid_Reference_System) is used as a coordinate system for a georeferenced map. In a georeferenced map, the X and Y coordinates of each point represent the point's location within a 100 km MGRS grid, while the Z coordinate represents the point's elevation.

Traditionally, a Mobile Mapping System (MMS) is used in order to create highly accurate large-scale point cloud maps. However, since a MMS requires high-end sensors for precise positioning, its operational cost can be very expensive and may not be suitable for a relatively small driving environment. Alternatively, a Simultaneous Localization And Mapping (SLAM) algorithm can be used to create a point cloud map from recorded LiDAR scans. Commonly used open-source SLAM implementations include [hdl-graph-slam](https://github.com/koide3/hdl_graph_slam) (LiDAR, IMU\*, GNSS\*), [LeGO-LOAM](https://github.com/facontidavide/LeGO-LOAM-BOR) (LiDAR, IMU\*), [LeGO-LOAM-BOR](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) (LiDAR), [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) (LiDAR, IMU, GNSS), and [lidarslam-ros2](https://github.com/rsasaki0109/lidarslam_ros2) (LiDAR, IMU\*). The required sensor data for each algorithm is specified in the parentheses, where an asterisk (\*) indicates that such sensor data is optional. Most of these algorithms already have a built-in loop-closure and pose graph optimization. However, if the built-in, automatic loop-closure fails or does not work correctly, you can use [Interactive SLAM](https://github.com/SMRT-AIST/interactive_slam) to adjust and optimize a pose graph manually. For supported LiDAR models, please check the Github repository of each algorithm.

It is important to note that apart from [lidarslam-ros2](https://github.com/rsasaki0109/lidarslam_ros2), these open-source SLAM implementations are based on ROS 1. Therefore, it could be problematic if you want to install them on the same machine that runs Autoware, which is based on ROS 2. To avoid this problem, you can use Docker or install them on a different machine. Another problem is the ROSBAG version, as many of these SLAM implementations require ROSBAG 1 instead of ROSBAG 2 used by Autoware. For the ROSBAG version problem, you may use this [stand-alone converter](https://gitlab.com/MapIV/rosbags) to convert a ROSBAG 2 file to a ROSBAG 1 file and vice versa.

If you prefer proprietary software that is easy to use, you can try a fully automatic mapping tool from [MAP IV, Inc.](https://www.map4.jp/), [_MapIV Engine_](https://www.map4.jp/map4_engine_en/). They currently provide a trial license for Autoware users free of charge.

### Creating a vector map

Another type of map used in Autoware is a vector map. A vector map or road network map is required for route planning, traffic light detection, and other traffic participants' trajectory prediction. Specifications of an Autoware-compatible road network map are as follows:

- A map must cover the entire operational area of the vehicle and should include an additional buffer zone of at least 200 m in all directions.
- A map is in [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) format with[additional modifications](https://github.com/autowarefoundation/autoware.universe/blob/main/map/lanelet2_extension/docs/lanelet2_format_extension.md).
- A map must contain the shape and position information of lanes, traffic lights, stop lines, crosswalks, parking spaces, and parking lots.
- Each lanelet must contain information regarding its right of way, speed limit, traffic direction, and associated traffic lights, stop lines, and traffic signs.
- Unless it is the end/beginning of a road, each lanelet must be correctly connected to its predecessor, successors, left neighbor, and right neighbor for routing planning to be working.

Technically, you can use open-source software such as [JOSM](https://josm.openstreetmap.de/) to create Lanelet2 primitives, including point and linestring, and save the map as an OSM file. However, various modifications, e.g., adding mandatory tags and relations, must be done manually to make the map compatible with Autoware. This process can be very tedious and time-consuming. Another open-source option worth mentioning is [MapToolbox](https://github.com/autocore-ai/MapToolbox), a plugin for [Unity](https://unity.com/) designed to create a Lanelet2 map for Autoware.

However, the easiest way to create a complete road network map that is compatible with Autoware is to use [Vector Map Builder](https://tools.tier4.jp/vector_map_builder/), a web-based tool provided by [TIER IV, Inc.](https://www.tier4.jp/) Vector Map Builder allows you to directly create lanes and add additional regulatory elements such as stop signs or traffic lights in the correct format required by Autoware while using a point cloud map as a reference.

## Autoware-compatible map providers

Apart from creating HD maps yourself, you can also use a mapping service from the following Autoware-compatible map providers:

- [MAP IV, Inc.](https://www.map4.jp/)
- [AISAN TECHNOLOGY CO., LTD.](https://www.aisantec.co.jp/)
- [TomTom](https://www.tomtom.com/)

The table below shows each company's mapping technology and the types of HD maps they support.

| **Company**                                               | **Mapping technology** | **Available maps**          |
| --------------------------------------------------------- | ---------------------- | --------------------------- |
| [MAP IV, Inc.](https://www.map4.jp/)                      | SLAM                   | Point cloud and Vector maps |
| [AISAN TECHNOLOGY CO., LTD.](https://www.aisantec.co.jp/) | MMS                    | Point cloud and Vector maps |
| [TomTom](https://www.tomtom.com/)                         | MMS                    | Vector map\*                |

\*TomTom provides a road network map in their AutoStream format, not Lanelet2. However, you can use this open-source [converter](https://github.com/tomtom-international/AutoStreamForAutoware) to convert it to a suitable format for Autoware. Note that the converter is still in a very early stage and may not be able to convert all available information to Lanelet2 format.
