# Creating maps

Autoware requires a pointcloud map and a vector map for the vehicle's operating environment. (Check the [map design documentation page](../../../design/autoware-architecture/map/index.md) for the detailed specification).

This page explains how users can create maps that can be used for Autoware.

## Creating a point cloud map

Traditionally, a Mobile Mapping System (MMS) is used in order to create highly accurate large-scale point cloud maps. However, since a MMS requires high-end sensors for precise positioning, its operational cost can be very expensive and may not be suitable for a relatively small driving environment. Alternatively, a Simultaneous Localization And Mapping (SLAM) algorithm can be used to create a point cloud map from recorded LiDAR scans. Some of the useful open-source SLAM implementations are listed in this [page](open-source-slam/index.md).

If you prefer proprietary software that is easy to use, you can try a fully automatic mapping tool from [MAP IV, Inc.](https://www.map4.jp/), [_MapIV Engine_](https://www.map4.jp/solutions/mapping_localization/map-%e2%85%b3-engine/). They currently provide a trial license for Autoware users free of charge.

## Creating a vector map

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
    The open-source [AutoStreamForAutoware tool](https://github.com/tomtom-international/AutoStreamForAutoware) can be used to convert an AutoStream map to a Lanelet2 map.
    However, the converter is still in its early stages and has some [known limitations](https://github.com/tomtom-international/AutoStreamForAutoware/blob/main/docs/known-issues.md).
