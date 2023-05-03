# Map component design

## 1. Requirements

!!! warning

    Under Construction

## 2. Architecture

!!! warning

    Under Construction

## 3. Features

!!! warning

    Under Construction

## 4. Interface and Data Structure

### Pointcloud map

The point cloud map must be supplied as a .pcd file, regardless of whether it is a single file or split into multiple files.

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

### Vector map

!!! warning

    Under Construction
