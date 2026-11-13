<!-- cspell: ignore northup -->

# Converting UTM maps to MGRS map format

## Overview

If you want to use MGRS (Military Grid Reference System) format in Autoware,
you need to convert UTM (Universal Transverse Mercator) map to MGRS format.
In order to do that, we will use [UTM to MGRS pointcloud converter](https://github.com/leo-drive/pc_utm_to_mgrs_converter) ROS 2 package provided by Leo Drive.

!!! tip

    MGRS is not the only projection that Autoware supports. The `map_projector_info.yaml` file in your map directory can also select `Local`, `LocalCartesianUTM`, `LocalCartesian`, or `TransverseMercator`. Convert the map only if you want to use MGRS. For the fields of each projection, see the [projection information of the map component design](../../../../design/autoware-architecture-v1/components/map/index.md#projection-information).

## Installation

### Dependencies

- ROS 2
- PCL-conversions
- [GeographicLib](https://geographiclib.sourceforge.io/C++/doc/install.html)

To install dependencies:

```bash
sudo apt install ros-$ROS_DISTRO-pcl-conversions \
       geographiclib-tools
```

The converter changes the ellipsoid height of each point to an orthometric height with the EGM2008 geoid. Download the geoid data set:

```bash
sudo geographiclib-get-geoids egm2008-1
```

If the geoid data set is not installed, the converter prints `Failed to convert Height from Ellipsoid to Orthometric` and sets the height of the point to 0.

### Building

```bash
    cd <PATH-TO-YOUR-ROS-2-WORKSPACE>/src
    git clone https://github.com/leo-drive/pc_utm_to_mgrs_converter.git
    cd ..
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Usage

!!! warning

    The converter uses UTM zone 35 in the northern hemisphere. The value is hard-coded in `src/pc_utm_to_mgrs_converter.cpp` (`gnss_stat_utm.zone = 35;` and `gnss_stat_utm.northup = true;`). If your map is in a different UTM zone or in the southern hemisphere, change these two lines and build the package again before you convert the map.

After the installation of converter tool,
we need to define northing,
easting and ellipsoid height of local UTM map origin in `pc_utm_to_mgrs_converter.param.yaml`.
For example, you can use latitude,
longitude and altitude values in the navsatfix message from your GNSS/INS sensor.

??? note "Sample ROS 2 topic echo from navsatfix message"

    ```sh
    header:
      stamp:
        sec: 1694612439
        nanosec: 400000000
      frame_id: GNSS_INS/gnss_ins_link
    status:
      status: 0
      service: 1
    latitude: 41.0216110801253
    longitude: 28.887096461148346
    altitude: 74.28264078891529
    position_covariance:
    - 0.0014575386885553598
    - 0.0
    - 0.0
    - 0.0
    - 0.004014162812381983
    - 0.0
    - 0.0
    - 0.0
    - 0.0039727711118757725
    position_covariance_type: 2
    ```

After that, you need to convert latitude and longitude values to northing and easting values.
You can use any converter on the internet for converting latitude longitude values to UTM.
(for example, [UTMconverter](https://www.latlong.net/lat-long-utm.html))

Now, we are ready to update `pc_utm_to_mgrs_converter.param.yaml`,
example for our navsatfix message:

```diff
/**:
  ros__parameters:
      # Northing of local origin
-     Northing: 4520550.0
+     Northing: 4542871.33

      # Easting of local origin
-     Easting: 698891.0
+     Easting: 658659.84

      # Elipsoid Height of local origin
-     ElipsoidHeight: 47.62
+     ElipsoidHeight: 74.28
```

Lastly, we will update input and pointcloud the map path in `pc_utm_to_mgrs_converter.launch.xml`:

```diff
...
- <arg name="input_file_path" default="/home/melike/projects/autoware_data/gebze_pospac_map/pointcloud_map.pcd"/>
+ <arg name="input_file_path" default="<PATH-TO-YOUR-INPUT-PCD-MAP>"/>
- <arg name="output_file_path" default="/home/melike/projects/autoware_data/gebze_pospac_map/pointcloud_map_mgrs_orto.pcd"/>
+ <arg name="output_file_path" default="<PATH-TO-YOUR-OUTPUT-PCD-MAP>"/>
...
```

After the setting of the package, we will launch pc_utm_to_mgrs_converter:

```bash
ros2 launch pc_utm_to_mgrs_converter pc_utm_to_mgrs_converter.launch.xml
```

The conversion process will be started,
you should see `Saved <YOUR-MAP-POINTS-SIZE> data points saved to <YOUR-OUTPUT-MAP-PATH>` message on your terminal.
MGRS format pointcloud map should be saved on your output map directory.
The converter writes the PCD file in ASCII format. The output file contains only the X, Y, and Z fields. The converter does not keep other fields, for example intensity.

### Create the projection information file

Autoware must know that the point cloud map uses MGRS. Create a `map_projector_info.yaml` file in your map directory, next to the point cloud map and the Lanelet2 map:

```yaml
projector_type: MGRS
vertical_datum: EGM2008
mgrs_grid: <YOUR-MGRS-GRID> # for example, 35TPF
```

The converter does not print the MGRS grid. To find it, convert the latitude and longitude of the map origin to MGRS with an online converter. The grid is the grid zone and the 100 km square at the start of the result, for example `35TPF`.

Use `EGM2008` as the vertical datum, because the converter writes orthometric heights. Your Lanelet2 map must use the same MGRS grid. For more information, see the [README of `autoware_map_projection_loader`](https://github.com/autowarefoundation/autoware_core/tree/main/map/autoware_map_projection_loader).
