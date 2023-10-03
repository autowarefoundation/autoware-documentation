# Converting UTM maps to MGRS map format

## Overview

If you want to use MGRS (Military Grid Reference System) format in Autoware,
you need to convert UTM (Universal Transverse Mercator) map to MGRS format.
In order to do that, we will use [UTM to MGRS pointcloud converter](https://github.com/leo-drive/pc_utm_to_mgrs_converter) ROS 2 package provided by Leo Drive.

## Installation

### Dependencies

- ROS 2
- PCL-conversions
- [GeographicLib](https://geographiclib.sourceforge.io/C++/doc/install.html)

To install dependencies:

```bash
sudo apt install ros-humble-pcl-conversions \
       geographiclib-tools
```

### Building

```bash
    cd <PATH-TO-YOUR-ROS-2-WORKSPACE>/src
    git clone https://github.com/leo-drive/pc_utm_to_mgrs_converter.git
    cd ..
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Usage

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
(i.e., [UTM converter](https://www.latlong.net/lat-long-utm.html))
