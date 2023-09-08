# Ground-Lidar calibration

## Overview

[Ground-Lidar Calibration](https://github.com/tier4/CalibrationTools/blob/tier4/universe/sensor/docs/how_to_extrinsic_ground_plane.md) method operates under the assumption
that the area surrounding the vehicle can be represented as a flat surface.
So, you must find as wide and flat a surface as possible for ROS 2 bag recording.
The method then modifies the calibration transformation
in a way that aligns the points
corresponding to the ground within the point cloud with the XY plane of the base_link.
This means that only the z, roll, and pitch values of the tf undergo calibration,
while the remaining x, y, and yaw values must be calibrated using other methods,
such as [manual adjustment](./generic-calibration.md) and [mapping-based lidar-lidar calibration](./lidar-lidar-calibration.md).

You need to apply this calibration method to each lidar separately,
so our bag should contain all lidars to be calibrated.

??? note "ROS 2 Bag example of our ground-based calibration process for tutorial_vehicle"

    ```sh

    Files:             rosbag2_2023_09_05-11_23_50_0.db3
    Bag size:          3.8 GiB
    Storage id:        sqlite3
    Duration:          112.702s
    Start:             Sep  5 2023 11:23:51.105 (1693902231.105)
    End:               Sep  5 2023 11:25:43.808 (1693902343.808)
    Messages:          2256
    Topic information: Topic: /sensing/lidar/front/pointcloud_raw | Type: sensor_msgs/msg/PointCloud2 | Count: 1128 | Serialization Format: cdr
                       Topic: /sensing/lidar/top/pointcloud_raw | Type: sensor_msgs/msg/PointCloud2 | Count: 1128 | Serialization Format: cdr
    ```

## Ground-lidar calibration

We will start with creating launch file four our own vehicle like the previous sections process:

```bash
cd <YOUR-OWN-AUTOWARE-DIRECTORY>/src/autoware/calibration_tools/sensor
cd extrinsic_calibration_manager/launch
cd <YOUR-OWN-SENSOR-KIT-NAME> # i.e. for our guide, it will ve cd tutorial_vehicle_sensor_kit
touch ground_plane.launch.xml ground_plane_sensor_kit.launch.xml
```

The created `ground_plane.launch.xml` and `ground_plane_sensor_kit.launch.xml` are version of sample sensor kit
[aip_x1](https://github.com/tier4/CalibrationTools/tree/tier4/universe/sensor/extrinsic_calibration_manager/launch/aip_x1) provided from Tier IV.

Then we will continue with adding vehicle_id and sensor model names to the `ground_plane.launch.xml`.
(Optionally, values are not important. These parameters Overrode from launch argument)

```diff
+ <?xml version="1.0" encoding="UTF-8"?>
+ <launch>
+   <arg name="vehicle_id" default="<YOUR_VEHICLE_ID>"/>
+
+   <let name="sensor_model" value="<YOUR_SENSOR_KIT_NAME>"/>
```

??? note "i.e. vehicle_id and sensor_model definition on tutorial_vehicle (ground_plane.launch.xml)"

    ```xml
    + <?xml version="1.0" encoding="UTF-8"?>
    + <launch>
    +   <arg name="vehicle_id" default="tutorial_vehicle"/>
    +
    +   <let name="sensor_model" value="tutorial_vehicle_sensor_kit"/>
    ```

After that, we will launch our sensor_kit for mapping-based lidar-lidar calibration,
so we must add these lines on manual.launch.xml:

```diff
+   <group>
+     <push-ros-namespace namespace="sensor_kit"/>
+     <include file="$(find-pkg-share extrinsic_calibration_manager)/launch/$(var sensor_model)/ground_plane_sensor_kit.launch.xml">
+       <arg name="vehicle_id" value="$(var vehicle_id)"/>
+     </include>
+   </group>
+ </launch>
```

The final version of the file (mapping_based.launch.xml) for tutorial_vehicle should be like this:

??? note "Sample mapping_based.launch.xml file for tutorial vehicle"

    ```xml
    <launch>
      <arg name="vehicle_id" default="tutorial_vehicle"/>
      <let name="sensor_model" value="tutorial_vehicle_sensor_kit"/>

      <group>
        <push-ros-namespace namespace="sensor_kit"/>
        <include file="$(find-pkg-share extrinsic_calibration_manager)/launch/$(var sensor_model)/ground_plane_sensor_kit.launch.xml">
          <arg name="vehicle_id" value="$(var vehicle_id)"/>
        </include>
      </group>
    </launch>

    ```
