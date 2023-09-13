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
such as [manual adjustment](./generic-calibration.md) or [mapping-based lidar-lidar calibration](./lidar-lidar-calibration.md).

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
cd <YOUR-OWN-SENSOR-KIT-NAME> # i.e. for our guide, it will ve cd tutorial_vehicle_sensor_kit which is created in generic calibration
touch ground_plane.launch.xml ground_plane_sensor_kit.launch.xml
```

The created `ground_plane.launch.xml` and `ground_plane_sensor_kit.launch.xml` are version of sample sensor kit
[aip_x1](https://github.com/tier4/CalibrationTools/tree/tier4/universe/sensor/extrinsic_calibration_manager/launch/aip_x1)
provided from TIER IV.

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

After that, we will launch our sensor_kit for ground - lidar calibration,
so we must add these lines on ground_plane.launch.xml:

```diff
+   <group>
+     <push-ros-namespace namespace="sensor_kit"/>
+     <include file="$(find-pkg-share extrinsic_calibration_manager)/launch/$(var sensor_model)/ground_plane_sensor_kit.launch.xml">
+       <arg name="vehicle_id" value="$(var vehicle_id)"/>
+     </include>
+   </group>
+ </launch>
```

The final version of the file (ground_plane.launch.xml) for tutorial_vehicle should be like this:

??? note "Sample ground_plane.launch.xml file for tutorial vehicle"

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

After the completing of ground_plane.launch.xml file,
we will be ready to implement ground_plane_sensor_kit.launch.xml for the own sensor model.

Optionally, (don't forget, these parameters overrode by ROS 2 launch arguments.)
you can modify sensor_kit and vehicle_id as `ground_plane.launch.xml`over this xml snippet:
(You can change rviz_profile path after the saving rviz config as video
which included at the end of the page)

```diff
+ <?xml version="1.0" encoding="UTF-8"?>
+ <launch>
+   <arg name="vehicle_id" default="<YOUR_VEHICLE_ID>"/>
+   <let name="sensor_model" value="<YOUR_SENSOR_KIT_NAME>"/>
+   <let name="base_frame" value="base_link"/>
+   <let name="parent_frame" value="sensor_kit_base_link"/>
+
+   <!-- You can change after the saving of rviz config like this -->
+   <let name="rviz_profile" value="$(find-pkg-share extrinsic_ground_plane_calibrator)/rviz/velodyne_top.rviz"/>
+   <arg name="calibration_rviz" default="true"/>
+
+   <!-- extrinsic_calibration_client -->
+   <arg name="src_yaml" default="$(find-pkg-share individual_params)/config/$(var vehicle_id)/$(var sensor_model)/sensor_kit_calibration.yaml"/>
+   <arg name="dst_yaml" default="$(env HOME)/sensor_kit_calibration.yaml"/>
+
+   <node pkg="extrinsic_calibration_client" exec="extrinsic_calibration_client" name="extrinsic_calibration_client" output="screen">
+     <param name="src_path" value="$(var src_yaml)"/>
+     <param name="dst_path" value="$(var dst_yaml)"/>
+   </node>
```

Then, we will add all our sensor frames on extrinsic_calibration_manager as child frames.

```diff
+   <!-- extrinsic_calibration_manager -->
+   <node pkg="extrinsic_calibration_manager" exec="extrinsic_calibration_manager" name="extrinsic_calibration_manager" output="screen">
+     <param name="parent_frame" value="$(var parent_frame)"/>
+     <!-- add your sensor frames here -->
+     <param name="child_frames" value="
+     [<YOUE_SENSOR_BASE_LINK>,
+     YOUE_SENSOR_BASE_LINK,
+     YOUE_SENSOR_BASE_LINK,
+     YOUE_SENSOR_BASE_LINK
+     ...]"/>
+   </node>
```

For tutorial_vehicle there are two lidar sensors (rs_helios_top and rs_bpearl_front),
so it will be like this:

??? note "i.e extrinsic_calibration_manager child_frames for tutorial_vehicle"

    ```xml
    +   <!-- extrinsic_calibration_manager -->
    +   <node pkg="extrinsic_calibration_manager" exec="extrinsic_calibration_manager" name="extrinsic_calibration_manager" output="screen">
    +     <param name="parent_frame" value="$(var parent_frame)"/>
    +     <!-- add your sensor frames here -->
    +     <param name="child_frames" value="
    +     [rs_helios_top_base_link,
    +     rs_bpearl_front_base_link]"/>
    +   </node>
    ```

After that we will add our lidar sensor configurations on ground-based calibrator,
to do that we will add these lines our `ground_plane_sensor_kit.launch.xml` file.

```diff
+  <group>
+   <include file="$(find-pkg-share extrinsic_ground_plane_calibrator)/launch/calibrator.launch.xml">
+     <arg name="ns" value="$(var parent_frame)/YOUR_SENSOR_BASE_LINK"/>
+     <arg name="base_frame" value="$(var base_frame)"/>
+     <arg name="parent_frame" value="$(var parent_frame)"/>
+     <arg name="child_frame" value="YOUR_SENSOR_BASE_LINK"/>
+     <arg name="pointcloud_topic" value="<YOUR_SENSOR_TOPIC_NAME>"/>
+   </include>
+ </group>
+  ...
+  ...
+  ...
+  ...
+  ...
+
```

??? note "i.e., launch calibrator.launch.xml for each tutorial_vehicle's lidar"

    ```xml
      <!-- rs_helios_top_base_link: extrinsic_ground_plane_calibrator -->
      <group>
        <include file="$(find-pkg-share extrinsic_ground_plane_calibrator)/launch/calibrator.launch.xml">
          <arg name="ns" value="$(var parent_frame)/rs_helios_top_base_link"/>
          <arg name="base_frame" value="$(var base_frame)"/>
          <arg name="parent_frame" value="$(var parent_frame)"/>
          <arg name="child_frame" value="rs_helios_top_base_link"/>
          <arg name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw"/>
        </include>
      </group>

      <!-- rs_bpearl_front_base_link: extrinsic_ground_plane_calibrator -->
      <group>
        <include file="$(find-pkg-share extrinsic_ground_plane_calibrator)/launch/calibrator.launch.xml">
          <arg name="ns" value="$(var parent_frame)/rs_bpearl_front_base_link"/>
          <arg name="base_frame" value="$(var base_frame)"/>
          <arg name="parent_frame" value="$(var parent_frame)"/>
          <arg name="child_frame" value="rs_bpearl_front_base_link"/>
          <arg name="pointcloud_topic" value="/sensing/lidar/front/pointcloud_raw"/>
        </include>
      </group>

      <node pkg="rviz2" exec="rviz2" name="rviz2" output="screen" args="-d $(var rviz_profile)" if="$(var calibration_rviz)"/>
    </launch>

    ```

The mapping_based_sensor_kit.launch.xml launch file for tutorial_vehicle should be this:

??? note "Sample ground_plane_sensor_kit.launch.xml for tutorial_vehicle"

    ```xml
    <?xml version="1.0" encoding="UTF-8"?>
    <launch>
    <arg name="vehicle_id" default="tutorial_vehicle"/>
    <let name="sensor_model" value="tutorial_vehicle_sensor_kit"/>
    <let name="base_frame" value="base_link"/>
    <let name="parent_frame" value="sensor_kit_base_link"/>
    <let name="rviz_profile" value="$(find-pkg-share extrinsic_ground_plane_calibrator)/rviz/velodyne_top.rviz"/>
    <arg name="calibration_rviz" default="true"/>

      <!-- extrinsic_calibration_client -->
      <arg name="src_yaml" default="$(find-pkg-share individual_params)/config/$(var vehicle_id)/$(var sensor_model)/sensor_kit_calibration.yaml"/>
      <arg name="dst_yaml" default="$(env HOME)/sensor_kit_calibration.yaml"/>

      <node pkg="extrinsic_calibration_client" exec="extrinsic_calibration_client" name="extrinsic_calibration_client" output="screen">
        <param name="src_path" value="$(var src_yaml)"/>
        <param name="dst_path" value="$(var dst_yaml)"/>
      </node>

      <!-- extrinsic_calibration_manager -->
      <node pkg="extrinsic_calibration_manager" exec="extrinsic_calibration_manager" name="extrinsic_calibration_manager" output="screen">
        <param name="parent_frame" value="$(var parent_frame)"/>
        <param name="child_frames" value="
        [rs_helios_top_base_link,
        rs_bpearl_front_base_link]"/>
      </node>

      <!-- rs_helios_top_base_link: extrinsic_ground_plane_calibrator -->
      <group>
        <include file="$(find-pkg-share extrinsic_ground_plane_calibrator)/launch/calibrator.launch.xml">
          <arg name="ns" value="$(var parent_frame)/rs_helios_top_base_link"/>
          <arg name="base_frame" value="$(var base_frame)"/>
          <arg name="parent_frame" value="$(var parent_frame)"/>
          <arg name="child_frame" value="rs_helios_top_base_link"/>
          <arg name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw"/>
        </include>
      </group>

      <!-- rs_bpearl_front_base_link: extrinsic_ground_plane_calibrator -->
      <group>
        <include file="$(find-pkg-share extrinsic_ground_plane_calibrator)/launch/calibrator.launch.xml">
          <arg name="ns" value="$(var parent_frame)/rs_bpearl_front_base_link"/>
          <arg name="base_frame" value="$(var base_frame)"/>
          <arg name="parent_frame" value="$(var parent_frame)"/>
          <arg name="child_frame" value="rs_bpearl_front_base_link"/>
          <arg name="pointcloud_topic" value="/sensing/lidar/front/pointcloud_raw"/>
        </include>
      </group>

      <node pkg="rviz2" exec="rviz2" name="rviz2" output="screen" args="-d $(var rviz_profile)" if="$(var calibration_rviz)"/>
    </launch>

    ```

After completing mapping_based.launch.xml and mapping_based_sensor_kit.launch.xml launch files for own sensor kit;
now we are ready to calibrate our lidars.
First of all, we need to build extrinsic_calibration_manager package:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select extrinsic_calibration_manager
```

So, we are ready to launch and use ground-based lidar-ground calibrator.

```bash
ros2 launch extrinsic_calibration_manager calibration.launch.xml mode:=ground_plane sensor_model:=<OWN-SENSOR-KIT> vehicle_model:=<OWN-VEHICLE-MODEL> vehicle_id:=<VEHICLE-ID>
```

For tutorial vehicle:

```bash
ros2 launch extrinsic_calibration_manager calibration.launch.xml mode:=ground_plane sensor_model:=tutorial_vehicle_sensor_kit vehicle_model:=tutorial_vehicle vehicle_id:=tutorial_vehicle
```

You will show the rviz2 screen with several configurations,
you need
to update it with your sensor information topics, sensor_frames and pointcloud_inlier_topics like the video,
which included an end of the document.
Also, you can save the rviz2 config on rviz directory,
so you can use it later with modifying `mapping_based_sensor_kit.launch.xml`.

```diff
extrinsic_mapping_based_calibrator/
   └─ rviz/
+        └─ tutorial_vehicle_sensor_kit.rviz
```

Then play ROS 2 bag file, the calibration process will be started:

```bash
ros2 bag play <rosbag_path> --clock -l -r 0.2 \
--remap /tf:=/null/tf /tf_static:=/null/tf_static # if tf is recorded
```

Since the calibration process is done automatically,
you can see the sensor_kit_calibration.yaml in your $HOME directory after the calibration process is complete.

|          Before Ground Plane - Lidar Calibration           |             After Ground Plane - Lidar Calibration              |
| :--------------------------------------------------------: | :-------------------------------------------------------------: |
| ![before-ground-plane.png](images/before-ground-plane.png) | ![images/after-ground-plane.png](images/after-ground-plane.png) |

Here is the video for demonstrating a ground plane - lidar calibration process on tutorial_vehicle:
![type:video](https://youtube.com/embed/EqaF1fufjUc)
