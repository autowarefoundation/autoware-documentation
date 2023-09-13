# Lidar-Camera calibration

## Overview

Lidar-camera calibration is a crucial process in the field of autonomous driving and robotics,
where both lidar sensors and cameras are used for perception.
The goal of calibration is
to accurately align the data from these different sensors
in order to create a comprehensive and coherent representation of the environment
by projecting lidar point onto camera image.
At this tutorial,
we will explain [TIER IV's interactive camera calibrator](https://github.com/leo-drive/CalibrationTools/blob/golf_test/sensor/docs/how_to_extrinsic_interactive.md).
Also, If you have aruco marker boards for calibration,
another [Lidar-Camera calibration method](https://github.com/leo-drive/CalibrationTools/blob/golf_test/sensor/docs/how_to_extrinsic_tag_based.md) is included in TIER IV's CalibrationTools repository.

!!! warning

    Please get initial calibration results from [Generic Calibration](./generic-calibration.md) section, it is important for getting accurate results from this tool.
    We will use initial calibration parameters that we calculated on previous step on this tutorial for lidar-camera extrinsic calibration.

Your bag file must include calibration lidar topic and camera topics.
Camera topics can be compressed or raw topics,
but remember
we will update interactive calibrator launch argument `use_compressed` according to the topic type.

??? note "ROS 2 Bag example of our calibration process for tutorial_vehicle (there is only one camera mounted.) If you have multiple cameras, please add camera_info and image topics as well"

    ```sh

    Files:             rosbag2_2023_09_12-13_57_03_0.db3
    Bag size:          5.8 GiB
    Storage id:        sqlite3
    Duration:          51.419s
    Start:             Sep 12 2023 13:57:03.691 (1694516223.691)
    End:               Sep 12 2023 13:57:55.110 (1694516275.110)
    Messages:          2590
    Topic information: Topic: /sensing/lidar/top/pointcloud_raw | Type: sensor_msgs/msg/PointCloud2 | Count: 515 | Serialization Format: cdr
    Topic: /sensing/camera/camera0/image_raw | Type: sensor_msgs/msg/Image | Count: 780 | Serialization Format: cdr
    Topic: /sensing/camera/camera0/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 780 | Serialization Format: cdr

    ```

## Lidar-Camera calibration

We start with creating launch file four our vehicle like "Extrinsic Manual Calibration"
process:

```bash
cd <YOUR-OWN-AUTOWARE-DIRECTORY>/src/autoware/calibration_tools/sensor
cd extrinsic_calibration_manager/launch
cd <YOUR-OWN-SENSOR-KIT-NAME> # i.e. for our guide, it will ve cd tutorial_vehicle_sensor_kit which is created in generic calibration
touch interactive.launch.xml interactive_sensor_kit.launch.xml
```

The created `interactive.launch.xml` and `interactive_sensor_kit.launch.xml` are version of sample sensor kit
[aip_xx1](https://github.com/tier4/CalibrationTools/tree/tier4/universe/sensor/extrinsic_calibration_manager/launch/aip_xx1) provided from TIER IV.

Then we will continue with adding vehicle_id and sensor model names to the `mapping_based.launch.xml`.
(Optionally, values are not important. These parameters Overrode from launch argument) Then,
we will add camera_name for calibrating camera
(can be one of the camera0, camera1, camera_front etc. as launch argument)
and `use_concatenated_pointcloud` argument.
If you want to use concatenated pointcloud as an input cloud
(the calibration process will initiate the logging simulator,
resulting in the construction of the lidar pipeline and the appearance of the concatenated point cloud),
you must set `use_concatenated_pointcloud` value as `true`.

```diff
+ <?xml version="1.0" encoding="UTF-8"?>
+ <launch>
+   <arg name="vehicle_id" default="<YOUR_VEHICLE_ID>"/>
+   <let name="sensor_model" value="<YOUR_SENSOR_KIT_NAME>"/>
+   <arg name="rviz" default="false"/>
+   <arg name="camera_name"/>
+   <arg name="use_concatenated_pointcloud" default="true"/>
```

After that, we will launch our sensor_kit for lidar-camera calibration,
so we must add these lines on interactive.launch.xml (Also, you can change rviz path (args) after the saving rviz config):

```diff
+  <group>
+    <push-ros-namespace namespace="sensor_kit"/>
+    <include file="$(find-pkg-share extrinsic_calibration_manager)/launch/$(var sensor_model)/interactive_sensor_kit.launch.xml" if="$(var rviz)">
+      <arg name="vehicle_id" value="$(var vehicle_id)"/>
+      <arg name="camera_name" value="$(var camera_name)"/>
+    </include>
+  </group>
+
+  <!-- You can change the config file path -->
+  <node pkg="rviz2" exec="rviz2" name="rviz2" output="screen" args="
+        -d $(find-pkg-share extrinsic_calibration_manager)/config/x2/extrinsic_interactive_calibrator.rviz" if="$(var rviz)"/>
```

The final version of the file (interactive.launch.xml) for tutorial_vehicle should be like this:

??? note "Sample interactive.launch.xml file for tutorial vehicle"

    ```xml
    <?xml version="1.0" encoding="UTF-8"?>
    <launch>
      <arg name="vehicle_id" default="tutorial_vehicle"/>
      <let name="sensor_model" value="tutorial_vehicle_sensor_kit"/>
      <arg name="camera_name"/>
      <arg name="rviz" default="false"/>
      <arg name="use_concatenated_pointcloud" default="true"/>

      <group>
        <push-ros-namespace namespace="sensor_kit"/>
        <include file="$(find-pkg-share extrinsic_calibration_manager)/launch/$(var sensor_model)/interactive_sensor_kit.launch.xml" if="$(var rviz)">
          <arg name="vehicle_id" value="$(var vehicle_id)"/>
          <arg name="camera_name" value="$(var camera_name)"/>
        </include>
      </group>

    <!-- You can change the config file path -->
    <node pkg="rviz2" exec="rviz2" name="rviz2" output="screen" args="
    -d $(find-pkg-share extrinsic_calibration_manager)/config/x2/extrinsic_interactive_calibrator.rviz" if="$(var rviz)"/>
    </launch>

    ```

After the completing of interactive.launch.xml file,
we will be ready to implement interactive_sensor_kit.launch.xml for the own sensor model.

Optionally, (don't forget, these parameters overrode by ROS 2 launch arguments.)
you can modify sensor_kit and vehicle_id as `interactive.launch.xml`over this xml snippet.
We will set parent_frame for calibration as `sensor_kit_base_link``:

```diff
+ <?xml version="1.0" encoding="UTF-8"?>
+ <launch>
+   <arg name="vehicle_id" default="<YOUR_VEHICLE_ID>"/>
+   <let name="sensor_model" value="<YOUR_SENSOR_KIT_NAME>"/>
+   <let name="parent_frame" value="sensor_kit_base_link"/>
+
+   <!-- extrinsic_calibration_client -->
+   <arg name="src_yaml" default="$(find-pkg-share individual_params)/config/$(var vehicle_id)/$(var sensor_model)/sensor_kit_calibration.yaml"/>
+   <arg name="dst_yaml" default="$(env HOME)/sensor_kit_calibration.yaml"/>
+
```

Next, we will define `camera_name` argument.
This parameter describes which camera will be calibrated.
For example,
your bag file can contain six camera topics
(camera0, camera1, etc.) but you can calibrate one camera at a time.
After that, we will define image_topic rule.
For example at this snippet, it controlled over `camera_name` variable.
Also, if you want to use compressed image for calibration,
you need update `image_topic` and `use_compressed` value with compressed topic information.

```diff
+   <arg name="camera_name"/>
+
+   <let name="image_topic" value="/sensing/camera/$(var camera_name)/image_raw"/>
+   <let name="camera_info_topic" value="/sensing/camera/$(var camera_name)/camera_info"/>
+
+   <!-- if your image topic is compressed, please enable it -->
+   <let name="use_compressed" value="false"/>
```

Then you can customize pointcloud topic for each camera,
for example,
we can add this evaluation mechanism to determining pointcloud_topic name
(in this case, all camera parent lidars are "top" lidar):

```diff
+   <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'camera0' &quot;)"/>
+   <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'camera1' &quot;)"/>
+   <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'camera2' &quot;)"/>
+   <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'camera3' &quot;)"/>
+   <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'camera4' &quot;)"/>
+   <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'camera5' &quot;)"/>
+   ...
```

??? note "Since there is one camera on tutorial_vehicle which is named as camera0, so pointcloud topic should be like this"

    ```xml

    +   <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'camera0' &quot;)"/>

    ```

We will add a calibration checking mechanism
to be sure that the right camera_name parameter is set.
For example, if camera_name is set launch argument as `camera10`,
the calibration process doesn't occur.

```diff
+   <let name="calibrate_sensor" value="false"/>
+   <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'camera0' &quot;)"/>
+   <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'camera1' &quot;)"/>
+   <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'camera2' &quot;)"/>
+   <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'camera3' &quot;)"/>
+   <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'camera4' &quot;)"/>
+   <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'camera5' &quot;)"/>
+   ...
```

??? note "Since there is one camera on tutorial_vehicle which is named as camera0, calibrate_sensor argument structure should be like this"

    ```xml

    +   <let name="calibrate_sensor" value="false"/>
    +   <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'camera0' &quot;)"/>

    ```

After that, we will define camera_links for each camera,
This structure checks
which camera will be calibrated and sets `camera_frame` according to the `camera_name`:

```diff
+   <let name="camera_frame" value=""/>
+   <let name="camera_frame" value="camera0/camera_link" if="$(eval &quot;'$(var camera_name)' == 'camera0' &quot;)"/>
+   <let name="camera_frame" value="camera1/camera_link" if="$(eval &quot;'$(var camera_name)' == 'camera1' &quot;)"/>
+   <let name="camera_frame" value="camera2/camera_link" if="$(eval &quot;'$(var camera_name)' == 'camera2' &quot;)"/>
+   <let name="camera_frame" value="camera3/camera_link" if="$(eval &quot;'$(var camera_name)' == 'camera3' &quot;)"/>
+   <let name="camera_frame" value="camera4/camera_link" if="$(eval &quot;'$(var camera_name)' == 'camera4' &quot;)"/>
+   <let name="camera_frame" value="camera5/camera_link" if="$(eval &quot;'$(var camera_name)' == 'camera5' &quot;)"/>
```

??? note "Since there is one camera on tutorial_vehicle which is named as camera0, camera_frame argument should be defined like this"

    ```xml

    +   <let name="camera_frame" value=""/>
    +   <let name="camera_frame" value="camera0/camera_link" if="$(eval &quot;'$(var camera_name)' == 'camera0' &quot;)"/>

    ```

Now,
we will launch the extrinsic_calibration_manager,
extrinsic_calibration_client and interactive calibrator according to the arguments
we defined before.

```diff
+   <node pkg="extrinsic_calibration_client" exec="extrinsic_calibration_client" name="extrinsic_calibration_client" output="screen" if="$(var calibrate_sensor)">
+     <param name="src_path" value="$(var src_yaml)"/>
+     <param name="dst_path" value="$(var dst_yaml)"/>
+   </node>
+   <!-- extrinsic_calibration_manager -->
+   <node pkg="extrinsic_calibration_manager" exec="extrinsic_calibration_manager" name="extrinsic_calibration_manager" output="screen" if="$(var calibrate_sensor)">
+     <param name="parent_frame" value="$(var parent_frame)"/>
+     <param name="child_frames" value="
+     [$(var camera_frame)]"/>
+   </node>
+
+   <!-- interactive calibrator -->
+   <group if="$(var calibrate_sensor)">
+     <push-ros-namespace namespace="$(var parent_frame)/$(var camera_frame)"/>
+
+     <node pkg="extrinsic_interactive_calibrator" exec="interactive_calibrator" name="interactive_calibrator" output="screen">
+       <remap from="pointcloud" to="$(var pointcloud_topic)"/>
+       <remap from="image" to="$(var image_topic)"/>
+       <remap from="camera_info" to="$(var camera_info_topic)"/>
+       <remap from="calibration_points_input" to="calibration_points"/>
+
+       <param name="camera_parent_frame" value="$(var parent_frame)"/>
+       <param name="camera_frame" value="$(var camera_frame)"/>
+       <param name="use_compressed" value="$(var use_compressed)"/>
+     </node>
+
+     <include file="$(find-pkg-share intrinsic_camera_calibration)/launch/optimizer.launch.xml"/>
+   </group>
</launch>
```

The interactive_sensor_kit.launch.xml launch file for tutorial_vehicle should be this:

??? note "i.e. interactive_sensor_kit.launch.xml for tutorial_vehicle"

    ```xml
    <?xml version="1.0" encoding="UTF-8"?>
    <launch>
        <arg name="vehicle_id" default="tutorial_vehicle"/>
        <let name="sensor_model" value="tutorial_vehicle_sensor_kit"/>
        <let name="parent_frame" value="sensor_kit_base_link"/>

        <!-- extrinsic_calibration_client -->
        <arg name="src_yaml" default="$(find-pkg-share individual_params)/config/$(var vehicle_id)/$(var sensor_model)/sensor_kit_calibration.yaml"/>
        <arg name="dst_yaml" default="$(env HOME)/sensor_kit_calibration.yaml"/>


        <arg name="camera_name"/>

        <let name="image_topic" value="/sensing/camera/$(var camera_name)/image_raw"/>
        <let name="image_topic" value="/sensing/camera/traffic_light/image_raw" if="$(eval &quot;'$(var camera_name)' == 'traffic_light_left_camera' &quot;)"/>

        <let name="use_compressed" value="false"/>

        <let name="image_compressed_topic" value="/sensing/camera/$(var camera_name)/image_raw/compressed"/>
        <let name="image_compressed_topic" value="/sensing/camera/traffic_light/image_raw/compressed" if="$(eval &quot;'$(var camera_name)' == 'traffic_light_left_camera' &quot;)"/>

        <let name="camera_info_topic" value="/sensing/camera/$(var camera_name)/camera_info"/>
        <let name="camera_info_topic" value="/sensing/camera/traffic_light/camera_info" if="$(eval &quot;'$(var camera_name)' == 'traffic_light_left_camera' &quot;)"/>

        <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'camera0' &quot;)"/>
        <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'camera1' &quot;)"/>
        <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'camera2' &quot;)"/>
        <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'camera3' &quot;)"/>
        <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'camera4' &quot;)"/>
        <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'camera5' &quot;)"/>
        <let name="pointcloud_topic" value="/sensing/lidar/top/pointcloud_raw" if="$(eval &quot;'$(var camera_name)' == 'traffic_light_left_camera' &quot;)"/>

        <let name="calibrate_sensor" value="false"/>
        <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'camera0' &quot;)"/>
        <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'camera1' &quot;)"/>
        <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'camera2' &quot;)"/>
        <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'camera3' &quot;)"/>
        <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'camera4' &quot;)"/>
        <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'camera5' &quot;)"/>
        <let name="calibrate_sensor" value="true" if="$(eval &quot;'$(var camera_name)' == 'traffic_light_left_camera' &quot;)"/>

        <let name="camera_frame" value=""/>
        <let name="camera_frame" value="camera0/camera_link" if="$(eval &quot;'$(var camera_name)' == 'camera0' &quot;)"/>
        <let name="camera_frame" value="camera1/camera_link" if="$(eval &quot;'$(var camera_name)' == 'camera1' &quot;)"/>
        <let name="camera_frame" value="camera2/camera_link" if="$(eval &quot;'$(var camera_name)' == 'camera2' &quot;)"/>
        <let name="camera_frame" value="camera3/camera_link" if="$(eval &quot;'$(var camera_name)' == 'camera3' &quot;)"/>
        <let name="camera_frame" value="camera4/camera_link" if="$(eval &quot;'$(var camera_name)' == 'camera4' &quot;)"/>
        <let name="camera_frame" value="camera5/camera_link" if="$(eval &quot;'$(var camera_name)' == 'camera5' &quot;)"/>
        <let name="camera_frame" value="traffic_light_left_camera/camera_link" if="$(eval &quot;'$(var camera_name)' == 'traffic_light_left_camera' &quot;)"/>

        <node pkg="extrinsic_calibration_client" exec="extrinsic_calibration_client" name="extrinsic_calibration_client" output="screen" if="$(var calibrate_sensor)">
            <param name="src_path" value="$(var src_yaml)"/>
            <param name="dst_path" value="$(var dst_yaml)"/>
        </node>

        <!-- extrinsic_calibration_manager -->
        <node pkg="extrinsic_calibration_manager" exec="extrinsic_calibration_manager" name="extrinsic_calibration_manager" output="screen" if="$(var calibrate_sensor)">
            <param name="parent_frame" value="$(var parent_frame)"/>
            <param name="child_frames" value="
        [$(var camera_frame)]"/>
        </node>

        <!-- interactive calibrator -->
        <group if="$(var calibrate_sensor)">
            <push-ros-namespace namespace="$(var parent_frame)/$(var camera_frame)"/>

            <node pkg="extrinsic_interactive_calibrator" exec="interactive_calibrator" name="interactive_calibrator" output="screen">
                <remap from="pointcloud" to="$(var pointcloud_topic)"/>
                <remap from="image" to="$(var image_topic)"/>
                <remap from="camera_info" to="$(var camera_info_topic)"/>
                <remap from="calibration_points_input" to="calibration_points"/>

                <param name="camera_parent_frame" value="$(var parent_frame)"/>
                <param name="camera_frame" value="$(var camera_frame)"/>
                <param name="use_compressed" value="$(var use_compressed)"/>
            </node>

            <include file="$(find-pkg-share intrinsic_camera_calibration)/launch/optimizer.launch.xml"/>
        </group>
    </launch>

    ```

After completing interactive.launch.xml and interactive_sensor_kit.launch.xml launch files for own sensor kit;
now we are ready to calibrate our lidars.
First of all, we need to build extrinsic_calibration_manager package:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select extrinsic_calibration_manager
```

So, we are ready to launch and use interactive lidar-camera calibrator.

```bash
ros2 launch extrinsic_calibration_manager calibration.launch.xml mode:=interactive sensor_model:=<OWN-SENSOR-KIT> vehicle_model:=<OWN-VEHICLE-MODEL> vehicle_id:=<VEHICLE-ID> camera_name:=<CALIBRATION-CAMERA>
```

For tutorial vehicle:

```bash
ros2 launch extrinsic_calibration_manager calibration.launch.xml mode:=mapping_based sensor_model:=tutorial_vehicle_sensor_kit vehicle_model:=tutorial_vehicle vehicle_id:=tutorial_vehicle
```
