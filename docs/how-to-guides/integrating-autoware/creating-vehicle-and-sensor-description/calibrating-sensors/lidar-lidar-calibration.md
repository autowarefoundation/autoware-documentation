# Lidar-Lidar Calibration

## Overview

In this tutorial,
we will explain lidar-lidar calibration over [mapping-based lidar-lidar calibration tool](https://github.com/tier4/CalibrationTools/blob/tier4/universe/sensor/docs/how_to_extrinsic_mapping_based.md) of Tier IV's
CalibrationTools.
Also,
[map-based lidar-lidar calibration method](https://github.com/tier4/CalibrationTools/blob/tier4/universe/sensor/docs/how_to_extrinsic_map_based.md) included in the Tier IV's tools.
[LL-Calib](https://github.com/autocore-ai/calibration_tools/tree/main/lidar-lidar-calib) on GitHub,
provided by [AutoCore](https://autocore.ai/),
is a lightweight toolkit for online/offline 3D LiDAR to LiDAR calibration.
It's based on local mapping and "GICP" method to derive the relation between main and sub lidar.
If you want more details about these methods such as usage, troubleshooting etc. please check the links above.

!!! warning

    Please get initial calibration results from [Generic Calibration](./generic-calibration.md) section, it is important for getting accurate results from this tool.
    We will use initial calibration parameters that we calculated on previous step on this tutorial.

## Mapping-based lidar-lidar

We start with creating launch file four our vehicle like "Extrinsic Manual Calibration"
process:

```bash
cd <YOUR-OWN-AUTOWARE-DIRECTORY>/src/autoware/calibration_tools/sensor
cd extrinsic_calibration_manager/launch
cd <YOUR-OWN-SENSOR-KIT-NAME> # i.e. for our guide, it will ve cd tutorial_vehicle_sensor_kit
touch mapping_based.launch.xml mapping_based_sensor_kit.launch.xml
```

The created `mapping_based.launch.xml` and `mapping_based_sensor_kit.launch.xml` are version of sample sensor kit
[aip_x1](https://github.com/tier4/CalibrationTools/tree/tier4/universe/sensor/extrinsic_calibration_manager/launch/aip_x1) provided from Tier IV.

Then we will continue with adding vehicle_id and sensor model names to the `mapping_based.launch.xml`.
(Optionally, values are not important. These parameters Overrode from launch argument)

```diff
+ <?xml version="1.0" encoding="UTF-8"?>
+ <launch>
+   <arg name="vehicle_id" default="tutorial_vehicle"/> <!-- You can update with your own vehicle_id -->
+
+   <let name="sensor_model" value="tutorial_vehicle_sensor_kit"/> <!-- You can update with your own sensor_kit -->
```

After that, we will launch our sensor_kit for mapping-based lidar-lidar calibration,
so we must add these lines on manual.launch.xml:

```diff
+   <group>
+     <push-ros-namespace namespace="sensor_kit"/>
+     <include file="$(find-pkg-share extrinsic_calibration_manager)/launch/$(var sensor_model)/mapping_based_sensor_kit.launch.xml">
+       <arg name="vehicle_id" value="$(var vehicle_id)"/>
+     </include>
+   </group>
+
+ </launch>
```

The final version of the file (mapping_based.launch.xml) for tutorial_vehicle should be like this:

??? note "Sample manual.launch.xml file for tutorial vehicle"

    ```xml
    <?xml version="1.0" encoding="UTF-8"?>
    <launch>
      <arg name="vehicle_id" default="tutorial_vehicle"/>
      <let name="sensor_model" value="tutorial_vehicle_sensor_kit"/>
      <arg name="rviz" default="true"/>

      <group>
        <push-ros-namespace namespace="sensor_kit"/>
        <include file="$(find-pkg-share extrinsic_calibration_manager)/launch/$(var sensor_model)/mapping_based_sensor_kit.launch.xml">
          <arg name="vehicle_id" value="$(var vehicle_id)"/>
          <arg name="rviz" value="$(var rviz)"/>
        </include>
      </group>
    </launch>

    ```

After the completing of mapping_based.launch.xml file,
we will be ready to implement mapping_based_sensor_kit.launch.xml for the own sensor model:

Optionally,
you can modify sensor_kit and vehicle_id as `mapping_based.launch.xml`over this xml snippet:
(You can change rviz_profile path after the saving rviz config as video
which included at the end of the page)

```diff
+ <?xml version="1.0" encoding="UTF-8"?>
+ <launch>
+   <arg name="vehicle_id" default="tutorial_vehicle"/>
+   <let name="sensor_model" value="tutorial_vehicle_sensor_kit"/>
+
+   <arg name="rviz"/>
+   <let name="rviz_profile" value="$(find-pkg-share extrinsic_mapping_based_calibrator)/rviz/x1.rviz"/>
+
+   <!-- You can change after the saving of rviz config like this -->
+   <!-- <let name="rviz_profile" value="$(find-pkg-share extrinsic_mapping_based_calibrator)/rviz/tutorial_vehicle.rviz"/> -->
+
+   <arg name="src_yaml" default="$(find-pkg-share individual_params)/config/$(var vehicle_id)/$(var sensor_model)/sensor_kit_calibration.yaml"/>
+   <arg name="dst_yaml" default="$(env HOME)/sensor_kit_calibration.yaml"/>
```

We will not fill anything on camera parameters for calibrator
because we will pair lidar-lidar for the calibration process:

```diff
+   <let name="camera_calibration_sensor_kit_frames" value="['']"/>
+   <let name="calibration_camera_frames" value="['']"/>
+   <let name="calibration_camera_optical_link_frames" value="['']"/>
+   <let name="calibration_camera_info_topics" value="['']"/>
+   <let name="calibration_image_topics" value="['']"/>
```

We will add sensor kit frames for each lidar (except mapping lidar),
we have one lidar for pairing to the main lidar sensor for tutorial vehicle, so it should be like:

```diff
+   <let name="lidar_calibration_sensor_kit_frames" value="[sensor_kit_base_link]"/>
```

??? note "i.e., If you have three lidars (one main for mapping, two others)"

    ```xml
    +  <let name="lidar_calibration_sensor_kit_frames" value="[
    +  sensor_kit_base_link,
    +  sensor_kit_base_link,
    +  sensor_kit_base_link]"/>
    ```

We will add lidar_calibration_service_names,
calibration_lidar_base_frames and calibration_lidar_frames for calibrator.
At the tutorial_vehicle it should be like this snippet:

```diff
+   <let
+           name="lidar_calibration_service_names"
+           value="[/sensor_kit/sensor_kit_base_link/rs_bpearl_front_base_link]"
+   />

+   <let name="calibration_lidar_base_frames" value="[rs_bpearl_front_base_link]"/>
+   <let name="calibration_lidar_frames" value="[rs_bpearl_front]"/>
```

??? note "i.e., If you have three lidars (one main for mapping, two others)"

    ```xml
    <let
    name="lidar_calibration_service_names"
    value="[
    /sensor_kit/sensor_kit_base_link/livox_front_left_base_link,
    /sensor_kit/sensor_kit_base_link/livox_front_right_base_link]"
    />
    <let name="calibration_lidar_base_frames" value="[
    livox_front_left_base_link,
    livox_front_right_base_link]"/>

    <let name="calibration_lidar_frames" value="[
    livox_front_left,
    livox_front_right]"/>
    ```
