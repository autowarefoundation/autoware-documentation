# Creating Sensor Kit Description

## Introduction

The primary objective behind the development of this package is to delineate the sensor types,
frame IDs, calibration parameters of all sensors, and to call initialization files for all sensors.

This page introduces the following topics;

- < VEHICLE-ID > \_sensor_kit_description
- < VEHICLE-ID > \_sensor_kit_launch
- common_sensor_launch

!!! Note

    [Here](https://github.com/leo-drive/tutorial_vehicle_sensor_kit) is the sample repository created for the tutorial vehicle.

- You must change the sensor_kit with you forked at creating repository sections.

```diff
sensor_kit/
-   ├─ sample_sensor_kit_launch/
+   └─ VEHICLE-ID_sensor_kit_launch/
```

## 1. VEHICLE-ID_sensor_kit_description

The contents of this package encompass xacro-formatted macro files that serve as interpretative tools for defining sensors, sensor calibration outcomes, and description files.

### Folder Structure

The folder structure of forked `VEHICLE-ID_sensor_kit_description` repository should be like this:

```diff
VEHICLE-ID_sensor_kit_description/
   ├─ config/
   │     ├─ sensor_kit_calibration.yaml
   │     └─ sensors_calibration.yaml
   └─ urdf/
         ├─ sensor_kit.xacro
         └─ sensors.xacro
```

In `VEHICLE-ID_sensor_kit_description`, the following configurations are set:

- sensor_kit_calibration.yaml
- sensors_calibration.yaml
- sensor_kit.xacro
- sensors.xacro

### 1. sensor_kit_calibration.yaml

A file that defines the mounting positions and orientations of sensors with `sensor_kit_base_link` as the parent frame.

The original file is [sensor_kit_calibration.yaml](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_description/config/sensor_kit_calibration.yaml) included in [sample_sensor_kit_launch](https://github.com/autowarefoundation/sample_sensor_kit_launch/tree/main) and given below.

```yaml
sensor_kit_base_link:
  camera0/camera_link:
    x: 0.10731
    y: 0.56343
    z: -0.27697
    roll: -0.025
    pitch: 0.315
    yaw: 1.035
    .
    .
    .
```

A sensor_kit refers to a subset that includes multiple sensors,
and `sensor_kit_base_link` is the name of sensor_kit frame in `sensor_kit_calibration.yaml`.
In the file above, the frame_ids of the sensors and the calibration results of the sensors,
according to the parent_frame, are given.
Calibration results are given in euler format as [x, y, z, roll, pitch, yaw].

The tutorial_vehicle file is [sensor_kit_calibration.yaml](https://github.com/leo-drive/tutorial_vehicle_sensor_kit/blob/main/tutorial_vehicle_sensor_kit_description/config/sensor_kit_calibration.yaml) included in [tutorial_vehicle_sensor_kit](https://github.com/leo-drive/tutorial_vehicle_sensor_kit/tree/main) and given below.
This sample file was created for one camera, two lidars and 1 GNSS/INS.

??? note "sensor_kit_calibration.yaml for tutorial_vehicle_sensor_kit_launch"

    ```yaml
    sensor_kit_base_link:
      camera0/camera_link: # Camera
        x: 0.0
        y: 0.0
        z: 0.0
        roll: 0.0
        pitch: 0.0
        yaw: 0.0
      rs_helios_top_base_link: # Lidar
        x: 0.0
        y: 0.0
        z: 0.0
        roll: 0.0
        pitch: 0.0
        yaw: 0.0
      rs_bpearl_front_base_link: # Lidar
        x: 0.0
        y: 0.0
        z: 0.0
        roll: 0.0
        pitch: 0.0
        yaw: 0.0
      gnss_link: # GNSS/INS
        x: 0.0
        y: 0.0
        z: 0.0
        roll: 0.0
        pitch: 0.0
        yaw: 0.0
    ```

As seen above, [x, y, z, roll, pitch, yaw] values are set to 0 for all sensors. This is because the relevant values in the created file will be found after the calibration between the sensor and the `sensor_kit_base_link`. The parent_frame expressed by `sensor_kit_base_link` can belong to any sensor. However, it is preferred that the frame of the sensor located in the center
of the vehicle is parent.

### 2. sensors_calibration.yaml

A file that defines the mounting positions and orientations of sensors with `base_link` as the parent frame.

The original file is [sensors_calibration.yaml](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_description/config/sensors_calibration.yaml) included in [sample_sensor_kit_launch](https://github.com/autowarefoundation/sample_sensor_kit_launch/tree/main) and given below.

```yaml
  base_link:
    sensor_kit_base_link:
    x: 0.9
    y: 0.0
    z: 2.0
    roll: -0.001
    pitch: 0.015
    yaw: -0.0364
    .
    .
    .
```

The frame given as `base_link` in the above file can be defined as point 0 of the vehicle.
For this point, the desired sensor frame or any point on the vehicle can be selected,
and the most suitable point is the point defined as the "axle,"
which is the middle of the two rear wheels of the vehicle.

The tutorial_vehicle file is [sensors_calibration.yaml](https://github.com/leo-drive/tutorial_vehicle_sensor_kit/blob/main/tutorial_vehicle_sensor_kit_description/config/sensors_calibration.yaml) included in [tutorial_vehicle_sensor_kit](https://github.com/leo-drive/tutorial_vehicle_sensor_kit/tree/main) and given below.
In sensors_calibration.yaml file, parent_frame's position relative to base_link is given in euler format as [x, y, z, roll, pitch, yaw].

```yaml
base_link:
  sensor_kit_base_link:
  x: 0.0
  y: 0.0
  z: 0.0
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

In this example file, the top_lidar in the center of the vehicle is selected as the parent_frame, and it is prepared in such a way that the position of the parent_frame element relative to the axle of the vehicle is entered.
The [x, y, z, roll, pitch, yaw] values in this file can be entered by base_link - lidar calibration or by measuring from the 3D model.

### 3. sensor_kit.xacro

Resolves the positions of sensors with sensor_kit_base_link as the parent and defines the positions and orientations based on sensor_kit_calibration.yaml in `individual_params`.

The default [sensor_kit.xacro](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_description/urdf/sensor_kit.xacro)
included in [sample_sensor_kit_launch](https://github.com/autowarefoundation/sample_sensor_kit_launch/tree/main) file for sample_sensor_kit is given below.

??? note "sensor_kit.xacro for sample_sensor_kit_launch"

    ```xml
    <?xml version="1.0"?>
    <robot xmlns:xacro="http://ros.org/wiki/xacro">
      <xacro:macro name="sensor_kit_macro" params="parent x y z roll pitch yaw">
        <xacro:include filename="$(find velodyne_description)/urdf/VLP-16.urdf.xacro"/>
        <xacro:include filename="$(find vls_description)/urdf/VLS-128.urdf.xacro"/>
        <xacro:include filename="$(find camera_description)/urdf/monocular_camera.xacro"/>
        <xacro:include filename="$(find imu_description)/urdf/imu.xacro"/>

        <xacro:arg name="gpu" default="false"/>
        <xacro:arg name="config_dir" default="$(find sample_sensor_kit_description)/config"/>

        <xacro:property name="sensor_kit_base_link" default="sensor_kit_base_link"/>

        <joint name="${sensor_kit_base_link}_joint" type="fixed">
          <origin rpy="${roll} ${pitch} ${yaw}" xyz="${x} ${y} ${z}"/>
          <parent link="${parent}"/>
          <child link="${sensor_kit_base_link}"/>
        </joint>
        <link name="${sensor_kit_base_link}">
          <origin rpy="0 0 0" xyz="0 0 0"/>
        </link>

        <!-- sensor -->
        <xacro:property name="calibration" value="${xacro.load_yaml('$(arg config_dir)/sensor_kit_calibration.yaml')}"/>

        <!-- lidar -->
        <xacro:VLS-128 parent="sensor_kit_base_link" name="velodyne_top" topic="/points_raw" hz="10" samples="220" gpu="$(arg gpu)">
          <origin
            xyz="${calibration['sensor_kit_base_link']['velodyne_top_base_link']['x']}
                 ${calibration['sensor_kit_base_link']['velodyne_top_base_link']['y']}
                 ${calibration['sensor_kit_base_link']['velodyne_top_base_link']['z']}"
            rpy="${calibration['sensor_kit_base_link']['velodyne_top_base_link']['roll']}
                 ${calibration['sensor_kit_base_link']['velodyne_top_base_link']['pitch']}
                 ${calibration['sensor_kit_base_link']['velodyne_top_base_link']['yaw']}"
          />
          .
          .
          .
      </xacro:macro>
    </robot>
    ```

The tutorial_vehicle's [sensor_kit.xacro](https://github.com/leo-drive/tutorial_vehicle_sensor_kit/blob/main/tutorial_vehicle_sensor_kit_description/urdf/sensor_kit.xacro) included in [tutorial_vehicle_sensor_kit](https://github.com/leo-drive/tutorial_vehicle_sensor_kit/tree/main).
The `sensor_kit.xacro` given below for the tutorial vehicle is prepared for 1 camera,
two lidars and 1 GNSS/INS,
and there are definitions of macro files in xacro format for each different sensor.

??? note "sensor_kit.xacro for tutorial_vehicle_sensor_kit_launch"

    ```yaml
    <?xml version="1.0"?>
    <robot xmlns:xacro="http://ros.org/wiki/xacro">
      <xacro:macro name="sensor_kit_macro" params="parent x y z roll pitch yaw">
        <xacro:include filename="$(find velodyne_description)/urdf/VLP-16.urdf.xacro"/>
        <xacro:include filename="$(find vls_description)/urdf/VLS-128.urdf.xacro"/>
        <xacro:include filename="$(find camera_description)/urdf/monocular_camera.xacro"/>
        <xacro:include filename="$(find imu_description)/urdf/imu.xacro"/>

        <xacro:arg name="gpu" default="false"/>
        <xacro:arg name="config_dir" default="$(find sample_sensor_kit_description)/config"/>

        <xacro:property name="sensor_kit_base_link" default="sensor_kit_base_link"/>

        <joint name="${sensor_kit_base_link}_joint" type="fixed">
          <origin rpy="${roll} ${pitch} ${yaw}" xyz="${x} ${y} ${z}"/>
          <parent link="${parent}"/>
          <child link="${sensor_kit_base_link}"/>
        </joint>
        <link name="${sensor_kit_base_link}">
          <origin rpy="0 0 0" xyz="0 0 0"/>
        </link>

        <!-- sensor -->
        <xacro:property name="calibration" value="${xacro.load_yaml('$(arg config_dir)/sensor_kit_calibration.yaml')}"/>

        <!-- lidar -->
        <xacro:VLS-128 parent="sensor_kit_base_link" name="rs_helios_top" topic="/points_raw" hz="10" samples="220" gpu="$(arg gpu)">
          <origin
            xyz="${calibration['sensor_kit_base_link']['rs_helios_top_base_link']['x']}
                 ${calibration['sensor_kit_base_link']['rs_helios_top_base_link']['y']}
                 ${calibration['sensor_kit_base_link']['rs_helios_top_base_link']['z']}"
            rpy="${calibration['sensor_kit_base_link']['rs_helios_top_base_link']['roll']}
                 ${calibration['sensor_kit_base_link']['rs_helios_top_base_link']['pitch']}
                 ${calibration['sensor_kit_base_link']['rs_helios_top_base_link']['yaw']}"
          />
        </xacro:VLS-128>
        <xacro:VLP-16 parent="sensor_kit_base_link" name="rs_bpearl_front" topic="/points_raw" hz="10" samples="220" gpu="$(arg gpu)">
          <origin
            xyz="${calibration['sensor_kit_base_link']['rs_bpearl_front_base_link']['x']}
                 ${calibration['sensor_kit_base_link']['rs_bpearl_front_base_link']['y']}
                 ${calibration['sensor_kit_base_link']['rs_bpearl_front_base_link']['z']}"
            rpy="${calibration['sensor_kit_base_link']['rs_bpearl_front_base_link']['roll']}
                 ${calibration['sensor_kit_base_link']['rs_bpearl_front_base_link']['pitch']}
                 ${calibration['sensor_kit_base_link']['rs_bpearl_front_base_link']['yaw']}"
          />
        </xacro:VLP-16>

        <!-- camera -->
        <xacro:monocular_camera_macro
          name="camera0/camera"
          parent="sensor_kit_base_link"
          namespace=""
          x="${calibration['sensor_kit_base_link']['camera0/camera_link']['x']}"
          y="${calibration['sensor_kit_base_link']['camera0/camera_link']['y']}"
          z="${calibration['sensor_kit_base_link']['camera0/camera_link']['z']}"
          roll="${calibration['sensor_kit_base_link']['camera0/camera_link']['roll']}"
          pitch="${calibration['sensor_kit_base_link']['camera0/camera_link']['pitch']}"
          yaw="${calibration['sensor_kit_base_link']['camera0/camera_link']['yaw']}"
          fps="30"
          width="800"
          height="400"
          fov="1.3"
        />

        <!-- gnss -->
        <xacro:imu_macro
          name="gnss"
          parent="sensor_kit_base_link"
          namespace=""
          x="${calibration['sensor_kit_base_link']['gnss_link']['x']}"
          y="${calibration['sensor_kit_base_link']['gnss_link']['y']}"
          z="${calibration['sensor_kit_base_link']['gnss_link']['z']}"
          roll="${calibration['sensor_kit_base_link']['gnss_link']['roll']}"
          pitch="${calibration['sensor_kit_base_link']['gnss_link']['pitch']}"
          yaw="${calibration['sensor_kit_base_link']['gnss_link']['yaw']}"
          fps="100"
        />

      </xacro:macro>
    </robot>
    ```

### 4. sensors.xacro

Resolves the positions of sensors with `base_link` as the parent frame and defines the positions and orientations based on `sensors_calibration.yaml` in `individual_params`.

The default [sensors.xacro](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_description/urdf/sensors.xacro)
included in [sample_sensor_kit_launch](https://github.com/autowarefoundation/sample_sensor_kit_launch/tree/main) file for sample_sensor_kit is given below.

??? note "sensors.xacro for sample_sensor_kit_launch"

    ```yaml
    <?xml version="1.0"?>
    <robot name="vehicle" xmlns:xacro="http://ros.org/wiki/xacro">
      <xacro:arg name="config_dir" default="$(find sample_sensor_kit_description)/config"/>
      <xacro:property name="calibration" value="${xacro.load_yaml('$(arg config_dir)/sensors_calibration.yaml')}"/>

      <!-- sensor kit -->
      <xacro:include filename="sensor_kit.xacro"/>
      <xacro:sensor_kit_macro
        parent="base_link"
        x="${calibration['base_link']['sensor_kit_base_link']['x']}"
        y="${calibration['base_link']['sensor_kit_base_link']['y']}"
        z="${calibration['base_link']['sensor_kit_base_link']['z']}"
        roll="${calibration['base_link']['sensor_kit_base_link']['roll']}"
        pitch="${calibration['base_link']['sensor_kit_base_link']['pitch']}"
        yaw="${calibration['base_link']['sensor_kit_base_link']['yaw']}"
      />
      .
      .
      .
    </robot>
    ```

The tutorial_vehicle file's [sensors.xacro](https://github.com/leo-drive/tutorial_vehicle_sensor_kit/blob/main/tutorial_vehicle_sensor_kit_description/urdf/sensors.xacro) included in [tutorial_vehicle_sensor_kit](https://github.com/leo-drive/tutorial_vehicle_sensor_kit/tree/main) file is given below.

??? note "sensors.xacro for tutorial_vehicle_sensor_kit_launch"

    ```yaml
    <?xml version="1.0"?>
    <robot name="vehicle" xmlns:xacro="http://ros.org/wiki/xacro">
      <xacro:arg name="config_dir" default="$(find sample_sensor_kit_description)/config"/>
      <xacro:property name="calibration" value="${xacro.load_yaml('$(arg config_dir)/sensors_calibration.yaml')}"/>

      <!-- sensor kit -->
      <xacro:include filename="sensor_kit.xacro"/>
      <xacro:sensor_kit_macro
        parent="base_link"
        x="${calibration['base_link']['sensor_kit_base_link']['x']}"
        y="${calibration['base_link']['sensor_kit_base_link']['y']}"
        z="${calibration['base_link']['sensor_kit_base_link']['z']}"
        roll="${calibration['base_link']['sensor_kit_base_link']['roll']}"
        pitch="${calibration['base_link']['sensor_kit_base_link']['pitch']}"
        yaw="${calibration['base_link']['sensor_kit_base_link']['yaw']}"
      />
    </robot>
    ```

## 2. VEHICLE-ID_sensor_kit_launch

VEHICLE-ID_sensor_kit_launch is where the launch files related to sensor startup are stored.

### Folder Structure

The folder structure of forked `VEHICLE-ID_sensor_kit_launch` repository should be like this:

```diff
VEHICLE-ID_sensor_kit_launch/
      ├─ config/
      │     ├─ diagnostic_aggregator/
+     │     │     └─ sensor_kit.param.yaml
      │     └─ dummy_diag_publisher/
+     │           └─ sensor_kit.param.yaml
      ├─ data/
+     │     └─ traffic_light_camera.yaml
      └─ launch/
+           ├─ camera.launch.xml
+           ├─ gnss.launch.xml
+           ├─ imu.launch.xml
+           ├─ lidar.launch.xml
+           ├─ pointcloud_preprocessor.launch.py
+           └─ sensing.launch.xml
```

In `VEHICLE-ID_sensor_kit_launch`, the following configurations are set:

- diagnostic_aggregator/sensor_kit.param.yaml
- traffic_light_camera.yaml
- camera.launch.xml
- gnss.launch.xml
- imu.launch.xml
- lidar.launch.xml
- pointcloud_preprocessor.launch.py
- sensing.launch.xml

### 1. diagnostic_aggregator

The term "diagnostic aggregator" is a term used within ROS 2 and is used to monitor and diagnose the health of systems.
The aggregator_node will load analyzers to store and process the diagnostic data.
Each analyzer inherits from the base class diagnostic_aggregator::Analyzer.
Analyzers must be in packages that depend directly on pluginlib and [diagnostic_aggregator](http://wiki.ros.org/diagnostic_aggregator).

The required parameter file for the diagnostic_aggregator package is `sensor_kit.param.yaml`. The original
[sensor_kit.param.yaml](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_launch/config/diagnostic_aggregator/sensor_kit.param.yaml) file is given below.

??? note "diagnostic_aggregator/sensor_kit.param.yaml for sample_sensor_kit_launch"

    ```yaml
    /**:
      ros__parameters:
        sensing:
          type: diagnostic_aggregator/AnalyzerGroup
          path: sensing
          analyzers:
            lidar:
              type: diagnostic_aggregator/AnalyzerGroup
              path: lidar
              analyzers:
                velodyne:
                  type: diagnostic_aggregator/AnalyzerGroup
                  path: velodyne
                  analyzers:
                    health_monitoring:
                      type: diagnostic_aggregator/AnalyzerGroup
                      path: health_monitoring
                      analyzers:
                        connection:
                          type: diagnostic_aggregator/GenericAnalyzer
                          path: connection
                          contains: [": velodyne_connection"]
                          timeout: 3.0

                        temperature:
                          type: diagnostic_aggregator/GenericAnalyzer
                          path: temperature
                          contains: [": velodyne_temperature"]
                          timeout: 3.0

                        rpm:
                          type: diagnostic_aggregator/GenericAnalyzer
                          path: rpm
                          contains: [": velodyne_rpm"]
                          timeout: 3.0
                          .
                          .
                          .
    ```

As seen above, this yaml file contains parameters such as temperature, rpm, etc. of the sensors.
The outputs of these parameters indicate whether the sensors are working properly.
The tutorial_vehicle file is [diagnostic_aggregator/sensor_kit.param.yaml](https://github.com/leo-drive/tutorial_vehicle_sensor_kit/blob/main/tutorial_vehicle_sensor_kit_launch/config/diagnostic_aggregator/sensor_kit.param.yaml) included in [tutorial_vehicle_sensor_kit](https://github.com/leo-drive/tutorial_vehicle_sensor_kit/tree/main) and given below.

??? note "diagnostic_aggregator/sensor_kit.param.yaml for tutorial_vehicle_sensor_kit_launch"

    ```yaml
    /**:
      ros__parameters:
            gnss:
              type: diagnostic_aggregator/AnalyzerGroup
              path: gnss
              analyzers:
                health_monitoring:
                  type: diagnostic_aggregator/AnalyzerGroup
                  path: health_monitoring
                  analyzers:
                    connection:
                      type: diagnostic_aggregator/GenericAnalyzer
                      path: connection
                      contains: [": gnss_connection"]
                      timeout: 3.0

                    data:
                      type: diagnostic_aggregator/GenericAnalyzer
                      path: data
                      contains: [": gnss_data"]
                      timeout: 3.0

                    antenna:
                      type: diagnostic_aggregator/GenericAnalyzer
                      path: antenna
                      contains: [": gnss_antenna"]
                      timeout: 3.0

                    tx_usage:
                      type: diagnostic_aggregator/GenericAnalyzer
                      path: tx_usage
                      contains: [": gnss_tx_usage"]
                      timeout: 3.0

                    spoofing:
                      type: diagnostic_aggregator/GenericAnalyzer
                      path: spoofing
                      contains: [": gnss_spoofing"]
                      timeout: 3.0

                    jamming:
                      type: diagnostic_aggregator/GenericAnalyzer
                      path: jamming
                      contains: [": gnss_jamming"]
                      timeout: 3.0

                    fix_topic_status:
                      type: diagnostic_aggregator/GenericAnalyzer
                      path: fix_topic_status
                      contains: [": fix topic status"]
                      timeout: 3.0
    ```

### 2. dummy_diag_publisher

This package outputs a dummy diagnostic data for debugging and developing.

The required parameter file for the dummy_diag_publisher package is `sensor_kit.param.yaml`. The original file is
[sensor_kit.param.yaml](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_launch/config/dummy_diag_publisher/sensor_kit.param.yaml)
included in [sample_sensor_kit_launch](https://github.com/autowarefoundation/sample_sensor_kit_launch/tree/main) and given below.

??? note "dummy_diag_publisher/sensor_kit.param.yaml for sample_sensor_kit_launch"

    ```yaml
    # Description:
    #   required_diags:
    #     <name>: {is_active: <is_active>, status: <status>}
    #   name: diag name
    #   is_active: Force update or not
    #   status: diag status set by dummy diag publisher "OK, Warn, Error, Stale"
    #
    # Note:
    #
    # default values are:
    #   is_active: "true"
    #   status: "OK"
    ---
    /**:
      ros__parameters:
        required_diags:
          dummy_diag_empty: default
    ```

!!! Note

    Since this package is for debug, the parameters do not need to be edited.

### 3. traffic_light_camera.yaml

!!! Warning

    Under Construction

### 4. camera.launch.xml

!!! Warning

    Under Construction

### 5. gnss.launch.xml

gnss.launch.xml is the launch file related to starting the GNSS driver.
The original [gnss.launch.xml](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_launch/launch/gnss.launch.xml) file is given below.

??? note "gnss.launch.xml for sample_sensor_kit_launch"

    ```xml
    <launch>
      <arg name="launch_driver" default="true"/>
      <arg name="gnss_receiver" default="ublox" description="ublox(default) or septentrio"/>

      <group>
        <push-ros-namespace namespace="gnss"/>

        <!-- Switch topic name -->
        <let name="navsatfix_topic_name" value="ublox/nav_sat_fix" if="$(eval &quot;'$(var gnss_receiver)'=='ublox'&quot;)"/>
        <let name="navsatfix_topic_name" value="septentrio/nav_sat_fix" if="$(eval &quot;'$(var gnss_receiver)'=='septentrio'&quot;)"/>
        <let name="orientation_topic_name" value="/autoware_orientation"/>

        <!-- Ublox Driver -->
        <group if="$(eval &quot;'$(var gnss_receiver)'=='ublox'&quot;)">
          <node pkg="ublox_gps" name="ublox" exec="ublox_gps_node" if="$(var launch_driver)" respawn="true" respawn_delay="1.0">
            <remap from="~/fix" to="~/nav_sat_fix"/>
            <param from="$(find-pkg-share ublox_gps)/c94_f9p_rover.yaml"/>
          </node>
        </group>

        <!-- Septentrio GNSS Driver -->
        <group if="$(eval &quot;'$(var launch_driver)' and '$(var gnss_receiver)'=='septentrio'&quot;)">
          <include file="$(find-pkg-share septentrio_gnss_driver)/launch/mosaic_x5_rover.launch.xml"/>
        </group>

        <!-- NavSatFix to MGRS Pose -->
        <include file="$(find-pkg-share gnss_poser)/launch/gnss_poser.launch.xml">
          <arg name="input_topic_fix" value="$(var navsatfix_topic_name)"/>
          <arg name="input_topic_orientation" value="$(var orientation_topic_name)"/>

          <arg name="output_topic_gnss_pose" value="pose"/>
          <arg name="output_topic_gnss_pose_cov" value="pose_with_covariance"/>
          <arg name="output_topic_gnss_fixed" value="fixed"/>

          <arg name="use_gnss_ins_orientation" value="true"/>

          <arg name="gnss_frame" value="gnss_link"/>
        </include>
      </group>
    </launch>
    ```

The tutorial_vehicle's [gnss.launch.xml](https://github.com/leo-drive/tutorial_vehicle_sensor_kit/blob/main/tutorial_vehicle_sensor_kit_launch/launch/gnss.launch.xml) file is given below.

??? note "gnss.launch.xml for tutorial_vehicle_sensor_kit_launch"

    ```xml
    <launch>
      <arg name="launch_driver" default="true"/>
      <arg name="coordinate_system" default="1" description="0:UTM, 1:MGRS, 2:PLANE"/>
      <arg name="vehicle_id" default="default" description="vehicle specific ID"/>
      <arg name="height_system" default="1" description="0:Orthometric Height 1:Ellipsoid Height"/>
      <arg name="gnss_pose_pub_method" default="0" description="0: Instant Value 1: Average Value 2: Median Value"/>

      <group>
        <push-ros-namespace namespace="gnss"/>
        <!-- Switch topic name -->
        <let name="navsatfix_topic_name" value="clap/ros/gps_nav_sat_fix" />

        <let name="orientation_topic_name" value="clap/autoware_orientation"/>

        <group if="$(var launch_driver)">
          <!-- Clap B7 Driver -->
          <node pkg="clap_b7_driver" exec="clap_b7_driver_node" name="clap_b7_driver" output="screen">
            <param from="$(find-pkg-share clap_b7_driver)/config/clap_b7_driver.param.yaml"/>
          </node>
          <!-- ntrip Client -->
          <include file="$(find-pkg-share ntrip_client_ros)/launch/ntrip_client_ros.launch.py"/>
        </group>

        <!-- NavSatFix to MGRS Pose -->
        <include file="$(find-pkg-share gnss_poser)/launch/gnss_poser.launch.xml">
          <arg name="input_topic_fix" value="$(var navsatfix_topic_name)"/>
          <arg name="input_topic_orientation" value="$(var orientation_topic_name)"/>

          <arg name="output_topic_gnss_pose" value="pose"/>
          <arg name="output_topic_gnss_pose_cov" value="pose_with_covariance"/>
          <arg name="output_topic_gnss_fixed" value="fixed"/>

          <arg name="base_frame" value="base_link"/>
          <arg name="gnss_base_frame" value="gnss_ins_base_link"/>
          <arg name="gnss_frame" value="GNSS_INS/gnss_ins_link"/>
          <arg name="map_frame" value="map"/>

          <arg name="coordinate_system" value="$(var coordinate_system)"/>
          <arg name="use_gnss_ins_orientation" value="true"/>
          <arg name="height_system" value="$(var height_system)"/>

          <arg name="buff_epoch" value="4"/>
          <arg name="gnss_pose_pub_method" value="$(var gnss_pose_pub_method)"/>
        </include>
      </group>

      <group>
        <push-ros-namespace namespace="imu"/>
        <include file="$(find-pkg-share imu_corrector)/launch/imu_corrector.launch.xml">
          <arg name="input_topic" value="/sensing/gnss/clap/ros/imu"/>
          <arg name="output_topic" value="imu_data"/>
          <arg name="param_file" value="$(find-pkg-share individual_params)/config/$(var vehicle_id)/tutorial_vehicle_sensor_kit/imu_corrector.param.yaml"/>
        </include>
      </group>
    </launch>
    ```

!!! Warning

    Under Construction

### 6. imu.launch.xml

imu.launch.xml is the launch file related to starting the IMU driver.
The default [imu.launch.xml](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_launch/launch/imu.launch.xml) file for sample_sensor_kit is given below.

??? note "imu.launch.xml for sample_sensor_kit_launch"

    ```xml
    <launch>
    <arg name="launch_driver" default="true"/>

    <group>
    <push-ros-namespace namespace="imu"/>

    <group>
    <push-ros-namespace namespace="tamagawa"/>
    <node pkg="tamagawa_imu_driver" name="tag_serial_driver" exec="tag_serial_driver" if="$(var launch_driver)">
    <remap from="imu/data_raw" to="imu_raw"/>
    <param name="port" value="/dev/imu"/>
    <param name="imu_frame_id" value="tamagawa/imu_link"/>
    </node>
    </group>

    <arg name="imu_raw_name" default="tamagawa/imu_raw"/>
    <arg name="imu_corrector_param_file" default="$(find-pkg-share individual_params)/config/$(var vehicle_id)/sample_sensor_kit/imu_corrector.param.yaml"/>
    <include file="$(find-pkg-share imu_corrector)/launch/imu_corrector.launch.xml">
    <arg name="input_topic" value="$(var imu_raw_name)"/>
    <arg name="output_topic" value="imu_data"/>
    <arg name="param_file" value="$(var imu_corrector_param_file)"/>
    </include>

    <include file="$(find-pkg-share imu_corrector)/launch/gyro_bias_estimator.launch.xml">
    <arg name="input_imu_raw" value="$(var imu_raw_name)"/>
    <arg name="input_twist" value="/sensing/vehicle_velocity_converter/twist_with_covariance"/>
    <arg name="imu_corrector_param_file" value="$(var imu_corrector_param_file)"/>
    </include>
    </group>
    </launch>
    ```

!!! Warning

    Since no external IMU is used in the tutorial vehicle, the parameters do not need to be edited.

!!! Warning

    Under Construction

### 7. lidar.launch.xml

lidar.launch.xml is the launch file related to starting the LiDAR driver.

The original file is [lidar.launch.xml](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_launch/launch/lidar.launch.xml)
included in [sample_sensor_kit_launch](https://github.com/autowarefoundation/sample_sensor_kit_launch/tree/main) and given below.

??? note "lidar.launch.xml for sample_sensor_kit_launch"

    ```xml
    <launch>
      <arg name="launch_driver" default="true"/>
      <arg name="host_ip" default="192.168.1.10"/>
      <arg name="use_concat_filter" default="true"/>
      <arg name="vehicle_id" default="$(env VEHICLE_ID default)"/>
      <arg name="vehicle_mirror_param_file"/>
      <arg name="use_pointcloud_container" default="false" description="launch pointcloud container"/>
      <arg name="pointcloud_container_name" default="pointcloud_container"/>

      <group>
        <push-ros-namespace namespace="lidar"/>

        <group>
          <push-ros-namespace namespace="top"/>
          <include file="$(find-pkg-share common_sensor_launch)/launch/velodyne_VLS128.launch.xml">
            <arg name="max_range" value="250.0"/>
            <arg name="sensor_frame" value="velodyne_top"/>
            <arg name="sensor_ip" value="192.168.1.201"/>
            <arg name="host_ip" value="$(var host_ip)"/>
            <arg name="data_port" value="2368"/>
            <arg name="scan_phase" value="300.0"/>
            <arg name="launch_driver" value="$(var launch_driver)"/>
            <arg name="vehicle_mirror_param_file" value="$(var vehicle_mirror_param_file)"/>
            <arg name="use_pointcloud_container" value="$(var use_pointcloud_container)"/>
            <arg name="container_name" value="$(var pointcloud_container_name)"/>
          </include>
        </group>

        <group>
          <push-ros-namespace namespace="left"/>
          <include file="$(find-pkg-share common_sensor_launch)/launch/velodyne_VLP16.launch.xml">
            <arg name="max_range" value="5.0"/>
            <arg name="sensor_frame" value="velodyne_left"/>
            <arg name="sensor_ip" value="192.168.1.202"/>
            <arg name="host_ip" value="$(var host_ip)"/>
            <arg name="data_port" value="2369"/>
            <arg name="scan_phase" value="180.0"/>
            <arg name="cloud_min_angle" value="300"/>
            <arg name="cloud_max_angle" value="60"/>
            <arg name="launch_driver" value="$(var launch_driver)"/>
            <arg name="vehicle_mirror_param_file" value="$(var vehicle_mirror_param_file)"/>
            <arg name="use_pointcloud_container" value="$(var use_pointcloud_container)"/>
            <arg name="container_name" value="$(var pointcloud_container_name)"/>
          </include>
        </group>

        <group>
          <push-ros-namespace namespace="right"/>
          <include file="$(find-pkg-share common_sensor_launch)/launch/velodyne_VLP16.launch.xml">
            <arg name="max_range" value="5.0"/>
            <arg name="sensor_frame" value="velodyne_right"/>
            <arg name="sensor_ip" value="192.168.1.203"/>
            <arg name="host_ip" value="$(var host_ip)"/>
            <arg name="data_port" value="2370"/>
            <arg name="scan_phase" value="180.0"/>
            <arg name="cloud_min_angle" value="300"/>
            <arg name="cloud_max_angle" value="60"/>
            <arg name="launch_driver" value="$(var launch_driver)"/>
            <arg name="vehicle_mirror_param_file" value="$(var vehicle_mirror_param_file)"/>
            <arg name="use_pointcloud_container" value="$(var use_pointcloud_container)"/>
            <arg name="container_name" value="$(var pointcloud_container_name)"/>
          </include>
        </group>

        <group>
          <push-ros-namespace namespace="rear"/>
          <include file="$(find-pkg-share common_sensor_launch)/launch/velodyne_VLP16.launch.xml">
            <arg name="max_range" value="1.5"/>
            <arg name="sensor_frame" value="velodyne_rear"/>
            <arg name="sensor_ip" value="192.168.1.204"/>
            <arg name="host_ip" value="$(var host_ip)"/>
            <arg name="data_port" value="2371"/>
            <arg name="scan_phase" value="180.0"/>
            <arg name="cloud_min_angle" value="300"/>
            <arg name="cloud_max_angle" value="60"/>
            <arg name="launch_driver" value="$(var launch_driver)"/>
            <arg name="vehicle_mirror_param_file" value="$(var vehicle_mirror_param_file)"/>
            <arg name="use_pointcloud_container" value="$(var use_pointcloud_container)"/>
            <arg name="container_name" value="$(var pointcloud_container_name)"/>
          </include>
        </group>

        <include file="$(find-pkg-share sample_sensor_kit_launch)/launch/pointcloud_preprocessor.launch.py">
          <arg name="base_frame" value="base_link"/>
          <arg name="use_intra_process" value="true"/>
          <arg name="use_multithread" value="true"/>
          <arg name="use_pointcloud_container" value="$(var use_pointcloud_container)"/>
          <arg name="container_name" value="$(var pointcloud_container_name)"/>
        </include>
      </group>
    </launch>
    ```

!!! Warning

    Under Construction

### 8. pointcloud_preprocessor.launch.py

!!! Warning

    Under Construction

### 9. sensing.launch.xml

!!! Warning

    Under Construction

## 3. common_sensor_launch

!!! Warning

    Under Construction
