# Creating a sensor model for Autoware

## Introduction

This page introduces the following packages for sensor the model:

1. `common_sensor_launch`
2. `<YOUR-VEHICLE-NAME>_sensor_kit_description`
3. `<YOUR-VEHICLE-NAME>_sensor_kit_launch`

So,
we forked our sensor model
at [creating autoware repositories](../../creating-your-autoware-meta-repository/creating-autoware-meta-repository.md) page step,
(For example,
we created [tutorial_vehicle_sensor_kit_launch](https://github.com/leo-drive/tutorial_vehicle_sensor_kit_launch) for our documentation vehicle at this step)
please be sure `<YOUR-VEHICLE-NAME>_sensor_kit_launch` repository is included in the following directory:

```diff
<YOUR-OWN-AUTOWARE-DIR>/
  └─ src/
       └─ sensor_kit/
            └─ <YOUR-VEHICLE-NAME>_sensor_kit_launch/
                 ├─ common_sensor_launch/
                 ├─ <YOUR-VEHICLE-NAME>_sensor_kit_description/
                 └─ <YOUR-VEHICLE-NAME>_sensor_kit_launch/
```

If your forked repository doesn't include in the correct structure like above,
please add your forked sensor_kit repo to autoware.repos file
and run `vcs import src < autoware.repos` command on your terminal
to import new included repositories at autoware.repos file.

Now, we are ready to modify the following sensor model packages for our vehicle.
Firstly, we need to rename the description and launch packages:

```diff
<YOUR-VEHICLE-NAME>_sensor_kit_launch/
  ├─ common_sensor_launch/
- ├─ sample_sensor_kit_description/
+ ├─ <YOUR-VEHICLE-NAME>_sensor_kit_description/
- └─ sample_sensor_kit_launch/
+ └─ <YOUR-VEHICLE-NAME>_sensor_kit_launch/
```

After that,
we will change our package names at `package.xml` file and `CmakeLists.txt` file at
`sample_sensor_kit_description` and `sample_sensor_kit_launch` packages.
So, open `package.xml` file and `CmakeLists.txt` file with any text editor or ide that you prefer.

First Step: You need to change `<name>` attribute at `package.xml` file.

```diff
<package format="3">
- <name>sample_sensor_kit_description</name>
+ <name><YOUR-VEHICLE-NAME>_sensor_kit_description</name>
  <version>0.1.0</version>
  <description>The sensor_kit_description package</description>
  ...
  ...
```

Second Step: You need to change `project()` method at `CmakeList.txt` file.

```diff
  cmake_minimum_required(VERSION 3.5)
- project(sample_sensor_kit_description)
+ project(<YOUR-VEHICLE-NAME>_sensor_kit_description)

  find_package(ament_cmake_auto REQUIRED)
...
...
```

You need to apply these two steps for `<YOUR-VEHICLE-NAME>_sensor_kit_description`and `<YOUR-VEHICLE-NAME>_sensor_kit_launch`
ROS 2 packages.
After the completing of changing package names, we need to build these packages:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <YOUR-VEHICLE-NAME>_sensor_kit_description <YOUR-VEHICLE-NAME>_sensor_kit_launch
```

## Sensor description

The main purpose of this package is to describe the sensor frame IDs,
calibration parameters of all sensors, and their links with urdf files.

The folder structure of sensor_kit_description package is:

```diff
<YOUR-VEHICLE-NAME>_sensor_kit_description/
   ├─ config/
   │     ├─ sensor_kit_calibration.yaml
   │     └─ sensors_calibration.yaml
   └─ urdf/
         ├─ sensor_kit.xacro
         └─ sensors.xacro
```

Now, we will modify these files according to our sensor design.

### sensor_kit_calibration.yaml

This file defines the mounting positions and orientations of sensors with `sensor_kit_base_link` as the parent frame.
We can assume `sensor_kit_base_link` frame is bottom of your main Lidar sensor.
We must create this file with euler format as [x, y, z, roll, pitch, yaw].
Also, we will set these values with "0" until the [calibration steps](../calibrating-sensors).

We will define new frames for this file, and we will connect them `.xacro` files.
We recommend naming as if your lidar sensor frame as "velodyne_top",
you can add "\_base_link" to our calibration .yaml file. W

So, the sample file must be like:

```yaml
sensor_kit_base_link:
  velodyne_top_base_link:
    x: 0.000000
    y: 0.000000
    z: 0.000000
    roll: 0.000000
    pitch: 0.000000
    yaw: 0.000000
  camera0/camera_link:
    x: 0.000000
    y: 0.000000
    z: 0.000000
    roll: 0.000000
    pitch: 0.000000
    yaw: 0.000000
  ...
  ...
```

This file for `tutorial_vehicle` was created for one camera, two lidars and one GNSS/INS sensors.

??? note "`sensor_kit_calibration.yaml` for tutorial_vehicle_sensor_kit_description"

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
      GNSS_INS/gnss_ins_link: # GNSS/INS
        x: 0.0
        y: 0.0
        z: 0.0
        roll: 0.0
        pitch: 0.0
        yaw: 0.0
    ```

### sensors_calibration.yaml

This file defines the mounting positions and orientations of `sensor_kit_base_link` (child frame)
with `base_link` as the parent frame.
At Autoware, `base_link` is on projection of the rear-axle center onto the ground surface.
For more information,
you can check [vehicle dimension](../../../../design/autoware-interfaces/components/vehicle-dimensions.md) page.
You can use CAD values for this, but we will fill the values with `0` for now.

```yaml
base_link:
  sensor_kit_base_link:
    x: 0.000000
    y: 0.000000
    z: 0.000000
    roll: 0.000000
    pitch: 0.000000
    yaw: 0.000000
```

Now, we are ready to implement .xacro files.
These files provide linking our sensor frames and adding sensor urdf files

### sensor_kit.xacro

We will add our sensors and remove unnecessary xacros from this file.
For example,
we want
to add our lidar sensor with `velodyne_top` frame,
we will add the following xacro to our sensor_kit.xacro file.
Please add your sensors to this file and remove unnecessary sensor's xacros.

```xml
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
    </xacro:VLS-128>
```

Here is the sample xacro file for tutorial_vehicle with one camera, two lidars and one GNSS/INS sensors.

??? note "`sensor_kit.xacro` for tutorial_vehicle_sensor_kit_description"

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

### sensors.xacro

This files links our sensor_kit main frame (`sensor_kit_base_link`) to base_link.
Also, you have sensors which will be calibrated directly to base_link, you can add it to here.

Here is the sensors.xacro file for sample_sensor_kit_description package:
(velodyne_rear transformation is directly used with base_link)

```xml
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

  <!-- embedded sensors -->
  <xacro:include filename="$(find velodyne_description)/urdf/VLP-16.urdf.xacro"/>
  <xacro:VLP-16 parent="base_link" name="velodyne_rear" topic="velodyne_rear/velodyne_points" hz="10" samples="220" gpu="false">
    <origin
      xyz="${calibration['base_link']['velodyne_rear_base_link']['x']}
           ${calibration['base_link']['velodyne_rear_base_link']['y']}
           ${calibration['base_link']['velodyne_rear_base_link']['z']}"
      rpy="${calibration['base_link']['velodyne_rear_base_link']['roll']}
           ${calibration['base_link']['velodyne_rear_base_link']['pitch']}
           ${calibration['base_link']['velodyne_rear_base_link']['yaw']}"
    />
  </xacro:VLP-16>
</robot>
```

At out tutorial vehicle,
there is no directly sensor transformation for base_link,
thus our sensors.xacro file includes only `base_link` and `sensor_kit_base_link` link.

??? note "`sensors.xacro` for tutorial_vehicle_sensor_kit_description"

    ```xml
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

After the completing `sensor_kit_calibration.yaml`, `sensors_calibration.yaml`, `sensor_kit.xacro`
and `sensors.xacro` file, our sensor description package is finished,
we will continue with modifying `<YOUR-VEHICLE-NAME>_sensor_kit_launch` package.

## Sensor launch

At this package (`<YOUR-VEHICLE-NAME>_sensor_kit_launch`),
we will launch our sensors and their pipelines.
So, we will also use `common_sensor_launch` package for launching the lidar sensing pipeline.
This image below demonstrates our sensor pipeline, which we will construct in this section.

<figure markdown>
  ![sensor_launch_design](images/sensor_launch_design.svg){ align=center }
  <figcaption>
    Sample Launch workflow for sensing design.
  </figcaption>
</figure>

The `<YOUR-VEHICLE-NAME>_sensor_kit_launch` package folder structure like this:

```diff
<YOUR-VEHICLE-NAME>_sensor_kit_launch/
      ├─ config/
      ├─ data/
      └─ launch/
+           ├─ camera.launch.xml
+           ├─ gnss.launch.xml
+           ├─ imu.launch.xml
+           ├─ lidar.launch.xml
+           ├─ pointcloud_preprocessor.launch.py
+           └─ sensing.launch.xml
```

So,
we will modify the launch files
which located the `launch` folder for launching and manipulating our sensors.
The main launch file is `sensing.launch.xml`.
This launch file launches other sensing launch files.
The current autoware sensing launch files design for `sensor_kit_launch` package is the diagram below.

<figure markdown>
  ![sensing_launch_files_design](images/sensing_launch_files.svg){ align=center }
  <figcaption>
    Launch file flows over sensing.launch.xml launch file.
  </figcaption>
</figure>

The `sensing.launch.xml` also launches `vehicle_velocity_converter` package
for converting `autoware_auto_vehicle_msgs::msg::VelocityReport` message to `geometry_msgs::msg::TwistWithCovarianceStamped` for gyro_odometer node.
So,
be sure
your vehicle_interface publishes `/vehicle/status/velocity_status` topic with `autoware_auto_vehicle_msgs::msg::VelocityReport` type,
or you must update `input_vehicle_velocity_topic` at `sensing.launch.xml`.

```diff
    ...
    <include file="$(find-pkg-share vehicle_velocity_converter)/launch/vehicle_velocity_converter.launch.xml">
-     <arg name="input_vehicle_velocity_topic" value="/vehicle/status/velocity_status"/>
+     <arg name="input_vehicle_velocity_topic" value="<YOUR-VELOCITY-STATUS-TOPIC>"/>
      <arg name="output_twist_with_covariance" value="/sensing/vehicle_velocity_converter/twist_with_covariance"/>
    </include>
    ...
```

### Lidar Launching

Let's
start with modifying `lidar.launch.xml` file for launching our lidar sensor driver with autoware.
Please check supported lidar sensors over the nebula driver in the [GitHub repository](https://github.com/tier4/nebula).

If you are using [Velodyne Lidar](https://velodynelidar.com/) sensor,
you can use the [sample_sensor_kit_launch template](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_launch/launch/lidar.launch.xml),
but you need to update `sensor_id`, `data_port`, `sensor_frame` and other necessary changes
(`max_range`, `scan_phase`, etc.).

```diff
    <group>
-     <push-ros-namespace namespace="left"/>
+     <push-ros-namespace namespace="<YOUR-SENSOR-NAMESPACE>"/>
      <include file="$(find-pkg-share common_sensor_launch)/launch/velodyne_VLP16.launch.xml">
        <arg name="max_range" value="5.0"/>
-       <arg name="sensor_frame" value="velodyne_left"/>
+       <arg name="sensor_frame" value="<YOUR-SENSOR-FRAME>"/>
-       <arg name="sensor_ip" value="192.168.1.202"/>
+       <arg name="sensor_ip" value="<YOUR-SENSOR-IP>"/>
        <arg name="host_ip" value="$(var host_ip)"/>
-       <arg name="data_port" value="2369"/>
+       <arg name="data_port" value=<YOUR-DATA-PORT>/>
        <arg name="scan_phase" value="180.0"/>
        <arg name="cloud_min_angle" value="300"/>
        <arg name="cloud_max_angle" value="60"/>
        <arg name="launch_driver" value="$(var launch_driver)"/>
        <arg name="vehicle_mirror_param_file" value="$(var vehicle_mirror_param_file)"/>
        <arg name="use_pointcloud_container" value="$(var use_pointcloud_container)"/>
        <arg name="container_name" value="$(var pointcloud_container_name)"/>
      </include>
    </group>
```

Please add similar launch groups according to your sensor architecture.
For example, we use Robosense Lidars for our `tutorial_vehicle`,
so the lidar group for Robosense Lidar should be like this structure:

!!! warning

    under construction

The [nebula_node_container.py](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/common_sensor_launch/launch/nebula_node_container.launch.py) creates Lidar pipeline for autoware.
The pointcloud preprocessing pipeline is constructed for each lidar please check [pointcloud_preprocessor](https://github.com/autowarefoundation/autoware.universe/tree/main/sensing/pointcloud_preprocessor) package for filters information.

For example, If you want to change your `outlier_filter` method,
you can modify the pipeline components like this way:

```diff

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
-           plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
-           name="ring_outlier_filter",
+           plugin="pointcloud_preprocessor::DualReturnOutlierFilterComponent",
+           name="dual_return_outlier_filter",
            remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", "outlier_filtered/pointcloud"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )
```

We will use the default pointcloud_preprocessor pipeline for our tutorial_vehicle,
thus we will not modify [nebula_node_container.py](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/common_sensor_launch/launch/nebula_node_container.launch.py).

### Camera Launching

!!! warning

    under construction

### GNSS/INS Launching

We will set up the GNSS/INS sensor launches at `gnss.launch.xml`.
The default GNSS sensor options at [`sample_sensor_kit_launch`](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_launch/launch/gnss.launch.xml) for [u-blox](https://www.u-blox.com/en/)
and [septentrio](https://www.septentrio.com/en) is included in `gnss.launch.xml`,
so If we use other sensors as GNSS/INS receiver, we need to add it here.
Moreover, [gnss_poser](https://github.com/autowarefoundation/autoware.universe/tree/main/sensing/gnss_poser) package launches here,
we will use this package for initial pose of our vehicle but remember,
your sensor_driver must provide [autoware gnss orientation message](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_sensing_msgs/msg/GnssInsOrientationStamped.msg) for this node.
If you are ready with your GNSS/INS driver,
you must set `navsatfix_topic_name` and `orientation_topic_name` variables at this launch file.
For Example, necessary modifications for <YOUR-GNSS-SENSOR> should be like this:

```diff
  ...
- <arg name="gnss_receiver" default="ublox" description="ublox(default) or septentrio"/>
+ <arg name="gnss_receiver" default="<YOUR-GNSS-SENSOR>" description="ublox(default), septentrio or <YOUR-GNSS-SENSOR>"/>

  <group>
    <push-ros-namespace namespace="gnss"/>

    <!-- Switch topic name -->
    <let name="navsatfix_topic_name" value="ublox/nav_sat_fix" if="$(eval &quot;'$(var gnss_receiver)'=='ublox'&quot;)"/>
    <let name="navsatfix_topic_name" value="septentrio/nav_sat_fix" if="$(eval &quot;'$(var gnss_receiver)'=='septentrio'&quot;)"/>
+   <let name="navsatfix_topic_name" value="<YOUR-SENSOR>/nav_sat_fix" if="$(eval &quot;'$(var gnss_receiver)'=='<YOUR-GNSS-SENSOR>'&quot;)"/>
    <let name="orientation_topic_name" value="/autoware_orientation"/>

    ...

+   <!-- YOUR GNSS Driver -->
+   <group if="$(eval &quot;'$(var launch_driver)' and '$(var gnss_receiver)'=='<YOUR-GNSS-SENSOR>'&quot;)">
+     <include file="$(find-pkg-share <YOUR-GNSS-SENSOR-DRIVER-PKG>)/launch/<YOUR-GNSS-SENSOR>.launch.xml"/>
+   </group>
    ...
-   <arg name="gnss_frame" value="gnss_link"/>
+   <arg name="gnss_frame" value="<YOUR-GNSS-SENSOR-FRAME>"/>
    ...
```

Also, you can remove dependencies and unused sensor launch files at `gnss.launch.xml`.
For example,
we will use [Clap B7 sensor](https://en.unicorecomm.com/assets/upload/file/CLAP-B7_Product_Brief_En.pdf) as a GNSS/INS and IMU sensor,
and we will use [nrtip_client_ros](https://github.com/Robeff-Technology/ntrip_client_ros/tree/release/humble) for RTK.
Also, we will add these packages to [autoware.repos](https://github.com/leo-drive/autoware.tutorial_vehicle/blob/main/autoware.repos) file.

```diff
+ sensor_component/external/clap_b7_driver:
+   type: git
+   url: https://github.com/Robeff-Technology/clap_b7_driver.git
+   version: dev/autoware
+ sensor_component/external/ntrip_client_ros :
+   type: git
+   url: https://github.com/Robeff-Technology/ntrip_client_ros.git
+   version: release/humble
```

So,
our `gnss.launch.xml` for tutorial vehicle should be like this file
(Clap B7 includes IMU also, so we will add imu_corrector at this file):

??? note "`gnss.launch.xml` for tutorial_vehicle"

    ```xml
    <launch>
      <arg name="launch_driver" default="true"/>

      <group>
        <push-ros-namespace namespace="gnss"/>

        <!-- Switch topic name -->
        <let name="navsatfix_topic_name" value="/clap/ros/gps_nav_sat_fix"/>
        <let name="orientation_topic_name" value="/clap/autoware_orientation"/>

        <!-- CLAP GNSS Driver -->
        <group if="$(eval &quot;'$(var launch_driver)'">
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

          <arg name="use_gnss_ins_orientation" value="true"/>
          <!-- Please enter your gnss frame here -->
          <arg name="gnss_frame" value="GNSS_INS/gnss_ins_link"/>
        </include>
      </group>

      <!-- IMU corrector -->
      <group>
        <push-ros-namespace namespace="imu"/>
        <include file="$(find-pkg-share imu_corrector)/launch/imu_corrector.launch.xml">
          <arg name="input_topic" value="/sensing/gnss/clap/ros/imu"/>
          <arg name="output_topic" value="imu_data"/>
          <arg name="param_file" value="$(find-pkg-share individual_params)/config/$(var vehicle_id)/robione_sensor_kit/imu_corrector.param.yaml"/>
        </include>
        <include file="$(find-pkg-share imu_corrector)/launch/gyro_bias_estimator.launch.xml">
          <arg name="input_imu_raw" value="/sensing/gnss/clap/ros/imu"/>
          <arg name="input_twist" value="/sensing/vehicle_velocity_converter/twist_with_covariance"/>
          <arg name="imu_corrector_param_file" value="$(find-pkg-share individual_params)/config/$(var vehicle_id)/robione_sensor_kit/imu_corrector.param.yaml"/>
        </include>
      </group>
    </launch>
    ```

### IMU Launching

You can add your IMU sensor launch file at `imu.launch.xml` file.
At the [sample_sensor_kit](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_launch/launch/imu.launch.xml),
there is [Tamagawa IMU sensor](https://mems.tamagawa-seiki.com/en/) used as a IMU sensor.
You can add your IMU driver instead of the Tamagawa IMU driver.
Also,
we will launch [gyro_bias_estimator](https://github.com/autowarefoundation/autoware.universe/tree/main/sensing/imu_corrector#gyro_bias_estimator) and
[imu_corrector](https://github.com/autowarefoundation/autoware.universe/tree/main/sensing/imu_corrector#imu_corrector) at `imu.launch.xml` file.
Please refer these documentations for more information
(We added imu_corrector and gyro_bias_estimator at gnss.launch.xml at tutorial_vehicle,
so we will not create and use `imu.launch.xml` for tutorial_vehicle).
Please don't forget changing `imu_raw_name` argument for describing the raw imu topic.

```diff
  ...
- <arg name="imu_raw_name" default="tamagawa/imu_raw"/>
+ <arg name="imu_raw_name" default="<YOUR-RAW-IMU-TOPIC>"/>
  ...
```
