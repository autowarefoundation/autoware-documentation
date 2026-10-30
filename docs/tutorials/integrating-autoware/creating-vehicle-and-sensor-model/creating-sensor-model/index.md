# Creating a sensor model for Autoware

## Introduction

This page introduces the following packages for the sensor model:

1. `<YOUR-VEHICLE-NAME>_sensor_kit_description`
2. `<YOUR-VEHICLE-NAME>_sensor_kit_launch`

The template for these packages is [`sample_sensor_kit_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/sensor_kit/sample_sensor_kit_launch).
It is in the `sensor_kit/` folder of the `autoware_launch` repository.
(The old `sample_sensor_kit_launch` repository is archived.)
The sensor kit uses shared launch files from two other packages of `autoware_launch`:
[`autoware_sensor_driver_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_universe_launch/autoware_sensor_driver_launch) for the LiDAR drivers
and [`autoware_sensing_launch`](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_universe_launch/autoware_sensing_launch) for the pointcloud preprocessing.
You do not copy these two packages.

Copy `sample_sensor_kit_launch` into your own repository, as we described on the [creating autoware repositories](../../creating-your-autoware-repositories/creating-autoware-repositories.md) page.
For instance,
we created [tutorial_vehicle_sensor_kit_launch](https://github.com/leo-drive/tutorial_vehicle_sensor_kit_launch)
as an implementation example for the said step.
(The `tutorial_vehicle_sensor_kit_launch` repository was made for an earlier Autoware version. It still has a `common_sensor_launch` package, which Autoware 2.0 does not use.)
Please ensure that the <YOUR-VEHICLE-NAME>\_sensor_kit_launch repository is included in Autoware,
following the directory structure below:

```diff
<YOUR-OWN-AUTOWARE-DIR>/
  └─ src/
       └─ sensor_kit/
            └─ <YOUR-VEHICLE-NAME>_sensor_kit_launch/
                 ├─ <YOUR-VEHICLE-NAME>_sensor_kit_description/
                 └─ <YOUR-VEHICLE-NAME>_sensor_kit_launch/
```

If your forked Autoware meta-repository doesn't include `<YOUR-VEHICLE-NAME>_sensor_kit_launch` with the correct folder structure
as shown above,
please add your forked `<YOUR-VEHICLE-NAME>_sensor_kit_launch` repository to the autoware.repos file
and run the `vcs import src < repositories/autoware.repos` command in your terminal
to import the newly included repositories at autoware.repos file.

Now, we are ready to modify the following sensor model packages for our vehicle.
Firstly, we need to rename the description and launch packages.
Autoware finds these packages by the `sensor_model` launch argument:
it loads `<sensor_model>_description` and `<sensor_model>_launch`.
For example, `sensor_model:=sample_sensor_kit` loads `sample_sensor_kit_description` and `sample_sensor_kit_launch`.
So the package names must have this form:

```diff
<YOUR-VEHICLE-NAME>_sensor_kit_launch/
- ├─ sample_sensor_kit_description/
+ ├─ <YOUR-VEHICLE-NAME>_sensor_kit_description/
- └─ sample_sensor_kit_launch/
+ └─ <YOUR-VEHICLE-NAME>_sensor_kit_launch/
```

After that,
we will change our package names in the package.xml file and CMakeLists.txt file of the sample_sensor_kit_description and sample_sensor_kit_launch packages.
So,
open the package.xml file and CMakeLists.txt file with any text editor or IDE of your preference
and perform the following changes:

Change the `<name>` attribute at `package.xml` file:

```diff
<package format="3">
- <name>sample_sensor_kit_description</name>
+ <name><YOUR-VEHICLE-NAME>_sensor_kit_description</name>
  <version>0.53.0</version>
  <description>The sample_sensor_kit_description package</description>
  ...
  ...
```

Change the `project()` method at `CMakeLists.txt` file.

```diff
  cmake_minimum_required(VERSION 3.5)
- project(sample_sensor_kit_description)
+ project(<YOUR-VEHICLE-NAME>_sensor_kit_description)

  find_package(ament_cmake_auto REQUIRED)
...
...
```

Remember to apply the name changes and project method for **BOTH**
`<YOUR-VEHICLE-NAME>_sensor_kit_description`and `<YOUR-VEHICLE-NAME>_sensor_kit_launch` ROS 2 packages.
Once finished, we can proceed to build said packages:

```bash
cd <YOUR-AUTOWARE-DIR>
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to <YOUR-VEHICLE-NAME>_sensor_kit_description <YOUR-VEHICLE-NAME>_sensor_kit_launch
```

## Sensor description

The main purpose of this package is to describe the sensor frame IDs,
calibration parameters of all sensors, and their links with urdf files.

The folder structure of sensor_kit_description package is:

```diff
<YOUR-VEHICLE-NAME>_sensor_kit_description/
   ├─ config/
   │     ├─ imu_corrector.param.yaml
   │     ├─ sensor_kit_calibration.yaml
   │     └─ sensors_calibration.yaml
   └─ urdf/
         ├─ sensor_kit.xacro
         └─ sensors.xacro
```

Autoware reads the calibration files from the `config/` folder of this package.
`autoware.launch.xml` gives the `config_dir` argument `$(find-pkg-share <sensor_model>_description)/config` to `tier4_vehicle_launch`, and the xacro files load the calibration files from this folder.
Earlier Autoware versions read the calibration from a separate `individual_params` package. Autoware 2.0 does not use this package.
`imu_corrector.param.yaml` holds the IMU bias and noise values. `imu.launch.xml` of the sensor kit gives it to the IMU corrector.

Now, we will modify these files according to our sensor design.

### sensor_kit_calibration.yaml

This file defines the mounting positions and orientations of sensors with `sensor_kit_base_link` as the parent frame.
We can assume `sensor_kit_base_link` frame is bottom of your main Lidar sensor.
We must create this file with euler format as [x, y, z, roll, pitch, yaw].
Also, we will set these values with "0" until the [calibration steps](../calibrating-sensors/index.md).

We will define new frames for this file, and we will connect them `.xacro` files.
We recommend naming as if your lidar sensor frame as "velodyne_top",
you can add "\_base_link" to our calibration .yaml file.

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
you can check [vehicle dimension](../../../../design/autoware-architecture-v1/interfaces/components/vehicle-dimensions.md) page.
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
to add our lidar sensor with `velodyne_top` frame from the sensor driver,
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
        <xacro:arg name="config_dir" default="$(find tutorial_vehicle_sensor_kit_description)/config"/>

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

At our tutorial vehicle,
there is no directly sensor transformation for base_link,
thus our sensors.xacro file includes only `base_link` and `sensor_kit_base_link` link.

??? note "`sensors.xacro` for tutorial_vehicle_sensor_kit_description"

    ```xml
    <?xml version="1.0"?>
    <robot name="vehicle" xmlns:xacro="http://ros.org/wiki/xacro">
      <xacro:arg name="config_dir" default="$(find tutorial_vehicle_sensor_kit_description)/config"/>
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
The shared launch files of `autoware_sensor_driver_launch` and `autoware_sensing_launch` build the LiDAR pipeline.
This image below demonstrates our sensor pipeline, which we will construct in this section.

{{ diagram("images/sensor-kit-launch-design.svg", "A sample sensing design: the drivers and the processing steps that the sensor kit launches.") }}

The `<YOUR-VEHICLE-NAME>_sensor_kit_launch` package folder structure like this:

```diff
<YOUR-VEHICLE-NAME>_sensor_kit_launch/
      ├─ config/
      │     ├─ concatenate_and_time_sync_node.param.yaml
      │     ├─ crop_box_filter_mirror.param.yaml
      │     ├─ crop_box_filter_self.param.yaml
      │     └─ ...
      ├─ data/
      └─ launch/
+           ├─ camera.launch.xml
+           ├─ gnss.launch.xml
+           ├─ imu.launch.xml
+           ├─ lidar.launch.xml
+           └─ sensing.launch.xml
```

So,
we will modify the launch files
which located the `launch` folder for launching and manipulating our sensors.
The main launch file is `sensing.launch.xml`.
`component_sensing.launch.xml` of `autoware_launch` includes this file in the `/sensing` namespace.
This launch file launches other sensing launch files.
The current autoware sensing launch files design for `sensor_kit_launch` package is the diagram below.

{{ diagram("images/sensing-launch-files.svg", "The launch files that sensing.launch.xml includes, and the shared launch files that they use.") }}

The `sensing.launch.xml` also launches `autoware_vehicle_velocity_converter` package
for converting `autoware_vehicle_msgs::msg::VelocityReport` message to `geometry_msgs::msg::TwistWithCovarianceStamped` for gyro_odometer node.
So,
be sure
your vehicle_interface publishes `/vehicle/status/velocity_status` topic with `autoware_vehicle_msgs::msg::VelocityReport` type,
or you must update `input_vehicle_velocity_topic` at `sensing.launch.xml`.

```diff
    ...
    <include file="$(find-pkg-share autoware_vehicle_velocity_converter)/launch/vehicle_velocity_converter.launch.xml">
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

In `lidar.launch.xml`, each LiDAR has its own group with its own namespace.
A group includes three launch files:

1. `pointcloud_container.launch.xml` of `autoware_sensing_launch` creates a component container for the LiDAR.
2. The driver launch file of `autoware_sensor_driver_launch` loads the Nebula driver into this container. There is one file for each LiDAR model: `velodyne_VLP16.launch.xml`, `velodyne_VLS128.launch.xml`, `robosense_Bpearl.launch.xml`, `robosense_Helios.launch.xml`, `hesai_XT32.launch.xml`, and `hesai_OT128.launch.xml`.
3. `pointcloud_preprocessor_cpu.launch.xml` of `autoware_sensing_launch` loads the preprocessing filters into the same container: the crop box filters, the distortion corrector, and the ring outlier filter.

The driver publishes `pointcloud_raw_ex`, and the preprocessing publishes `pointcloud_before_sync` in the namespace of the group.
The `target_container` argument is the absolute path of the container, so it must match the namespace of the group.

If you are using [Velodyne Lidar](https://velodynelidar.com/) sensor,
you can use the [sample_sensor_kit_launch template](https://github.com/autowarefoundation/autoware_launch/blob/main/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml),
but you need to update `sensor_ip`, `data_port`, `sensor_frame` and other necessary changes
(`max_range`, `scan_phase`, etc.).

```diff
    <group>
-     <push-ros-namespace namespace="left"/>
-     <let name="target_container" value="/sensing/lidar/left/pointcloud_preprocessor/pointcloud_container"/>
+     <push-ros-namespace namespace="<YOUR-SENSOR-NAMESPACE>"/>
+     <let name="target_container" value="/sensing/lidar/<YOUR-SENSOR-NAMESPACE>/pointcloud_preprocessor/pointcloud_container"/>

      <include file="$(find-pkg-share autoware_sensing_launch)/launch/pointcloud_container.launch.xml">
-       <arg name="container_namespace" value="/sensing/lidar/left/pointcloud_preprocessor"/>
+       <arg name="container_namespace" value="/sensing/lidar/<YOUR-SENSOR-NAMESPACE>/pointcloud_preprocessor"/>
        <arg name="container_name" value="pointcloud_container"/>
        <arg name="use_multithread" value="false"/>
      </include>

      <include file="$(find-pkg-share autoware_sensor_driver_launch)/launch/velodyne_VLP16.launch.xml">
        <arg name="target_container" value="$(var target_container)"/>
        <arg name="launch_driver" value="$(var launch_driver)"/>
-       <arg name="sensor_frame" value="velodyne_left"/>
+       <arg name="sensor_frame" value="<YOUR-SENSOR-FRAME>"/>
        <arg name="max_range" value="5.0"/>
-       <arg name="sensor_ip" value="192.168.1.202"/>
+       <arg name="sensor_ip" value="<YOUR-SENSOR-IP>"/>
        <arg name="host_ip" value="$(var host_ip)"/>
-       <arg name="data_port" value="2369"/>
+       <arg name="data_port" value="<YOUR-DATA-PORT>"/>
        <arg name="scan_phase" value="180.0"/>
        <arg name="cloud_min_angle" value="300"/>
        <arg name="cloud_max_angle" value="60"/>
      </include>

      <include file="$(find-pkg-share autoware_sensing_launch)/launch/pointcloud_preprocessor_cpu.launch.xml">
        <arg name="target_container" value="$(var target_container)"/>
-       <arg name="frame_id" value="velodyne_left"/>
+       <arg name="frame_id" value="<YOUR-SENSOR-FRAME>"/>
        <arg name="use_intra_process" value="true"/>
        <arg name="crop_box_filter_self_param_path" value="$(var crop_box_filter_self_param_path)"/>
        <arg name="crop_box_filter_secondary_param_path" value="$(var crop_box_filter_mirror_param_path)"/>
      </include>
    </group>
```

Please add similar launch groups according to your sensor architecture.
For example, we use Robosense Lidars for our `tutorial_vehicle`,
so the driver include for a Robosense Lidar (for example, Bpearl) looks like this:

```xml
      <include file="$(find-pkg-share autoware_sensor_driver_launch)/launch/robosense_Bpearl.launch.xml">
        <arg name="target_container" value="$(var target_container)"/>
        <arg name="launch_driver" value="$(var launch_driver)"/>
        <arg name="max_range" value="30.0"/>
        <arg name="sensor_frame" value="<YOUR-ROBOSENSE-SENSOR-FRAME>"/>
        <arg name="sensor_ip" value="<YOUR-ROBOSENSE-SENSOR-IP>"/>
        <arg name="host_ip" value="$(var host_ip)"/>
        <arg name="data_port" value="<YOUR-ROBOSENSE-SENSOR-DATA-PORT>"/>
        <arg name="gnss_port" value="<YOUR-ROBOSENSE-SENSOR-GNSS-PORT>"/>
        <arg name="scan_phase" value="0.0"/>
      </include>
```

If you are using a Hesai lidar (i.e. PandarXT32,
please check [nebula](https://github.com/tier4/nebula) driver page for supported sensors),
the driver include in `lidar.launch.xml` looks like this:

```xml
      <include file="$(find-pkg-share autoware_sensor_driver_launch)/launch/hesai_XT32.launch.xml">
        <arg name="target_container" value="$(var target_container)"/>
        <arg name="launch_driver" value="$(var launch_driver)"/>
        <arg name="max_range" value="120.0"/>
        <arg name="sensor_frame" value="<YOUR-HESAI-SENSOR-FRAME>"/>
        <arg name="sensor_ip" value="<YOUR-HESAI-SENSOR-IP>"/>
        <arg name="host_ip" value="$(var host_ip)"/>
        <arg name="data_port" value="<YOUR-HESAI-SENSOR-DATA-PORT>"/>
        <arg name="cloud_min_angle" value="0"/>
        <arg name="cloud_max_angle" value="360"/>
      </include>
```

If `autoware_sensor_driver_launch` has no launch file for your LiDAR model, you can write one in your sensor kit launch package.
Use a file of `autoware_sensor_driver_launch` as an example: it includes the Nebula launch file of the LiDAR make (`nebula_velodyne.launch.xml`, `nebula_robosense.launch.xml`, or `nebula_hesai.launch.xml`) and loads the driver into `target_container`.

After the groups, `lidar.launch.xml` includes `pointcloud_concatenation_cpu.launch.xml` of `autoware_sensing_launch`.
It concatenates the pointclouds of all LiDARs into `/sensing/lidar/concatenated/pointcloud`.
Write the `pointcloud_before_sync` topics of your LiDARs in the `input_topics` parameter of `config/concatenate_and_time_sync_node.param.yaml`:

```diff
    input_topics: [
-                   "/sensing/lidar/right/pointcloud_before_sync",
-                   "/sensing/lidar/top/pointcloud_before_sync",
-                   "/sensing/lidar/left/pointcloud_before_sync",
+                   "/sensing/lidar/<YOUR-SENSOR-NAMESPACE>/pointcloud_before_sync",
+                   ...
                ]
```

The `lidar_timestamp_offsets` and `lidar_timestamp_noise_window` arrays in the same file must have one value for each input topic.
Please check the [concatenate and time synchronize node documentation](https://autowarefoundation.github.io/autoware_universe/main/sensing/autoware_pointcloud_preprocessor/docs/concatenate-data/) for the meaning of these values.

#### Crop box parameter files

The crop box filters remove the points of your own vehicle body and of the side mirrors.
Their parameter files are in the `config/` folder of the sensor kit launch package: `crop_box_filter_self.param.yaml` and `crop_box_filter_mirror.param.yaml`.
A script of `autoware_sensing_launch` makes these two files from the `vehicle_info.param.yaml` and `mirror.param.yaml` files of your [vehicle model](../creating-vehicle-model/index.md).
Run it again after you change the vehicle dimensions:

```bash
cd <YOUR-AUTOWARE-DIR>
python3 src/launcher/autoware_launch/autoware_universe_launch/autoware_sensing_launch/scripts/generate_crop_box_params.py \
  src/vehicle/<YOUR-VEHICLE-NAME>_vehicle_launch/<YOUR-VEHICLE-NAME>_vehicle_description/config/vehicle_info.param.yaml \
  src/vehicle/<YOUR-VEHICLE-NAME>_vehicle_launch/<YOUR-VEHICLE-NAME>_vehicle_description/config/mirror.param.yaml \
  src/sensor_kit/<YOUR-VEHICLE-NAME>_sensor_kit_launch/<YOUR-VEHICLE-NAME>_sensor_kit_launch/config
```

#### Changing the preprocessing pipeline

The [autoware_pointcloud_preprocessor](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_pointcloud_preprocessor) package has the filters of the pipeline.
Each filter reads a parameter file, and `pointcloud_preprocessor_cpu.launch.xml` has an argument for the path of each file
(`distortion_corrector_node_param_path` and `ring_outlier_filter_node_param_path`).
To change a parameter for one LiDAR, give your own parameter file with these arguments.

To use a different filter (for example, the dual return outlier filter instead of the ring outlier filter),
do not include `pointcloud_preprocessor_cpu.launch.xml` for this LiDAR.
Load the filters of your choice into the container of the LiDAR in your own launch file.
Use `pointcloud_preprocessor_cpu.launch.xml` as a template:

```diff
  <load_composable_node target="$(var target_container)">
    ...
-   <composable_node pkg="autoware_pointcloud_preprocessor" plugin="autoware::pointcloud_preprocessor::RingOutlierFilterComponent" name="ring_outlier_filter">
+   <composable_node pkg="autoware_pointcloud_preprocessor" plugin="autoware::pointcloud_preprocessor::DualReturnOutlierFilterComponent" name="dual_return_outlier_filter">
      <remap from="input" to="rectified/pointcloud_ex"/>
      <remap from="output" to="$(var output/pointcloud)"/>
      ...
    </composable_node>
  </load_composable_node>
```

We will use the default pointcloud preprocessing pipeline for our tutorial_vehicle.

### Camera Launching

In this section,
we will launch our camera driver.
In the `sample_sensor_kit_launch` package, `sensing.launch.xml` does not include `camera.launch.xml`, because the sample vehicle has no camera driver on the main computer.
Its `camera.launch.xml` only relays the topics of a traffic light camera.

You can clone your camera driver
at `src/sensor_component/external` folder
(please don't forget adding this driver to `repositories/autoware.repos` file):

```diff
<YOUR-AUTOWARE-DIR>
   └─ src/
         └─ sensor_component/
            └─ external/
                └─ YOUR-CAMERA-DRIVER
```

After that, you can add your camera driver at `camera.launch.xml`,
and include `camera.launch.xml` in `sensing.launch.xml`:

```diff
    ...
+   <include file="$(find-pkg-share <YOUR-SENSOR-DRIVER>)/launch/YOUR-CAMERA-LAUNCH-FILE">
    ...
```

Please be careful with the topic names.
The camera-based perception of Autoware reads these topics by default
(`N` is the camera number, starting from 0):

- `/sensing/camera/cameraN/image_rect_color`: the rectified image, for the 2D detector (`autoware_tensorrt_yolox`) and the camera-lidar fusion
- `/sensing/camera/cameraN/camera_info`: the camera information, for the camera-lidar fusion

If your driver uses other topic names, remap them in `camera.launch.xml`, or give the topic names to the perception launch files.
The perception launch files can also launch the 2D detector.
See the [perception tuning](../../tuning-parameters-and-performance/tuning-parameters/perception-tuning/index.md#enabling-camera-lidar-fusion) page.

The camera-lidar fusion expects rectified images.
If your sensor driver doesn't support image rectification, you can use the [`image_proc`](https://github.com/ros-perception/image_pipeline/tree/humble/image_proc) package.
You can add something like this in your pipeline for getting rectifying image:

```python
        ...
        ComposableNode(
        namespace=camera_ns,
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify_camera_image_node',
        # Remap subscribers and publishers
        remappings=[
        ('image', camera_ns+"/image"),
        ('camera_info', input_camera_info),
        ('image_rect', 'image_rect')
        ],
        extra_arguments=[
        {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
        ],
        ),
        ...
```

For tutorial_vehicle,
we use the [Lucid Vision TRIO54S](https://thinklucid.com/product/triton-5-mp-imx490/) camera
with the [lucid_vision_driver](https://github.com/autowarefoundation/lucid_vision_driver) at the [autowarefoundation](https://github.com/autowarefoundation) organization.
This driver supports image rectification, so there is no need to add `image_proc` for tutorial_vehicle.

We will add our sensor driver to sensor_kit_launch's `package.xml` dependencies.

```bash
+ <exec_depend><YOUR-CAMERA-DRIVER-PACKAGE></exec_depend>
```

Build necessary packages with:

```bash
cd <YOUR-AUTOWARE-DIR>
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to <YOUR-VEHICLE-NAME>_sensor_kit_launch
```

Then, you can test your camera pipeline:

```bash
ros2 launch <YOUR-SENSOR-KIT-LAUNCH> camera.launch.xml
# example for tutorial_vehicle: ros2 launch tutorial_vehicle_sensor_kit_launch camera.launch.xml
```

Then the image topics will appear,
you can check the image with rviz2 or [rqt](https://docs.ros.org/en/humble/Concepts/Intermediate/About-RQt.html).

### GNSS/INS Launching

We will set up the GNSS/INS sensor launches at `gnss.launch.xml`.
The default GNSS sensor options at [`sample_sensor_kit_launch`](https://github.com/autowarefoundation/autoware_launch/blob/main/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml) for [u-blox](https://www.u-blox.com/en/)
and [septentrio](https://www.septentrio.com/en) is included in `gnss.launch.xml`,
so If we use other sensors as GNSS/INS receiver, we need to add it here.
Moreover, [gnss_poser](https://github.com/autowarefoundation/autoware_core/tree/main/sensing/autoware_gnss_poser) package launches here,
we will use this package for the pose source of our vehicle at localization initialization but remember,
your sensor_driver must provide [autoware gnss orientation message](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_sensing_msgs/msg/GnssInsOrientationStamped.msg) for this node.
If you are ready with your GNSS/INS driver,
you must set `navsatfix_topic_name` and `orientation_topic_name` variables at this launch file for [gnss_poser](https://github.com/autowarefoundation/autoware_core/tree/main/sensing/autoware_gnss_poser) arguments.
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
```

The `gnss_poser` node uses the `header.frame_id` of the `NavSatFix` message as the frame of the GNSS antenna.
So your driver must set this frame ID to the GNSS frame that you defined in `sensor_kit_calibration.yaml`.

Also, you can remove dependencies and unused sensor launch files at `gnss.launch.xml`.
For example,
we use [Clap B7 sensor](https://www.unicore.com/assets/upload/file/CLAP-B7_Product_Brief_En.pdf) as a GNSS/INS and IMU sensor at tutorial_vehicle,
and we use [ntrip_client](https://github.com/Robeff-Technology/ntrip_client) for RTK.
Also, we add these packages to [autoware.repos](https://github.com/leo-drive/autoware.tutorial_vehicle/blob/main/autoware.repos) file.

```diff
+ sensor_component/external/clap_b7_driver:
+   type: git
+   url: https://github.com/Robeff-Technology/clap_b7_driver.git
+   version: release/autoware
+ sensor_component/external/ntrip_client:
+   type: git
+   url: https://github.com/Robeff-Technology/ntrip_client.git
+   version: release/humble
```

So,
our `gnss.launch.xml` for tutorial vehicle looks like this:

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
        <group if="$(var launch_driver)">
          <node pkg="clap_b7_driver" exec="clap_b7_driver_node" name="clap_b7_driver" output="screen">
            <param from="$(find-pkg-share clap_b7_driver)/config/clap_b7_driver.param.yaml"/>
          </node>
          <!-- ntrip Client -->
          <include file="$(find-pkg-share ntrip_client)/launch/ntrip_client.launch.py"/>
        </group>

        <!-- NavSatFix to MGRS Pose -->
        <include file="$(find-pkg-share autoware_gnss_poser)/launch/gnss_poser.launch.xml">
          <arg name="input_topic_fix" value="$(var navsatfix_topic_name)"/>
          <arg name="input_topic_orientation" value="$(var orientation_topic_name)"/>

          <arg name="output_topic_gnss_pose" value="pose"/>
          <arg name="output_topic_gnss_pose_cov" value="pose_with_covariance"/>
          <arg name="output_topic_gnss_fixed" value="fixed"/>
        </include>
      </group>
    </launch>
    ```

Clap B7 includes an IMU also, so for tutorial_vehicle we use its IMU topic in `imu.launch.xml`, as the next section explains.

### IMU Launching

You can add your IMU sensor launch file at `imu.launch.xml` file.
At the [sample_sensor_kit](https://github.com/autowarefoundation/autoware_launch/blob/main/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/imu.launch.xml),
the driver of a [Tamagawa IMU sensor](https://mems.tamagawa-seiki.com/en/) is an example, but it is commented out.
You can add your IMU driver instead of the Tamagawa IMU driver.
Also,
we will launch [gyro_bias_estimator](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_imu_corrector#gyro_bias_estimator) and
[imu_corrector](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_imu_corrector#imu_corrector) at `imu.launch.xml` file.
Please refer these documentations for more information.
Please don't forget changing `imu_raw_name` argument for describing the raw imu topic.

Here is a sample `imu.launch.xml` launch file for autoware:

```diff
<launch>
  <arg name="launch_driver" default="true"/>

  <group>
    <push-ros-namespace namespace="imu"/>

-   <!-- <group>
-     <push-ros-namespace namespace="tamagawa"/>
-     <node pkg="tamagawa_imu_driver" name="tag_serial_driver" exec="tag_serial_driver" if="$(var launch_driver)">
-       <remap from="imu/data_raw" to="imu_raw"/>
-       <param name="port" value="/dev/imu"/>
-       <param name="imu_frame_id" value="tamagawa/imu_link"/>
-     </node>
-   </group> -->

+   <group>
+     <push-ros-namespace namespace="<YOUR-IMU_MODEL>"/>
+     <node pkg="<YOUR-IMU-DRIVER-PACKAGE>" name="<YOUR-IMU-DRIVER>" exec="<YOUR-IMU-DRIVER-EXECUTABLE>" if="$(var launch_driver)">
+     <!-- Add necessary params here -->
+     </node>
+   </group>

-   <arg name="imu_raw_name" default="tamagawa/imu_raw"/>
+   <arg name="imu_raw_name" default="<YOUR-IMU_MODEL/YOUR-RAW-IMU-TOPIC>"/>
-   <arg name="imu_corrector_param_file" default="$(find-pkg-share sample_sensor_kit_description)/config/imu_corrector.param.yaml"/>
+   <arg name="imu_corrector_param_file" default="$(find-pkg-share <YOUR-VEHICLE-NAME>_sensor_kit_description)/config/imu_corrector.param.yaml"/>
    <include file="$(find-pkg-share autoware_imu_corrector)/launch/imu_corrector.launch.xml">
      <arg name="input_topic" value="$(var imu_raw_name)"/>
      <arg name="output_topic" value="imu_data"/>
      <arg name="param_file" value="$(var imu_corrector_param_file)"/>
    </include>

    <include file="$(find-pkg-share autoware_imu_corrector)/launch/gyro_bias_estimator.launch.xml">
      <arg name="input_imu_raw" value="$(var imu_raw_name)"/>
      <arg name="input_odom" value="/localization/kinematic_state"/>
      <arg name="input_pose_ndt" value="/localization/pose_estimator/pose_with_covariance"/>
      <arg name="imu_corrector_param_file" value="$(var imu_corrector_param_file)"/>
    </include>
  </group>
</launch>
```

Please make necessary modifications on this file according to your IMU driver.
For tutorial_vehicle, there is no dedicated IMU sensor.
The Clap B7 driver publishes the IMU data on `/sensing/gnss/clap/ros/imu`, so we set `imu_raw_name` to this topic and remove the IMU driver group.

You can add or remove launch files in `sensing.launch.xml` according to your sensor architecture.
