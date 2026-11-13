# Manual calibration for all sensors

## Overview

In this section, we will make a manual extrinsic calibration of our sensors.
We set the sensor poses from measurements or CAD data, and then we correct them by hand while we look at the sensor data in RViz.
After this process, we won't get accurate calibration results for final use,
but we will have an initial calibration for other tools.
For example, in the lidar-lidar or camera-lidar calibration phase,
we will need initial calibration for getting accurate and successful calibration results.

We need a sample bag file for the calibration process
which includes raw lidar topics and camera topics.
The following shows an example of a bag file used for calibration:

??? note "ROS 2 Bag example of our calibration process"

    ```sh

    Files:             rosbag2_2023_09_06-13_43_54_0.db3
    Bag size:          18.3 GiB
    Storage id:        sqlite3
    Duration:          169.12s
    Start:             Sep  6 2023 13:43:54.902 (1693997034.902)
    End:               Sep  6 2023 13:46:43.914 (1693997203.914)
    Messages:          8504
    Topic information: Topic: /sensing/lidar/top/pointcloud_raw | Type: sensor_msgs/msg/PointCloud2 | Count: 1691 | Serialization Format: cdr
                       Topic: /sensing/lidar/front/pointcloud_raw | Type: sensor_msgs/msg/PointCloud2 | Count: 1691 | Serialization Format: cdr
                       Topic: /sensing/camera/camera0/image_rect | Type: sensor_msgs/msg/Image | Count: 2561 | Serialization Format: cdr
                       Topic: /sensing/camera/camera0/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 2561 | Serialization Format: cdr
    ```

## Extrinsic manual calibration

!!! note

    Earlier versions of CalibrationTools had an `extrinsic_calibration_manager` package with a manual calibrator.
    The current CalibrationTools does not have this package.
    Its `tunable_static_tf_broadcaster` package (an rqt_reconfigure tool to change a `tf` by hand) is not built by default, because the package has a `COLCON_IGNORE` file.
    So this page uses only the files of your sensor kit description and RViz.

### Setting the initial values

Open `sensor_kit_calibration.yaml` and `sensors_calibration.yaml` in the `config/` folder of your sensor kit description package.
Write the pose of each sensor from your measurements or from the CAD data of your vehicle.
The [creating sensor model](../../creating-sensor-model/index.md#sensor-description) page explains the frames in these files.

For tutorial_vehicle, there are four sensors in `sensor_kit_calibration.yaml` (two lidars, one camera, one GNSS/INS):

```yaml
sensor_kit_base_link:
  rs_helios_top_base_link: # Lidar
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  rs_bpearl_front_base_link: # Lidar
    ...
  camera0/camera_link: # Camera
    ...
  GNSS_INS/gnss_ins_link: # GNSS/INS
    ...
```

### Checking the calibration in RViz

Build the sensor kit description package after each change:

```bash
cd <YOUR-OWN-AUTOWARE-DIRECTORY>
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <OWN-SENSOR-KIT>_description
```

Then publish the `tf` of your vehicle and sensors with `tier4_vehicle_launch`.
It starts `robot_state_publisher` with the URDF of your vehicle model and sensor model:

```bash
ros2 launch tier4_vehicle_launch vehicle.launch.xml vehicle_model:=<OWN-VEHICLE-MODEL> sensor_model:=<OWN-SENSOR-KIT> launch_vehicle_interface:=false use_sim_time:=true
```

For tutorial vehicle:

```bash
ros2 launch tier4_vehicle_launch vehicle.launch.xml vehicle_model:=tutorial_vehicle sensor_model:=tutorial_vehicle_sensor_kit launch_vehicle_interface:=false use_sim_time:=true
```

Then play ROS 2 bag file:

```bash
ros2 bag play <rosbag_path> --clock -l -r 0.2 \
--remap /tf:=/null/tf /tf_static:=/null/tf_static # if tf is recorded
```

Start `rviz2`, set the fixed frame to `base_link`, and add a `PointCloud2` display for each lidar topic.
For the camera, add a `Camera` display. It shows the lidar points on the camera image.

- If the point clouds of two lidars do not overlap, change the pose of one lidar.
- If the ground is not flat or not at the height zero of `base_link`, change `z`, `roll`, and `pitch`.
- If the lidar points do not fall on the same objects in the camera image, change the pose of the camera.

`robot_state_publisher` reads the calibration files only when it starts.
So after each change, build the package again, and launch `vehicle.launch.xml` again.

When the sensors are approximately aligned, keep the values in `sensor_kit_calibration.yaml` and `sensors_calibration.yaml`.
The other calibration tools start from these values.

!!! warning

    The initial calibration process can be important before the using other calibrations. We will look into the lidar-lidar calibration
    and camera-lidar calibration. At this point, there is hard to calibrate two sensors with exactly same frame, so you should find
    approximately (it not must be perfect) calibration pairs between sensors.

For camera-lidar pairs, you can also use the [interactive camera-lidar calibrator](../lidar-camera-calibration/index.md).
You select pairs of points in the image and in the point cloud, and the tool calculates the pose of the camera.
