# Creating vehicle and sensor description

## Introduction

This page introduces the custom packages that you should include in your autoware project to integrate autoware to your vehicle:

1. YOUR_VEHICLE_description
2. YOUR_SENSOR_KIT_description
3. individual_parameter
4. YOUR_VEHICLE_launch
5. YOUR_SENSOR_KIT_launch

## Repository and package structure based on Autoware's sample packages

This guide assumes you will fork Autoware's sample repositories and packages to use as templates for your custom implementation. You can fork Autoware's sample packages:([sample_sensor_kit_launch](https://github.com/autowarefoundation/sample_sensor_kit_launch.git) , [sample_vehicle_launch](https://github.com/autowarefoundation/sample_vehicle_launch.git), and [autoware_individual_params](https://github.com/autowarefoundation/autoware_individual_params.git)), and rename their folders and files according to your vehicle name/denomination. If do so, you should have folder structures like these:

```
YOUR_VEHICLE_launch (Custom Repository name)
├── YOUR_VEHICLE_description (package)
│ ├── CMakeLists.txt
│ ├── config
│ │ ├── mirror.param.yaml (must be changed)
│ │ ├── simulator_model.param.yaml
│ │ └── vehicle_info.param.yaml (must be changed)
│ ├── mesh
│ │ └── YOUR_VEHICLE_MESH.dae (if you have a .dae mesh, otherwise, use the provided one)
│ ├── package.xml
│ └── urdf
│ └── vehicle.xacro
├── YOUR_VEHICLE_launch (package)
│ ├── CMakeLists.txt
│ ├── launch
│ │ └── vehicle_interface.launch.xml (must be changed)
│ └── package.xml
└── ...

YOUR_SENSOR_KIT_launch (Custom Repository name)
├── common_sensor_launch
│ ├── ...
├── YOUR_SENSOR_KIT_description (package)
│ ├── CMakeLists.txt
│ ├── config
│ │ ├── sensor_kit_calibration.yaml (NOT USED BY AUTOWARE)
│ │ └── sensors_calibration.yaml (NOT USED BY AUTOWARE)
│ ├── package.xml
│ └── urdf
│ ├── sensor_kit.xacro (must be changed)
│ └── sensors.xacro (must be changed)
├── YOUR_SENSOR_KIT_launch (package)
│ ├── CMakeLists.txt
│ ├── config
│ │ ├── ...
│ │
│ ├── data
│ │ └── ...
│ ├── launch
│ │ ├── camera.launch.xml
│ │ ├── gnss.launch.xml
│ │ ├── imu.launch.xml (must be changed)
│ │ ├── lidar.launch.xml (must be changed)
│ │ ├── pointcloud_preprocessor.launch.py (must be changed)
│ │ └── sensing.launch.xml (must be changed)
│ └── package.xml
├── README.md
└── ...

YOUR_AUTOWARE_INDIVIDUAL_PARAMS (Custom Repository name)
├── individual_params (package)
│ ├── config
│ │ └── default
│ │ ├── awsim_sensor_kit
│ │ │ ├── sensor_kit_calibration.yaml
│ │ │ └── sensors_calibration.yaml
│ │ └── YOUR_SENSOR_KIT
│ │ ├── imu_corrector.param.yaml
│ │ ├── sensor_kit_calibration.yaml (must be changed)
│ │ └── sensors_calibration.yaml (must be changed)
│ └── ...
└── ...
```

Please note that the YOUR_VEHICLE_launch repository contains two ROS packages that need to be modified: YOUR_VEHICLE_description, and YOUR_VEHICLE_launch (same name as he repository itself). Likewise, YOUR_SENSOR_KIT_launch also contains two ROS packages that require customization: YOUR_SENSOR_KIT_description and YOUR_SENSOR_KIT_launch (same name as the repo). Finally, YOUR_AUTOWARE_INDIVIDUAL_PARAMS contains information about your sensor implementation.

Also,the YOUR_AUTOWARE_INDIVIDUAL_PARAMS package contains the information about your sensor kit and calibration (sensor_kit_calibration.yaml and sensors_calibration.yaml) Autoware will NOT use the parameters contained in YOUR_SENSOR_KIT_launch/YOUR_SENSOR_KIT_description/config. Please, do not confuse them.

## 1. YOUR_VEHICLE_launch/YOUR_VEHICLE_description

In the `YOUR_VEHICLE_description` package, the following configurations are set:

1. vehicle_info.param.yaml (must be changed)
2. mesh file (\*.dae)
3. mirror.param.yaml(must be changed)
4. simulator_model.param.yaml
5. vehicle.xacro

### 1. vehicle_info.param.yaml

Defines the vehicle dimensions. For more details on each parameter, please click [here](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-dimensions).

### 2. mesh file

A 3D model file used for visualization in rviz.

### 3. mirror.param.yaml

Set according to the vehicle dimensions. Used in the [crop-box-filter](https://autowarefoundation.github.io/autoware.universe/main/sensing/pointcloud_preprocessor/docs/crop-box-filter) of [PointCloudPreprocessor](../../../design/autoware-architecture/sensing/data-types/point-cloud.md).

### 4. simulator_model.param.yaml

Configuration file for the [simulator environment](https://autowarefoundation.github.io/autoware.universe/main/simulator/simple_planning_simulator/design/simple_planning_simulator-design/).

### 5. vehicle.xacro

The entry point file that defines the entire URDF of the vehicle. It refers to `sensors.xacro`, which specifies the sensor mounting positions.

## 2. YOUR_SENSOR_KIT_launch/YOUR_SENSOR_KIT_description

In the `YOUR_SENSOR_KIT_description` package, the following files are configured:

1. sensors.xacro (must be changed)
2. sensor_kit.xacro (must be changed)

### 1. sensors.xacro

Resolves the positions of sensors with `base_link` as the parent frame and defines the positions and orientations based on `sensors_calibration.yaml` in `YOUR_AUTOWARE_INDIVIDUAL_PARAMS`.

> In Autoware, `<YOUR_SENSOR_KIT_description>/config/sensors_calibration.yaml` is not used.

#### About sensor_kit_base_link

A `sensor_kit` refers to a subset that includes multiple sensors, and `sensor_kit_base_link` is the name of its frame.
The positions and orientations within the kit are defined in `sensor_kit.xacro`.

### 2. sensor_kit.xacro

Resolves the positions of sensors with `sensor_kit_base_link` as the parent and defines the positions and orientations based on `sensor_kit_calibration.yaml` in `YOUR_AUTOWARE_INDIVIDUAL_PARAMS`.

> In Autoware, `<YOUR_SENSOR_KIT_description>/config/sensor_kit_calibration.yaml` is not used.

## 3. YOUR_AUTOWARE_INDIVIDUAL_PARAMS/individual_params

The `individual_params` package is where parameters referenced by `sensors.xacro` and `sensor_kit.xacro` are stored. As the name implies, it is intended to manage parameters for multiple individual instances.

### Introduction to Various Parameters

1. sensors_calibration.yaml (must be changed)
2. sensor_kit_calibration.yaml (must be changed)
3. imu_corrector.param.yaml

### 1. sensors_calibration.yaml

A file that defines the mounting positions and orientations of sensors with `base_link` as the parent frame.

### 2. sensor_kit_calibration.yaml

A file that defines the mounting positions and orientations of sensors with `sensor_kit_base_link` as the parent frame.

### 3. imu_corrector.param.yaml

A file used by `imu_corrector`.

### 4. Folder Structure

Below is the default directory structure.

```diff
individual_params/
└─ config/
     └─ default/
          └─ sample_sensor_kit/
               ├─ imu_corrector.param.yaml
               ├─ sensor_kit_calibration.yaml
               └─ sensors_calibration.yaml
```

Copy and create a folder based on your `YOUR_SENSOR_KIT` name.

```diff
individual_params/
└─ config/
     └─ default/
-         └─ sample_sensor_kit/
+         └─ <YOUR_SENSOR_KIT>/
               ├─ imu_corrector.param.yaml
               ├─ sensor_kit_calibration.yaml
               └─ sensors_calibration.yaml
```

#### 4.1 Sample Usage

Here is an example of managing parameters for multiple instances.
Add a `<vehicle_id>` directory and switch parameters using options at startup.

```bash
# example1 (do not set vehicle_id)
$ ros2 launch autoware_launch autoware.launch.xml sensor_model:=<YOUR_SENSOR_KIT> vehicle_mode:=<your_vehicle_model>
# example2 (set vehicle_id as VEHICLE_1)
$ ros2 launch autoware_launch autoware.launch.xml sensor_model:=<YOUR_SENSOR_KIT> vehicle_mode:=<your_vehicle_model> vehicle_id:=VEHICLE_1
# example3 (set vehicle_id as VEHICLE_2)
$ ros2 launch autoware_launch autoware.launch.xml sensor_model:=<YOUR_SENSOR_KIT> vehicle_mode:=<your_vehicle_model> vehicle_id:=VEHICLE_2
```

##### Sample Directory Structure

```diff
individual_params/
└─ config/
     ├─ default/
     │   └─ <YOUR_SENSOR_KIT>/                  # example1
     │        ├─ imu_corrector.param.yaml
     │        ├─ sensor_kit_calibration.yaml
     │        └─ sensors_calibration.yaml
+    ├─ VEHICLE_1/
+    │   └─ <YOUR_SENSOR_KIT>/                  # example2
+    │        ├─ imu_corrector.param.yaml
+    │        ├─ sensor_kit_calibration.yaml
+    │        └─ sensors_calibration.yaml
+    └─ VEHICLE_2/
+         └─ <YOUR_SENSOR_KIT>/                  # example3
+              ├─ imu_corrector.param.yaml
+              ├─ sensor_kit_calibration.yaml
+              └─ sensors_calibration.yaml
```

## 4.YOUR_VEHICLE_launch/YOUR_VEHICLE_launch

The `YOUR_VEHICLE_launch` package is where the launch file for starting the drive system devices is stored.

1. vehicle_interface.launch.xml (must be changed)

### 1. vehicle_interface.launch.xml

`vehicle_interface.launch.xml` is the launch file related to the drive system. Please modify it according to the configuration of your vehicle's drive system.

If you are operating multiple vehicles, use the `vehicle_id` to switch to the corresponding configuration for each vehicle.

## 5. YOUR_SENSOR_KIT_launch/YOUR_SENSOR_KIT_launch

The `YOUR_SENSOR_KIT_launch` package is where the launch files related to sensor startup are stored.

1. sensing.launch.xml (must be changed)
2. lidar.launch.xml (must be changed)
3. camera.launch.xml
4. imu.launch.xml (must be changed)
5. gnss.launch.xml
6. pointcloud_preprocessor.launch.py (must be changed)

### 1. sensing.launch.xml

`sensing.launch.xml` is the entry point that calls the launch files for all sensors. Modify it according to your sensor configuration.

### 2. lidar.launch.xml

`lidar.launch.xml` is the launch file related to starting the LiDAR driver. Modify it according to your LiDAR configuration.

> In Autoware's initial configuration, it assumes converting the acquired data using `pointcloud_preprocessor.launch.py`.

#### Example Configuration Items

- Setting the frame_id defined in `YOUR_SENSOR_KIT_description`.
- Connection information for each device.

### 3. camera.launch.xml

`camera.launch.xml` is the launch file related to starting the camera driver.

### 4. imu.launch.xml

`imu.launch.xml` is the launch file related to starting the IMU driver.

### 5. gnss.launch.xml

`gnss.launch.xml` is the launch file related to starting the GNSS driver.

### 6. pointcloud_preprocessor.launch.py

`pointcloud_preprocessor.launch.py` is the launch file to convert the raw sensor data. For more information, please click [here](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/sensing/data-types/point-cloud/).
