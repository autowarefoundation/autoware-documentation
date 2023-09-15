# Creating vehicle description

## Introduction

This package contains the parameters of the vehicle, macro files, fbx files and launch file.

This page introduce following topics;

- < VEHICLE-ID > vehicle_description
- < VEHICLE-ID > vehicle_launch

!!! Note

    [Here](https://github.com/leo-drive/tutorial_vehicle_launch) is the sample repository created for the tutorial vehicle.

- You must replace the `sample_vehicle_launch` with the one you forked when creating the Autoware repository page.

```diff
YOUR-OWN-AUTOWARE-DIR/src/vehicle/
-       ├─ sample_vehicle_launch/
+       └─ VEHICLE-ID_vehicle_launch
```

## 1. VEHICLE-ID_vehicle_description

This package contains the parameters of the vehicle, macro files and fbx files.

### Folder Structure

The folder structure of forked `VEHICLE-ID_vehicle_description` should be like this:

```diff
VEHICLE-ID_vehicle_description/
   ├─ config/
   │     ├─ mirror.param.yaml
   │     ├─ simulator_model.param.yaml
   │     └─ vehicle_info.param.yaml
   ├─ mesh/
   │     └─ vehicle.fbx
   └─ urdf/
         └─ vehicle.xacro
```

In `VEHICLE-ID_vehicle_description`, the following configurations are set:

- mirror.param.yaml
- simulator_model.param.yaml
- vehicle_info.param.yaml
- vehicle.fbx
- vehicle.xacro

### 1. mirror.param.yaml

This parameter file contains the dimensions of the mirrors on the sides of the vehicle.
Used in the [crop-box-filter](https://autowarefoundation.github.io/autoware.universe/main/sensing/pointcloud_preprocessor/docs/crop-box-filter/) of
[PointCloudPreprocessor](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/sensing/data-types/point-cloud/).

The original file is [mirror.param.yaml](https://github.com/autowarefoundation/sample_vehicle_launch/blob/main/sample_vehicle_description/config/mirror.param.yaml)
included in [sample_vehicle_launch](https://github.com/autowarefoundation/sample_vehicle_launch/tree/main) and given below:

```yaml
/**:
  ros__parameters:
    min_longitudinal_offset: 1.8
    max_longitudinal_offset: 3.2
    min_lateral_offset: -1.4
    max_lateral_offset: 1.4
    min_height_offset: 0.8
    max_height_offset: 1.5
```

!!! Warning

    Since the tutorial vehicle does not have a mirror, the parameters are set to 0.0.

### 2. simulator_model.param.yaml

Configuration file for the simulator environment.

The original file is [simulator_model.param.yaml](https://github.com/autowarefoundation/sample_vehicle_launch/blob/main/sample_vehicle_description/config/simulator_model.param.yaml)
included in [sample_vehicle_launch](https://github.com/autowarefoundation/sample_vehicle_launch/tree/main) and given below:

??? note "mirror.param.yaml for sample_vehicle_launch"

    ```yaml
    /**:
      ros__parameters:
        simulated_frame_id: "base_link" # center of the rear axle.
        origin_frame_id: "map"
        vehicle_model_type: "DELAY_STEER_ACC_GEARED" # options: IDEAL_STEER_VEL / IDEAL_STEER_ACC / IDEAL_STEER_ACC_GEARED / DELAY_STEER_ACC / DELAY_STEER_ACC_GEARED
        initialize_source: "INITIAL_POSE_TOPIC" #  options: ORIGIN / INITIAL_POSE_TOPIC
        timer_sampling_time_ms: 25
        add_measurement_noise: False # the Gaussian noise is added to the simulated results
        vel_lim: 50.0 # limit of velocity
        vel_rate_lim: 7.0 # limit of acceleration
        steer_lim: 1.0 # limit of steering angle
        steer_rate_lim: 5.0 # limit of steering angle change rate
        acc_time_delay: 0.1 # dead time for the acceleration input
        acc_time_constant: 0.1 # time constant of the 1st-order acceleration dynamics
        steer_time_delay: 0.24 # dead time for the steering input
        steer_time_constant: 0.27 # time constant of the 1st-order steering dynamics
        x_stddev: 0.0001 # x standard deviation for dummy covariance in map coordinate
        y_stddev: 0.0001 # y standard deviation for dummy covariance in map coordinate
    ```

### 3. vehicle_info.param.yaml

Defines the vehicle dimensions. For more details on each parameter, please click [here](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-dimensions/).

The original file is [vehicle_info.param.yaml](https://github.com/autowarefoundation/sample_vehicle_launch/blob/main/sample_vehicle_description/config/vehicle_info.param.yaml)
included in [sample_vehicle_launch](https://github.com/autowarefoundation/sample_vehicle_launch/tree/main) and given below:

```yaml
/**:
ros__parameters:
    wheel_radius: 0.383 # The radius of the wheel, primarily used for dead reckoning.
    wheel_width: 0.235 # The lateral width of a wheel tire, primarily used for dead reckoning.
    wheel_base: 2.79 # between front wheel center and rear wheel center
    wheel_tread: 1.64 # between left wheel center and right wheel center
    front_overhang: 1.0 # between front wheel center and vehicle front
    rear_overhang: 1.1 # between rear wheel center and vehicle rear
    left_overhang: 0.128 # between left wheel center and vehicle left
    right_overhang: 0.128 # between right wheel center and vehicle right
    vehicle_height: 2.5
    max_steer_angle: 0.70 # [rad]
```

The tutorial_vehicle file is [vehicle_info.param.yaml](https://github.com/leo-drive/tutorial_vehicle_launch/blob/main/tutorial_vehicle_description/config/vehicle_info.param.yaml)
included in [tutorial_vehicle_launch](https://github.com/leo-drive/tutorial_vehicle_launch/tree/main) and given below:

??? note "vehicle_info.param.yaml for tutorial_vehicle_launch"

    ```yaml
    /**:
      ros__parameters:
        wheel_radius: 0.27855 # The radius of the wheel, primarily used for dead reckoning.
        wheel_width: 0.155 # The lateral width of a wheel tire, primarily used for dead reckoning.
        wheel_base: 1.60 # between front wheel center and rear wheel center
        wheel_tread: 0.959 # between left wheel center and right wheel center
        front_overhang: 0.245 # between front wheel center and vehicle front
        rear_overhang: 0.245 # between rear wheel center and vehicle rear
        left_overhang: 0.075 # between left wheel center and vehicle left
        right_overhang: 0.075 # between right wheel center and vehicle right
        vehicle_height: 2.
        max_steer_angle: 0.70 # [rad]
    ```

### 4. vehicle.fbx

`vehicle.fbx` is a 3D model file used for visualization in RVIZ.
You must add the fbx or dae file of your own vehicle.

The 3D model of Lexus vehicle prepared for sample_vehicle_launch is [here](https://github.com/autowarefoundation/sample_vehicle_launch/tree/main/sample_vehicle_description/mesh).

### 5. vehicle.xacro

`vehicle.xacro` is a macro file that defines the vehicle model.

The original [vehicle.xacro](https://github.com/autowarefoundation/sample_vehicle_launch/blob/main/sample_vehicle_description/urdf/vehicle.xacro) file for sample_vehicle_launch is given below:

??? note "vehicle.xacro for sample_vehicle_launch"

    ```yaml
    <?xml version="1.0"?>
    <robot xmlns:xacro="http://ros.org/wiki/xacro">
      <!-- load parameter -->
      <xacro:property name="vehicle_info" value="${xacro.load_yaml('$(find sample_vehicle_description)/config/vehicle_info.param.yaml')}"/>

      <!-- vehicle body -->
      <link name="base_link">
        <visual>
          <origin xyz="${vehicle_info['/**']['ros__parameters']['wheel_base']/2.0} 0 0" rpy="${pi/2.0} 0 ${pi}"/>
          <geometry>
            <mesh filename="package://sample_vehicle_description/mesh/lexus.dae" scale="1 1 1"/>
          </geometry>
        </visual>
      </link>
    </robot>
    ```

## 2. VEHICLE-ID_vehicle_launch

`vehicle_interface.launch.xml` is the launch file related to the drive system. Please modify it according
to the configuration of your vehicle's drive system.

### Folder Structure

The folder structure of forked `VEHICLE-ID_vehicle_launch` should be like this:

```diff
VEHICLE-ID_vehicle_launch/
   └─ launch/
         └─ vehicle_interface.launch.xml
```

If you are operating multiple vehicles, use the vehicle_id to switch to the corresponding configuration for each vehicle.

The default file is [vehicle_interface.launch.xml](https://github.com/autowarefoundation/sample_vehicle_launch/blob/main/sample_vehicle_launch/launch/vehicle_interface.launch.xml) for sample_vehicle_launch is given below:

```yaml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
<arg name="vehicle_id" default="$(env VEHICLE_ID default)"/>
</launch>
```
