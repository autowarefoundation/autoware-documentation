# Creating individual params

## Introduction

In cases where there is more than one vehicle, each vehicle has its own sensor kit. Even if these sensor kits use the
same sensors, they may require different sensor calibrations.

This allows you to define customized sensor calibrations for different vehicles while using the same launch
vehicles or varying calibration requirements.

!!! Warning

    The "individual_params" package contains the calibration
    results for your sensor kit and overrides the default calibration results found in
    VEHICLE-ID_sensor_kit_description/config.

This page introduces the necessary individual parameter files required by Autoware:

- imu_corrector.param.yaml
- sensor_kit_calibration.yaml
- sensors_calibration.yaml

!!! Note

    [Here](https://github.com/leo-drive/tutorial_vehicle_individual_params/tree/main) is the sample repository created for the tutorial vehicle.

- You must change the sensor_kit with you forked at creating repository sections.

```diff
autoware_individual_params/individual_params/config/sensor_kit/
-   ├─ sample_sensor_kit/
+   └─ VEHICLE-ID_sensor_kit/
```

### Folder Structure

The folder structure of forked `individual_params` repository should be like this:

```diff
individual_params/
  └─ config/
       └─ VEHICLE-ID/
            └─ VEHICLE-ID_sensor_kit/
                 ├─ imu_corrector.param.yaml
                 ├─ sensor_kit_calibration.yaml
                 └─ sensors_calibration.yaml
```

## imu_corrector.param.yaml

A file used by imu_corrector.

The default [imu_corrector_param.yaml](https://github.com/autowarefoundation/autoware.universe/blob/main/sensing/imu_corrector/config/imu_corrector.param.yaml) included in [imu_corrector_package](https://github.com/autowarefoundation/autoware.universe/tree/main/sensing/imu_corrector),
but it will be overridden by [autoware_individual_param](https://github.com/autowarefoundation/autoware_individual_params/tree/main/individual_params)
The `imu_corrector_param.yaml` created for `sample_sensor_kit` is given below.

```yaml
/**:
  ros__parameters:
    angular_velocity_stddev_xx: 0.00339
    angular_velocity_stddev_yy: 0.00339
    angular_velocity_stddev_zz: 0.00339
    angular_velocity_offset_x: -0.005799
    angular_velocity_offset_y: -0.007148
    angular_velocity_offset_z: -0.001499
```

`imu_corrector_param.yaml` is calculated with [deviation_estimation_tools](https://github.com/tier4/CalibrationTools/blob/tier4/universe/localization/deviation_estimation_tools/ReadMe.md).

The tutorial_vehicle's [imu_corrector_param.yaml](https://github.com/leo-drive/tutorial_vehicle_individual_params/blob/main/individual_params/config/tutorial_vehicle/tutorial_vehicle_sensor_kit/imu_corrector.param.yaml) file is given below.

```yaml
/**:
  ros__parameters:
    angular_velocity_offset_x: -0.00017
    angular_velocity_offset_y: 0.00206
    angular_velocity_offset_z: -0.00006
    angular_velocity_stddev_xx: 0.05779
    angular_velocity_stddev_yy: 0.05779
    angular_velocity_stddev_zz: 0.05779
```

## sensor_kit_calibration.yaml

How it is created is explained [creating_sensor_description.md](./creating-sensor-description.md).

## sensors_calibration.yaml

How it is created is explained [creating_sensor_description.md](./creating-sensor-description.md).
