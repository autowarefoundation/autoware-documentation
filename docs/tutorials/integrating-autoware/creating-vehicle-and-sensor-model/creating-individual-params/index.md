# Sensor calibration files (formerly individual params)

## Introduction

Earlier Autoware versions used an `individual_params` package to store the sensor calibration of each vehicle.
The package overrode the default calibration of the sensor kit, and the `vehicle_id` launch argument selected the calibration of one vehicle.

Autoware 2.0 does not use the `individual_params` package.
The [autoware_individual_params](https://github.com/autowarefoundation/autoware_individual_params) repository is archived.
The sensor calibration is now only in the `config/` folder of the sensor kit description package (`<YOUR-VEHICLE-NAME>_sensor_kit_description`).
Autoware had the same calibration in two places, and this made the launch files hard to manage.
The following issue and pull request made this change:

- autowarefoundation/autoware#5975
- autowarefoundation/autoware_launch#1403

This page shows where the calibration files are now, and how to move your files from an `individual_params` package.

## Where the calibration files are

`autoware.launch.xml` gives the `config_dir` argument `$(find-pkg-share <sensor_model>_description)/config` to `tier4_vehicle_launch`.
The xacro files of the sensor kit description load the calibration files from this folder.
For the `sample_sensor_kit`, this folder is [`sample_sensor_kit_description/config`](https://github.com/autowarefoundation/autoware_launch/tree/main/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description/config) in the `autoware_launch` repository.

| File                          | Content                                                                         | Read by                            |
| ----------------------------- | ------------------------------------------------------------------------------- | ---------------------------------- |
| `sensors_calibration.yaml`    | The pose of `sensor_kit_base_link` and of other sensors relative to `base_link` | `sensors.xacro`                    |
| `sensor_kit_calibration.yaml` | The pose of each sensor relative to `sensor_kit_base_link`                      | `sensor_kit.xacro`                 |
| `imu_corrector.param.yaml`    | The angular velocity offsets and standard deviations of the IMU                 | `imu.launch.xml` of the sensor kit |

The [creating sensor model](../creating-sensor-model/index.md) page explains these files.
The [calibrating sensors](../calibrating-sensors/index.md) pages explain how to find the values.

## Moving the files from an individual_params package

If your Autoware workspace still has an `individual_params` package, do these steps:

1. Copy the files from `individual_params/config/<VEHICLE_ID>/<YOUR-SENSOR-KIT>/` to the `config/` folder of `<YOUR-VEHICLE-NAME>_sensor_kit_description`.
2. Find the launch files of your sensor kit that use `individual_params`, for example `imu.launch.xml` or `gnss.launch.xml`:

   ```bash
   grep -rn individual_params src/sensor_kit/<YOUR-VEHICLE-NAME>_sensor_kit_launch
   ```

3. Change each path to the file in the sensor kit description:

   ```diff
   - <arg name="imu_corrector_param_file" default="$(find-pkg-share individual_params)/config/$(var vehicle_id)/<YOUR-SENSOR-KIT>/imu_corrector.param.yaml"/>
   + <arg name="imu_corrector_param_file" default="$(find-pkg-share <YOUR-VEHICLE-NAME>_sensor_kit_description)/config/imu_corrector.param.yaml"/>
   ```

4. Remove the `individual_params` repository from your `repositories/autoware.repos` file, and remove the `individual_params` dependency from the `package.xml` files.
5. Build the sensor kit packages again:

   ```bash
   cd <YOUR-AUTOWARE-DIR>
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to <YOUR-VEHICLE-NAME>_sensor_kit_description <YOUR-VEHICLE-NAME>_sensor_kit_launch
   ```

## Several vehicles with the same sensors

Each vehicle has its own calibration values, even if the vehicles use the same sensors.
The `sensor_model` launch argument selects both the `<sensor_model>_description` and the `<sensor_model>_launch` packages.
So, to keep the calibration of several vehicles in one workspace, make one sensor model for each vehicle.
For example, `vehicle_1_sensor_kit` and `vehicle_2_sensor_kit` can have the same launch files but different calibration files.
Then select the sensor model of the vehicle when you launch Autoware:

```bash
ros2 launch autoware_launch autoware.launch.xml map_path:=<YOUR-MAP-PATH> vehicle_model:=<YOUR-VEHICLE-NAME>_vehicle sensor_model:=vehicle_1_sensor_kit
```

The `vehicle_id` launch argument still exists, but it does not select a calibration.
`autoware.launch.xml` gives it to the `vehicle_interface.launch.xml` of your vehicle model, and to the control component.
The control component can read controller parameters for each vehicle from `config/trajectory_follower/<VEHICLE_ID>/` of `autoware_control_config` if you set its `use_individual_control_param` argument to `true`.
