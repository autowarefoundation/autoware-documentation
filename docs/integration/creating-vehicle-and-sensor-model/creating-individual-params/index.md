# Creating individual params

## Introduction

The [individual_params](https://github.com/autowarefoundation/autoware_individual_params) package is used
to define customized sensor calibrations for different vehicles.
It lets
you define customized sensor calibrations for different vehicles
while using the same launch files with the same sensor model.

!!! Warning

    The "individual_params" package contains the calibration
    results for your sensor kit and overrides the default calibration results found in
    VEHICLE-ID_sensor_kit_description/config/ directory.

## Placing your `individual_parameters` repository inside Autoware

[Previously on this guide](../../creating-your-autoware-repositories/index.md),
we forked the `autoware_individual_params` repository
to create a [tutorial_vehicle_individual_params](https://github.com/leo-drive/tutorial_vehicle_individual_params) repository
which will be used as an example for this section of the guide.
Your individual_parameters repository should be placed inside your Autoware folder following the same folder structure as the one shown below:

??? note "sample folder structure for [`tutorial_vehicle_individual_params`](https://github.com/leo-drive/tutorial_vehicle_individual_params)"

    ```diff
      <YOUR-OWN-AUTOWARE-DIR>/
      └─ src/
      └─ param/
      └─ tutorial_vehicle_individual_params/
      └─ individual_params/
      └─ config/
      ├─ default/
    + └─ tutorial_vehicle/
    +     └─ tutorial_vehicle_sensor_kit_launch/
    +         ├─ imu_corrector.param.yaml
    +         ├─ sensor_kit_calibration.yaml
    +         └─ sensors_calibration.yaml
    ```

After that, we need to build our `individual_params` package:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to individual_params
```

Now you are ready to use Autoware with a vehicle_id as an argument.
For example, if you are several, similar vehicles with different sensor calibration requirements,
your autoware_individual_params structure should look like this:

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

Then, you can use autoware with vehicle_id arguments like this:

Add a `<vehicle_id>` as an argument and switch parameters using options at startup.

```bash
# example1 (do not set vehicle_id)
$ ros2 launch autoware_launch autoware.launch.xml sensor_model:=<YOUR-VEHICLE-NAME>_sensor_kit vehicle_model:=<YOUR-VEHICLE-NAME>_vehicle
# example2 (set vehicle_id as VEHICLE_1)
$ ros2 launch autoware_launch autoware.launch.xml sensor_model:=<YOUR-VEHICLE-NAME>_sensor_kit vehicle_model:=<YOUR-VEHICLE-NAME>_vehicle vehicle_id:=VEHICLE_1
# example3 (set vehicle_id as VEHICLE_2)
$ ros2 launch autoware_launch autoware.launch.xml sensor_model:=<YOUR-VEHICLE-NAME>_sensor_kit vehicle_model:=<YOUR-VEHICLE-NAME>_vehicle vehicle_id:=VEHICLE_2
```
