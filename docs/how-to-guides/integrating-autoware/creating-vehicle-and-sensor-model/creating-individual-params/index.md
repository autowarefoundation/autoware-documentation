# Creating individual params

## Introduction

In cases where there is more than one same vehicle, each vehicle has the same sensor kit,
but they may require different sensor calibrations, so
[individual_params](https://github.com/autowarefoundation/autoware_individual_params) package allows you
to define customized sensor calibrations for different vehicles while using the same launch
vehicles or varying calibration requirements.

!!! Warning

    The "individual_params" package contains the calibration
    results for your sensor kit and overrides the default calibration results found in
    VEHICLE-ID_sensor_kit_description/config/ directory.

We forked our autoware_individual_params repository
at [creating autoware repositories](../../creating-your-autoware-meta-repository/creating-autoware-meta-repository.md) page step,
(For example,
we created [tutorial_vehicle_individual_params](https://github.com/leo-drive/tutorial_vehicle_individual_params) for our documentation vehicle at this step)
please be sure `<YOUR-VEHICLE-NAME>_individual_params` repository is included in autoware like the directory below.
Please create directory under the `config` directory with your vehicle id of your vehicle.
(There will be one from the tutorial vehicle,
so we will think 'tutorial_vehicle' as vehicle_id and vehicle_name)
You need
to create or copy [`imu_corrector.param.yaml`](https://github.com/autowarefoundation/autoware.universe/blob/main/sensing/imu_corrector/config/imu_corrector.param.yaml),
[`sensor_kit_calibration.yaml`](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_description/config/sensor_kit_calibration.yaml) and [`sensors_calibration.yaml`](https://github.com/autowarefoundation/sample_sensor_kit_launch/blob/main/sample_sensor_kit_description/config/sensors_calibration.yaml) files
under the `config/<YOUR-VEHICLE-ID>/<YOUR-VEHICLE-NAME>_sensor_kit_launch/` directory.
Then, individual params will override these original files.

```diff
<YOUR-OWN-AUTOWARE-DIR>/
  └─ src/
       └─ param/
            └─ <YOUR-VEHICLE-NAME>_individual_params/
                 └─ individual_params/
                    └─ config/
                        ├─ default/
+                       └─ <YOUR-VEHICLE-ID>/
+                           └─ <YOUR-VEHICLE-NAME>_sensor_kit_launch/
+                               ├─ imu_corrector.param.yaml
+                               ├─ sensor_kit_calibration.yaml
+                               └─ sensors_calibration.yaml
```

??? note "sample folder structure for [`tutorial_vehicle_individual_params`](https://github.com/leo-drive/tutorial_vehicle_individual_params)"

    ```diff
    <YOUR-OWN-AUTOWARE-DIR>/
      └─ src/
           └─ param/
                └─ tutorial_vehicle_individual_params/
                     └─ individual_params/
                        └─ config/
                            ├─ default/
    +                       └─ tutorial_vehicle/
    +                           └─ tutorial_vehicle_sensor_kit_launch/
    +                               ├─ imu_corrector.param.yaml
    +                               ├─ sensor_kit_calibration.yaml
    +                               └─ sensors_calibration.yaml
    ```

After that, we need to build individual_params package:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select individual_params
```

Now we are ready to use Autoware with vehicle_id as an arguments.
For example, if our vehicle is more than one,
the autoware_individual_params structure should be like this:

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

Add a `<vehicle_id>` directory and switch parameters using options at startup.

```bash
# example1 (do not set vehicle_id)
$ ros2 launch autoware_launch autoware.launch.xml sensor_model:=<YOUR-VEHICLE-NAME>_sensor_kit vehicle_model:=<YOUR-VEHICLE-NAME>_vehicle
# example2 (set vehicle_id as VEHICLE_1)
$ ros2 launch autoware_launch autoware.launch.xml sensor_model:=<YOUR-VEHICLE-NAME>_sensor_kit vehicle_model:=<YOUR-VEHICLE-NAME>_vehicle vehicle_id:=VEHICLE_1
# example3 (set vehicle_id as VEHICLE_2)
$ ros2 launch autoware_launch autoware.launch.xml sensor_model:=<YOUR-VEHICLE-NAME>_sensor_kit vehicle_model:=<YOUR-VEHICLE-NAME>_vehicle vehicle_id:=VEHICLE_2
```
