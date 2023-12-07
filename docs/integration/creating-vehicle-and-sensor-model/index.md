# Creating vehicle and sensor models

## Overview

### Sensor Model

- **Purpose:** The sensor model includes the calibration (transformation) and launch files of the
  sensors used in the autonomous vehicle.
  This includes various sensors like LiDARs, cameras,
  radars, IMUs (Inertial Measurement Units), GPS units, etc.

- **Importance:** Accurate sensor modeling is essential for perception tasks.
  Precise calibration values help understand the environment by processing sensor data,
  such as detecting objects, estimating distances,
  and creating a 3D representation of the surroundings.

- **Usage:** The sensor model is utilized in Autoware for launching sensors,
  configuring their pipeline, and describing calibration values.

- The sensor model (sensor kit) consists of the following three packages:
  - `common_sensor_launch`
  - `<YOUR_VEHICLE_NAME>_sensor_kit_description`
  - `<YOUR_VEHICLE_NAME>_sensor_kit_launch`

Please refer to the [creating sensor model](./creating-sensor-model) page
for creating your individual sensor model.

For reference,
here is the folder structure for the [sample_sensor_kit_launch](https://github.com/autowarefoundation/sample_sensor_kit_launch) package in Autoware:

```diff
sample_sensor_kit_launch/
├─ common_sensor_launch/
├─ sample_sensor_kit_description/
└─ sample_sensor_kit_launch/
```

### Vehicle Model

- **Purpose:** The vehicle model includes individual vehicle specifications with dimensions,
  a 3D model of the vehicle (in .fbx or .dae format), etc.

- **Importance:** An accurate vehicle model is crucial for motion planning and control.

- **Usage:** The vehicle model is employed in Autoware to provide vehicle information for Autoware,
  including the 3D model of the vehicle.

- The vehicle model comprises the following two packages:
  - `<YOUR_VEHICLE_NAME>_vehicle_description`
  - `<YOUR_VEHICLE_NAME>_vehicle_launch`

Please consult the [creating vehicle model](./creating-vehicle-model) page
for creating your individual vehicle model.

As a reference,
here is the folder structure for the [sample_vehicle_launch](https://github.com/autowarefoundation/sample_vehicle_launch) package in Autoware:

```diff
sample_vehicle_launch/
├─ sample_vehicle_description/
└─ sample_vehicle_launch/
```
