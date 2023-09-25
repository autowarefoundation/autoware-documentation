# Creating vehicle and sensor models

## Overview

### Sensor Model

- **Purpose:** The sensor model includes the calibration (transformation) and launch files of the sensors used in the autonomous vehicle. This includes various sensors like LiDARs, cameras, radars, IMUs (Inertial Measurement Units), GPS units, etc.
- **Importance:** Accurate sensor modeling is essential for perception tasks. True calibration values helps in understanding the environment by processing sensor data, such as detecting objects, estimating distances, and creating a 3D representation of the surroundings.
- **Usage:** The sensor model is used in Autoware for launching sensors, their pipeline and describing calibration values.

- The sensor model (sensor kit) consists of three following packages:
  - `common_sensor_launch`
  - `<YOUR-VEHICLE-NAME>_sensor_kit_description`
  - `<YOUR-VEHICLE-NAME>_sensor_kit_launch`

Please follow the [creating sensor model](./creating-sensor-model) page
for creating your own individual sensor model.

Here is the [sample_sensor_kit_launch](https://github.com/autowarefoundation/sample_sensor_kit_launch) package folder structure for autoware:

```diff
sample_sensor_kit_launch/
├─ common_sensor_launch/
├─ sample_sensor_kit_description/
└─ sample_sensor_kit_launch/
```

### Vehicle Model

- **Purpose:** The vehicle model includes individual vehicle specifications with dimensions, 3D model of vehicle (.fbx or .dae format), etc.
- **Importance:** An accurate vehicle model is crucial for motion planning and control.
- **Usage:** The vehicle model is used in autoware for providing vehicle information for autoware, 3D model of vehicle, etc.

- The sensor model (sensor kit) consists of three following packages:
  - `<YOUR-VEHICLE-NAME>_vehicle_description`
  - `<YOUR-VEHICLE-NAME>_vehicle_launch`

Please follow the [creating vehicle model](./creating-vehicle-model) page
for creating your own individual vehicle model.

Here is the [sample_vehicle_launch](https://github.com/autowarefoundation/sample_vehicle_launch) package folder structure for autoware:

```diff
sample_vehicle_launch/
├─ sample_vehicle_description/
└─ sample_vehicle_launch/
```
