# Localization component design

## Abstract

The Localization component estimates the pose, velocity, and acceleration of the ego vehicle. Planning, control, and perception use these estimates. This page gives the requirements of the component, its architecture, and the packages that implement it in Autoware.

<div class="aw-pipeline" markdown>

- :fa-cl-s fa-satellite-dish: <span>**Sensors** and maps</span>
- :fa-cl-s fa-crosshairs: <span>**Pose and twist** estimators</span>
- :fa-cl-s fa-filter: <span>**Fusion filter** (EKF)</span>
- :fa-cl-s fa-car: <span>**Kinematic state** for other components</span>

</div>

## 1. Requirements

Localization aims to estimate vehicle pose, velocity, and acceleration.

Goals:

- Propose a system that can estimate vehicle pose, velocity, and acceleration for as long as possible.
- Propose a system that can diagnose the stability of estimation and send a warning message to the error-monitoring system if the estimation result is unreliable.
- Design a vehicle localization function that can work with various sensor configurations.

Non-goals:

- This design document does not aim to develop a localization system that
  - is infallible in all environments
  - works outside of the pre-defined ODD (Operational Design Domain)
  - has better performance than is required for autonomous driving

## 2. Sensor configuration examples

This section shows example sensor configurations and their expected performances.
Each sensor has its own advantages and disadvantages, but overall performance can be improved by fusing multiple sensors.

### 3D-LiDAR + PointCloud Map

#### Expected situation

- The vehicle is located in a structure-rich environment, such as an urban area

#### Situations that can make the system unstable

- The vehicle is placed in a structure-less environment, such as a rural landscape, highway, or tunnel
- Environmental changes have occurred since the map was created, such as snow cover or the construction/destruction of buildings.
- Surrounding objects are occluded
- The car is surrounded by objects undetectable by LiDAR, for example glass windows, reflections, or absorption (dark objects)
- The environment contains laser beams at the same frequency as the car's LiDAR sensor(s)

#### Functionality

- The system can estimate the vehicle location on the point cloud map with an error of about 10 cm.
- The system is operable at night.

### 3D-LiDAR or Camera + Vector Map

#### Expected situation

- Road with clear white lines and loose curvatures, such as a highway or an ordinary local road.

#### Situations that can make the system unstable

- White lines are scratchy or covered by rain or snow
- Tight curvature such as intersections
- Large reflection change of the road surface caused by rain or paint

#### Functionality

- Correct vehicle positions along the lateral direction.
- Pose correction along the longitudinal can be inaccurate, but can be resolved by fusing with GNSS.

### GNSS

#### Expected situation

- The vehicle is placed in an open environment with few to no surrounding objects, such as a rural landscape.

#### Situations that can make the system unstable

- GNSS signals are blocked by surrounding objects, for example tunnels or buildings.

#### Functionality

- The system can estimate vehicle position in the world coordinate within an error of about 10 m.
- With an RTK-GNSS (Real Time Kinematic Global Navigation Satellite System) attached, the accuracy can be improved to about 10 cm.
- A system with this configuration can work without environment maps (both point cloud and vector map types).

### Camera (Visual Odometry, Visual SLAM)

#### Expected situation

- The vehicle is placed in an environment with rich visual features, such as an urban area.

#### Situations that can make the system unstable

- The vehicle is placed in a texture-less environment.
- The vehicle is surrounded by other objects.
- The camera observes significant illumination changes, such as those caused by sunshine, headlights from other vehicles or when approaching the exit of a tunnel.
- The vehicle is placed in a dark environment.

#### Functionality

- The system can estimate odometry by tracking visual features.

### Wheel speed sensor

#### Expected situation

- The vehicle is running on a flat and smooth road.

#### Situations that can make the system unstable

- The vehicle is running on a slippery or bumpy road, which can cause incorrect observations of wheel speed.

#### Functionality

- The system can acquire the vehicle velocity and estimate distance traveled.

### IMU

#### Expected situation

- Flat, smooth roads

#### Situations that can make the system unstable

- IMUs have a bias[^1] that is dependent on the surrounding temperature, and can cause incorrect sensor observation or odometry drift.

[^1]: For more details about bias, refer to the [VectorNav IMU specifications page](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-imuspecs).

#### Functionality

- The system can observe acceleration and angular velocity.
- By integrating these observations, the system can estimate the local pose change and realize dead-reckoning

### Geomagnetic sensor

#### Expected situation

- The vehicle is placed in an environment with low magnetic noise

#### Situations that can make the system unstable

- The vehicle is placed in an environment with high magnetic noise, such as one containing buildings or structures with reinforced steel or other materials that generate electromagnetic waves.

#### Functionality

- The system can estimate the vehicle's direction in the world coordinate system.

### Magnetic markers

#### Expected situation

- The car is placed in an environment with magnetic markers installed.

#### Situations that can make the system unstable

- The markers are not maintained.

#### Functionality

- Vehicle location can be obtained on the world coordinate by detecting the magnetic markers.
- The system can work even if the road is covered with snow.

## 3. Functional requirements

- By implementing different modules, various sensor configurations and algorithms can be used.
- The localization system can start pose estimation from an ambiguous initial location.
- The system can produce a reliable initial location estimation.
- The system can manage the state of the initial location estimation and can report it to the error monitor. Autoware reports the states `UNINITIALIZED`, `INITIALIZING`, and `INITIALIZED` (and `UNKNOWN` before the first report).

## 4. Architecture

### Abstract

Two architectures are defined, "Required" and "Recommended". However, the "Required" architecture only contains the inputs and outputs necessary to accept various localization algorithms. To improve the reusability of each module, the required components are defined in the "Recommended" architecture section along with a more detailed explanation.

### Required architecture

{{ diagram("image/required-architecture.svg", "The inputs and outputs that every localization implementation must provide.") }}

#### Input

- Sensor message
  - for example LiDAR, camera, GNSS, IMU, CAN Bus, etc.
  - Data types should be ROS primitives for reusability
- Map data
  - for example point cloud map, lanelet2 map, feature map, etc.
  - The map format should be chosen based on use case and sensor configuration
  - Note that map data is not required for some specific cases (for example GNSS-only localization)
- tf, static_tf
  - map frame
  - base_link frame

#### Output

- Pose with covariance stamped
  - Vehicle pose, covariance, and timestamp on the map coordinate
  - 50 Hz or more (depending on the requirements of the Planning and Control components)
- Twist with covariance stamped
  - Vehicle velocity, covariance, and timestamp on the base_link coordinate
  - 50 Hz or more
- Accel with covariance stamped
  - Acceleration, covariance, and timestamp on the base_link coordinate
  - 50 Hz or more
- Diagnostics
  - Diagnostics information that indicates if the localization module works properly
- tf
  - tf of map to base_link

In the Autoware implementation, the pose and the twist come together in one `nav_msgs/msg/Odometry` message on `/localization/kinematic_state`. The acceleration is on `/localization/acceleration`. The EKF localizer runs its prediction and publishes the tf at 50 Hz by default (`predict_frequency` and `tf_rate` in `ekf_localizer.param.yaml`). See [5. Interface and data structure](#5-interface-and-data-structure) for the topics.

### Recommended architecture

{{ diagram("image/recommended-architecture.svg", "The recommended architecture, with the Autoware packages that implement each part.") }}

#### Pose estimator

- Estimates the vehicle pose on the map coordinate by matching external sensor observation to the map
- Provides the obtained pose and its covariance (`geometry_msgs/msg/PoseWithCovarianceStamped`) to the pose twist fusion filter

#### Twist-accel estimator

- Produces the vehicle velocity, angular velocity, acceleration, angular acceleration, and their covariances
  - It is possible to create a single module for both twist and acceleration or to create two separate modules. The choice of architecture is up to the developer.
- The twist estimator produces velocity and angular velocity (`geometry_msgs/msg/TwistWithCovarianceStamped`) from internal sensor observation
- The accel estimator produces acceleration and angular acceleration from internal sensor observations

In the Autoware implementation, the twist estimator sends its twist to the fusion filter. The fusion filter does not take an acceleration input. The `autoware_twist2accel` package computes the acceleration (`geometry_msgs/msg/AccelWithCovarianceStamped`) from the change of the velocity, with a low-pass filter. By default (`use_odom: true`), it uses the velocity of the fused kinematic state.

#### Kinematics fusion filter

- Produces the likeliest pose, velocity, acceleration, and their covariances, computed by fusing two kinds of information:
  - The pose obtained from the pose estimator.
  - The velocity and acceleration obtained from the twist-accel estimator
- Produces tf of map to base_link according to the pose estimation result

In Autoware, this part is the pose twist fusion filter. It contains the EKF localizer, the stop filter, and `autoware_twist2accel`.

#### Pose initializer

- Computes the first pose of the vehicle and sends it to the fusion filter
- Uses the GNSS pose, a pose that the user gives, or a pose from the pose estimator (for example an NDT alignment around the GNSS pose)
- Reports the initialization state

#### Localization diagnostics

- Monitors and guarantees the stability and reliability of pose estimation by fusing information obtained from multiple localization modules
- Reports error status to the error monitor

#### TF tree

{{ diagram("image/tf-tree.svg", "The tf tree of Autoware. The localization component publishes the tf from map to base_link.") }}

|   frame   | meaning                                                                                        |
| :-------: | :--------------------------------------------------------------------------------------------- |
|   earth   | ECEF (Earth Centered Earth Fixed)                                                              |
|    map    | Origin of the map coordinate (for example MGRS origin)                                         |
|  viewer   | User-defined frame for rviz                                                                    |
| base_link | Reference pose of the ego-vehicle (projection of the rear-axle center onto the ground surface) |
|  sensor   | Reference pose of each sensor                                                                  |

Developers can optionally add other frames such as odom or base_footprint as long as the tf structure above is maintained.

In Autoware, the EKF localizer publishes the tf from `map` to `base_link`. The `autoware_map_tf_generator` package publishes the `viewer` frame for RViz only. The vehicle description (`robot_state_publisher`) publishes the static tf from `base_link` to the sensor frames. In the sample sensor kit, the sensors are children of `sensor_kit_base_link`, which is a child of `base_link`.

### The localization module's ideal functionality

- The localization module should provide pose, velocity, and acceleration for control, planning, and perception.
- Latency and stagger should be sufficiently small or adjustable such that the estimated values can be used for control within the ODD (Operational Design Domain).
- The localization module should produce the pose on a fixed coordinate frame.
- Sensors should be independent of each other so that they can be easily replaced.
- The localization module should provide a status indicating whether or not the autonomous vehicle can operate with the self-contained function or map information.
- Tools or manuals should describe how to set proper parameters for the localization module
- Valid calibration parameters should be provided to align different frame or pose coordinates and sensor timestamps.

### KPI

To maintain sufficient pose estimation performance for safe operation, the following metrics are considered:

- Safety
  - The distance traveled within the ODD where pose estimation met the required accuracy, divided by the overall distance traveled within the ODD, as a percentage.
  - The anomaly detection rate for situations where the localization module cannot estimate pose within the ODD
  - The accuracy of detecting when the vehicle goes outside of the ODD, as a percentage.
- Computational load
- Latency

## 5. Interface and data structure

The [`autoware_component_interface_specs`](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_component_interface_specs) package of Autoware Core defines the outputs of the component. The [localization interface page](../../interfaces/components/localization.md) describes the message fields.

### Outputs

| Output                   | Topic or frame                       | Type                                                         |
| :----------------------- | :----------------------------------- | :----------------------------------------------------------- |
| Kinematic state          | `/localization/kinematic_state`      | `nav_msgs/msg/Odometry`                                      |
| Acceleration             | `/localization/acceleration`         | `geometry_msgs/msg/AccelWithCovarianceStamped`               |
| Pose with covariance     | `/localization/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped`                |
| Initialization state     | `/localization/initialization_state` | `autoware_adapi_v1_msgs/msg/LocalizationInitializationState` |
| Initialization (service) | `/localization/initialize`           | `autoware_localization_msgs/srv/InitializeLocalization`      |
| Vehicle pose (tf)        | `map` to `base_link`                 | tf                                                           |
| Diagnostics              | `/diagnostics`                       | `diagnostic_msgs/msg/DiagnosticArray`                        |

### Inputs

These are the inputs of the default launch configuration, with NDT as the pose estimator and the gyro odometer as the twist estimator.

| Input                     | Topic                                                       | Type                                                 |
| :------------------------ | :---------------------------------------------------------- | :--------------------------------------------------- |
| LiDAR point cloud         | `/sensing/lidar/concatenated/pointcloud`                    | `sensor_msgs/msg/PointCloud2`                        |
| IMU                       | `/sensing/imu/imu_data`                                     | `sensor_msgs/msg/Imu`                                |
| Vehicle twist             | `/sensing/vehicle_velocity_converter/twist_with_covariance` | `geometry_msgs/msg/TwistWithCovarianceStamped`       |
| GNSS pose                 | `/sensing/gnss/pose_with_covariance`                        | `geometry_msgs/msg/PoseWithCovarianceStamped`        |
| Point cloud map (partial) | `/map/get_differential_pointcloud_map` (service)            | `autoware_map_msgs/srv/GetDifferentialPointCloudMap` |

### Packages

The localization packages are in [Autoware Core](https://github.com/autowarefoundation/autoware_core/tree/main/localization) and [Autoware Universe](https://github.com/autowarefoundation/autoware_universe/tree/main/localization). The launch files are in the `autoware_localization_launch` package of `autoware_launch`. The `component_localization.launch.xml` file of `autoware_launch` selects the estimators with two launch arguments:

- `pose_source`: one or more of `ndt`, `yabloc`, `eagleye`, `artag`, and `lidar-marker`, joined with underscores (for example `ndt_yabloc`). The default is `ndt`. If you select more than one, the pose estimator arbiter runs.
- `twist_source`: `gyro_odom` or `eagleye`. The default is `gyro_odom`.

| Part                     | Package                                                                                                                                                                                                                                                                                                                                                              | Repository |
| :----------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :--------- |
| Pose estimator           | [`autoware_ndt_scan_matcher`](https://autowarefoundation.github.io/autoware_core/main/localization/autoware_ndt_scan_matcher/)                                                                                                                                                                                                                                       | Core       |
| Pose estimator           | [`yabloc`](https://autowarefoundation.github.io/autoware_universe/main/localization/yabloc/) (camera and vector map)                                                                                                                                                                                                                                                 | Universe   |
| Pose estimator           | [`autoware_ar_tag_based_localizer`](https://autowarefoundation.github.io/autoware_universe/main/localization/autoware_landmark_based_localizer/autoware_ar_tag_based_localizer/)                                                                                                                                                                                     | Universe   |
| Pose estimator           | [`autoware_lidar_marker_localizer`](https://autowarefoundation.github.io/autoware_universe/main/localization/autoware_landmark_based_localizer/autoware_lidar_marker_localizer/)                                                                                                                                                                                     | Universe   |
| Pose and twist estimator | [Eagleye](https://github.com/MapIV/eagleye) (GNSS and IMU)                                                                                                                                                                                                                                                                                                           | External   |
| Pose estimator support   | [`autoware_pose_estimator_arbiter`](https://autowarefoundation.github.io/autoware_universe/main/localization/autoware_pose_estimator_arbiter/), [`autoware_pose_covariance_modifier`](https://autowarefoundation.github.io/autoware_universe/main/localization/autoware_pose_covariance_modifier/)                                                                   | Universe   |
| Twist estimator          | [`autoware_gyro_odometer`](https://autowarefoundation.github.io/autoware_core/main/localization/autoware_gyro_odometer/)                                                                                                                                                                                                                                             | Core       |
| Fusion filter            | [`autoware_ekf_localizer`](https://autowarefoundation.github.io/autoware_core/main/localization/autoware_ekf_localizer/), [`autoware_stop_filter`](https://autowarefoundation.github.io/autoware_core/main/localization/autoware_stop_filter/), [`autoware_twist2accel`](https://autowarefoundation.github.io/autoware_core/main/localization/autoware_twist2accel/) | Core       |
| Pose initializer         | [`autoware_pose_initializer`](https://autowarefoundation.github.io/autoware_core/main/localization/autoware_pose_initializer/)                                                                                                                                                                                                                                       | Core       |
| Pose initializer support | [`autoware_pose_initialization_requester`](https://autowarefoundation.github.io/autoware_universe/main/localization/autoware_pose_initialization_requester/) (starts the initialization with GNSS)                                                                                                                                                                   | Universe   |
| Diagnostics              | [`autoware_localization_error_monitor`](https://autowarefoundation.github.io/autoware_universe/main/localization/autoware_localization_error_monitor/), [`autoware_pose_instability_detector`](https://autowarefoundation.github.io/autoware_universe/main/localization/autoware_pose_instability_detector/)                                                         | Universe   |

## 6. Concerns, assumptions, and limitations

### Prerequisites of sensors and inputs

#### Sensor prerequisites

- Input data is not defective.
  - Internal sensor observation such as IMU continuously keeps the proper frequency.
- Input data has correct and exact time stamps.
  - Estimated poses can be inaccurate or unstable if the timestamps are not exact.
- Sensors are correctly mounted with exact positioning and accessible from TF.
  - If the sensor positions are inaccurate, estimation results may be incorrect or unstable.
  - A sensor calibration framework is required to properly obtain the sensor positions.

#### Map prerequisites

- Sufficient information is contained within the map.
  - Pose estimation might be unstable if there is insufficient information in the map.
  - A testing framework is necessary to check if the map has adequate information for pose estimation.
- Map does not differ greatly from the actual environment.
  - Pose estimation might be unstable if the actual environment has different objects from the map.
  - Maps need updates according to new objects and seasonal changes.
- Maps must be aligned to a uniform coordinate, or an alignment framework is in place.
  - If multiple maps with different coordinate systems are used, the misalignment between them can affect the localization performance.

#### Computational resources

- Sufficient computational resources should be provided to maintain accuracy and computation speed.
