LOCALIZATION COMPONENT DESIGN DOC

## Abstract

## 1. Requirements

Localization aims to estimate vehicle pose, velocity, and acceleration.

Goals:

- Proposition of a system that can estimate the vehicle pose, velocity, and acceleration as long as possible
- Proposition of a system that can diagnose the stability of estimation and send a warning message to the error-monitoring system if the estimation result is unreliable
- Vehicle localization function can adequately work with various sensor configurations

Non-goals:

- We do not try to develop a localization system that
  - does not fail in any environment
  - works outside of the pre-defined ODD (Operational Design Domain)
  - has unnecessarily better performance than required in autonomous driving

## 2. Sensor Configuration Examples

In this section, we show example sensor configurations and their expected
performances. Each sensor has advantages and disadvantages. Therefore, by fusing multiple sensors, we can improve the overall performance.

### 3D-LiDAR + PointCloud Map

#### Expected situation

- The vehicle is located in a structure-rich environment, such as an urban area

#### Situation that can make the system unstable

- The vehicle is placed in a structure-less environment, such as rural landscape, highway or a tunnel
- Environment change from when the map is created e.g. construction or destruction of buildings, or snow cover
- Surrounding objects are occluded
- The car is surrounded by objects undetectable by LiDAR, e.g., glass windows, reflections, or absorption (dark objects)
- The environment has laser beams of the same frequency as LiDAR

#### Functionalities

- The system can estimate the vehicle location on the point cloud map with the error of ~10cm
- The system is Operable at the night

### 3D-LiDAR or Camera + Vector Map

#### Expected situation

- Road with a clear white line and loose curvature, such as highway or ordinary local road

#### Situation that can make the system unstable

- White lines are scratchy, or covered by rain or snow
- Tight curvature such as intersections
- Large reflection change of the road surface caused by rain or paint

#### Functionalities

- Correct vehicle positions along the lateral direction
- Pose correction along the longitudinal can be in accurate. But we can resolve this by fusing with GNSS

### GNSS

#### Expected situation

- The vehicle is placed in an open environment with no or few surrounding objects, such as rural landscape

#### Situation that can make the system unstable

- GNSS signals are intercepted by surrounding objects, e.g., tunnels or buildings

#### Functionalities

- The system can estimate the vehicle position with an error of ~10m in the world coordinate
- With RTK attached, the accuracy can be improved to ~10cm
- The system with this configuration can work without environment maps

### Camera (Visual Odometry, Visual SLAM)

#### Expected situation

- The vehicle is placed in a texture-rich environment, such as urban areas

#### Situation that can make the system unstable

- The vehicle is placed in a Texture-less environment
- The vehicle is surrounded by other objects
- The camera observes large illumination changes such as sunshine, headlight, or brigthness change at the exit of a tunnel
- The vehicle is placed in a dark environment

#### Functionalities

- The system can estimated the odometry by tracking visual features

### Wheel speed sensor

#### Expected situation

- The vehicle is running on an ordinary road

#### Situation that can make the system unstable

- The vehicle is running on a slippy or bumpy road
- The vehicle cannot observe the correct wheel speed because of poor road surface conditions such as bumps or slips

#### Functionalities

- The system can acquire the vehicle velocity and estimate the traveled distance
- The system can estimate can estimate a more accurate pose in a higher frequency by fusing with external sensors

<!-- NOTE: Is the second item necessary? Because it is general for any other sensors -->

### IMU

#### Expected environments

- Ordinary roads

<!-- NOTE: The availability of IMU is highest among the sensors attached to vehicles. Therefore we should list more situations or say we can use the IMU almost anywhere -->

#### Situation that can make the system unstable

- IMUs have bias that is dependent on the surrounding temperature. This can cause incorrect sensor observation or odometry drift

#### Functionalities

- The system can observe acceleration and angular velocity.
- By integrating these observation, the system can estimate the local pose change, and realize dead-reckoning
- By fusing with external sensors, it can estimate a more accurate pose in a higher frequency

<!-- NOTE: The third items says "it can estimate a more accurate pose in a higher frequency" but the IMU already has very frequent observation -->

### Geomagnetic sensor

#### Expected situation

- The vehicle is placed in an environment with low magnetic noise

#### Situation that can make the system unstable

- The vehicle is palced in an environment with high magnetic noise, such as buildings or structures with reinforced steel, or materials that generate electromagnetic waves

#### Functionalities

- The system can estimate the vehicle's direction in the world coordinate system

### Magnetic markers

#### Expected environment

- The car is placed in an environment with magnetic markers installed

#### Environment where the system becomes unstable

- The markers are not maintained

#### Functionalities

- Vehicle location can be obtained on the world coordinate by detecting the magnetic markers
- This system can work even if the road is covered with snow

## 3. Requirements

- By allowing to implement different modules, we accept various sensor configurations and algorithms.
- The localization system can start pose estimation from an ambiguous initial location
- The system can produce the reliability of the initial location estimation
- The system can manage the state of the initial location estimation (uninitialized, initializable or not) and can report to the error monitor

## 4. Architecture

### Abstract

We define two architectures: "Required" and "Recommended." Only input and output are defined in the "Required" architecture to accept various kinds of localization algorithms. Also, to improve the reusability of each module, we define more detailed parts in the "Recommended" architecture.

### Required Architecture

![](media/image2.png)

#### Input

- Sensor message
  - e.g., LiDAR, camera, GNSS, IMU, CANBus, etc.
  - Data types should be ROS primitive to keep reusability
- Map data
  - e.g., point cloud map, lanelet2 map, feature map, etc.
  - Choose a map format based on the use case and sensor configuration
  - Map data is not required in some specific cases. (e.g., GNSS-only localization)
- tf, static_tf
  - map frame
  - base_link frame

#### Output

- Pose with covariance stamped
  - Vehicle pose, covariance, and timestamp on the map coordinate
  - 50Hz~ frequency (depends on the requirement by planning/control)
- Twist with covariance stamped
  - Vehicle velocity, covariance, and timestamp on the base_link coordinate
  - 50Hz~ frequency
- Accel with covariance stamped
  - Acceleration, covariance, and timestamp on the base_link coordinate
  - 50Hz~ frequency
- Diagnostics
  - Diagnostics information that indicates if the localization module works properly
- tf
  - tf of map to base_link

<!-- -->

### Recommended Architecture

![](media/image3.png)

#### Pose Estimator

- The pose estimator estimates the vehicle pose on the map coordinate by matching external sensor observation to the map
- The pose estimator also provides the obtained pose and its covariance to `PoseTwistFusionFilter`

#### Twist-Accel Estimator

- The twist-accel estimator produces the vehicle velocity, angular velocity, acceleration, angular acceleration, and their covariances
- Developers can choose the architecture. It is possible to create one module for both twist and acceleration, or also possible to create one module for each
- The twist estimator produces velocity and angular velocity from internal sensor observation
- The accel estimator produces acceleration and angular acceleration from internal sensor observations

#### Kinematics Fusion Filter

- The kinematics fusion filter produces the likeliest pose, velocity, acceleration, and their covariances. This is computed by fusing two information. Ono is the pose obtained from the pose estimator. The other one is velocity and acceleration obtained from the twist-accel estimator
- The kinematics fusion filter also produces tf of map to base_link according to the pose estimation result

#### Localization Diagnostics

- The diagnostics module monitors and guarantees the stability and reliability of pose estimation by fusing information obtained from multiple localization modules
- The diagnostics module reports the error status to the error monitor

<!-- -->

#### TF tree

![](media/image1.png){width="2.6666666666666665in" height="2.8125in"}

|   frame   | meaning                                                                                        |
| :-------: | :--------------------------------------------------------------------------------------------- |
|   earth   | ECEF (Earth Centered Earth Fixed）                                                             |
|    map    | Origin of the map coordinate (ex. MGRS origin)                                                 |
|  viewer   | User-defined frame for rviz                                                                    |
| base_link | Reference pose of the ego-vehicle (projection of the rear-axle center onto the ground surface) |
|  sensor   | Reference pose of each sensor                                                                  |

Developers can optionally add other frames such as odom or base_footprint while keeping the tf structure above

### The localization module's ideal functionality

- The localization module should provide pose, velocity, and acceleration for control, planning, and perception
- Latency and stagger should be sufficiently small or adjustable so that the estimated values can be used for control within the ODD(Operational Design Domain)
- The localization module should produce the pose on a fixed coordinate frame
- Sensors should be independent of each other and keep the replaceability
- The localization module should provide the status indicating if the autonomous vehicle can operate or not with the self-contained function or map information.
- Tools or manuals should describe how to set proper parameters for the localization module <!-- TODO  原文にある「適切な位置推定」とはなんだ！！！ -->
- If provided maps or estimated poses for localization modules have different reference coordinate frames or distortion between each map or pose, valid transfer evaluation functions should be provided. This checks if the map has sufficient information to realize stable and reliable localization.
- If the map information should be supplied to absorb them

<!-- -->

### KPI

- If the vehicle keeps sufficient pose estimation for safe operation, we jointly consider these metrics
- The percentage of distance which the vehicle pose estimation satisfies the required accuracy to the overall distances defined in ODD
- The anomaly detection rate of when the localization module cannot estimate the pose within ODD
- The accuracy of out-of-ODD detection
- Additionally, we should save computational load

## 5. Interface and Data Structure

## 6. Concerns, Assumptions, and Limitations

### Prerequisites of sensors and inputs

#### Sensor prerequisites

- Input data is not defective. Internal sensor observation such as IMU continuously keeps the proper frequency
- Input data has correct and exact time stamps. Estimated poses might be inaccurate or unstable if the timestamps are not exact
- Sensors are correctly mounted on exact positions and accessible from TF. If the sensor positions are inaccurate, estimation results may be incorrect or unstable. To properly obtain sensor positions, a sensor calibration framework is going to be required.

#### Map prerequisites

- Pose estimation might be unstable If there is no sufficient information necessary on the map. A testing framework is demanded to check if the map has sufficient information for pose estimation.
- Pose estimation might be unstable if the actual environment has different objects than the map. Maps need updates according to new objects and seasonal changes.
- If multiple maps or maps that have different coordinate systems each other are used, the misalignment between them can affect the localization performance. Maps have to be aligned to a uniform coordinate or require a system for alignment.

#### Computational resources

- Sufficient computational resources should be provided to keep the accuracy and computation speed.
