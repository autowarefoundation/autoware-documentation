LOCALIZATION COMPONENT DESIGN DOC

## Abstract

## 1. Requirements

The purpose of localization is to estimate vehicle pose, velocity, and acceleration.

Goals:

* Proposition of a system that can estimate the vehicle pose, velocity and acceleration as long as possible
* Proposition of a system that can diagnose the stability of estimation and send a warning message to the error-monitoring system if the estimation result is unreliable
* Vehicle localization function can properly work with various sensor configurations

Non-goals:

* We do not try to develop a localization system that
  * does not fail in any environment
  * works outside of pre-defined ODD (Operational Design Domain)
  * has unnecessarily better performance than required in autonomous driving

## 2. Sensor Configuration Examples

In this section we show example sensor configurations and their expected
performances. Each sensor has advantages and disadvantages, therefore by
fusing multiple sensors we can improve the overall performance.

* 3D-LiDAR + PointCloud Map
* Expected environment
  * Structure-rich environment
    * e.g. urban areas
* Environment that the system becomes unstable
  * Structure-less environment
    * e.g. rural landscapes, highways, tunnels
  * Environment that has changes from when the map is created
    * e.g. construction or destruction of buildings, snow cover
  * Occlusion by surrounding objects
  * Surrounded by objects undetectable by LiDAR
    * Glass windows, reflections, absorption (dark objects)
  * Environment with lasers in the same frequency as LiDAR
* Functionalities
  * The system can estimate the vehicle location on the pointcloud map with the error of \~10cm
  * Operable in the night

*   3D-LiDAR or Camera + Vector Map
  * Expected environment
    * Road with a clear white line and loose curvature
      * e.g. highways, ordinary local roads
  * Environment that the system becomes unstable
    * Less or blurred white lines, or covered by rain or snow
    * Tight curvature such as intersections
    * Large reflection change of the road surface because of rain or paint
  * Functionalities
    * Correct vehicle positions along the lateral direction
    * Pose correction along the longitudinal becomes uncertain. This can be improved by fusing with GNSS

* GNSS
  * Expected environment
    * Open environment with no or few surrounding objects
      * e.g. rural landscapes
  * Environment that the system becomes unstable
    * Environments where GNSS signals are intercepted
      * e.g. tunnels, urban areas
  * Functionalities
    * The system can estimate the vehicle position with an error of \~10m in the world coordinate
    * With RTK attached, the accuracy can be improved to \~10cm
    * This can work without environmental maps

* Camera (Visual Odometry, Visual SLAM)
  * Expected environment
    * Texture-rich environment
      * Urban areas
  * Environment that the system becomes unstable
    * Texture-less environment
    * Surrounded by other objects
    * Large illumination changes
      * e.g. sunshine, headlight, exit of tunnels
    * Insufficient illumination
  * Functionalities
    * Odometry can be estimated by tracking visual features

* Wheel speed sensor
  * Expected environment
    * Ordinary roads
  * Environment that the system becomes unstable
    * Slippy or bumpy roads, or places where wheel speed cannot be correctly obtained
  * Functionalities
    * It can obtain the vehicle velocity and estimate the distance traveled
    * By fusing with external sensors, it can estimate more accurate pose in a higher frequency

* IMU
  * Expected environments
    * Ordinary roads
  * Environment that the system becomes unstable
    * Temperature change
  * Functionalities
    * It can obtain acceleration and angular velocity, and can calculate the local vehicle rotation change by dead-reckoning
    * By fusing with external sensors, it can estimate more accurate pose in a higher frequency

* Geomagnetic sensor
  * Expected environment
    * Environment with low magnetic noise
  * Environment that the system becomes unstable
    * Environment with high magnetic noise
      * e.g. buildings or structures with reinforced steel, materials that generate electromagnetic waves
  * Functionalities
    * Direction can be estimated in the world coordinate system

* Magnetic markers
  * Expected environment
    * Environment with magnetic markers installed
  * Environment that the system becomes unstable
    * Environment with markers installed but without maintenance
  * Functionalities
    * Vehicle location can be obtained on the world coordinate by detecting the magnetic markers
    * This system can work even if the road is covered with snow

## 3. Requirements

* By allowing to implement different modules, we accept various sensor configurations and algorithms.
* The localization system can start the pose estimation from an ambiguous initial location
* The system can produce the reliability of the initial location estimation
* The system can manage the state of the initial location estimation (uninitialized, Initializable or not) and can report to the error monitor

## 4. Architecture

### Abstract
 We define two architectures: "Required" and "Recommended". Only input and output are defined in the "Required" architecture to accept various kinds of localization algorithms. Also, to improve reusability of each module, we define more detailed parts in the "Recommended" architecture.

### Required Architecture

![](media/image2.png)

* Input
  * sensor message
    * e.g., LiDAR, camera, GNSS, IMU, CANBus, etc.
    * To keep reusability, data types should be ROS primitive
  * map data
    * e.g., pointcloud map, lanelet2 map, feature map, etc.
    * Choose a map format based on use case and sensor configuration
    * Map data is not required in some specific cases. (e.g. GNSS-only localization)
  * tf, static\_tf
    * map frame
    * base\_link frame

* Output
  * pose with covariance stamped
    * Vehicle pose, covariance and timestamp on the map coordinate
    *   50Hz\~ frequency (depends on the requirement by planning/control)
  * twist with covariance stamped
    * Vehicle velocity, covariance, and timestamp on the base\_link coordinate
    *   50Hz\~ frequency
  * accel with covariance stamped
    * Acceleration, covariance and timestamp on the base\_link coordinate
    *   50Hz\~ frequency
  * diagnostics
    * Diagnostics information of whether the localization module works properly or not
  * tf
    * tf of map to base\_link

<!-- -->

### Recommended Architecture

![](media/image3.png)

* Pose Estimator
  * Estimate the vehicle pose on the map coordinate by matching external sensor observation to the map
  * Provite the obtained pose and its covariance to PoseTwistFusionFilter

* Twist-Accel Estimator
  * Produce vehicle velocity, angular velocity, acceleration, angular acceleration, and their covariances
  * Developers can create one module for both twist and acceleration or can create one module for each of them
    * Twist estimator
      * Produce velocity and angular velocity from internal sensor observation
    * Accel estimator
      * Produce acceleration and angular acceleration from internal sensor observations

* Kinematics Fusion Filter
  * Produce the most likely pose, velocity, acceleration, and their covariances by fusing the pose obtained from the pose estimator and velocity and acceleration obtained from the twist-accel estimator
  * Produce tf of map to base\_link according to the pose estimation result

* Localization Diagnostics
  * Monitor and guarantee the stability and reliability of pose estimation by fusing information obtained from multiple localization modules and report the status to the error monitor

<!-- -->

* TF tree
 ![](media/image1.png){width="2.6666666666666665in" height="2.8125in"}

* earth
  * ECEF (Earth Centered Earth Fixed）

* map
  * Origin of the map coordinate (ex. MGRS origin)

* viewer
  * User-defined frame for rviz

* base\_link
  * Reference pose of the ego-vehicle (projection of the rear-axle center onto the ground surface)

* sensor\*
  * Reference pose of each sensor
     Developers can optionally add other frames such as odom or base\_footprint while keeping the tf structure above 

### Ideal localization functionality

* Localization module should provide pose, velocity, and acceleration for control, planning, and perception
* Latency and stagger should be sufficiently small or adjustable so that the estimated values can be used for control in ODD(Operational Design Domain)
* Localization module should produce the pose on a fixed coordinate frame
* Sensors should be independent to each other and keep the replaceability
* Localization modules should output status which the autonomous vehicle can operate or cannot operate using self-contained function or map information.
* The ways to set proper parameters for the localization module should be provided by tools or manuals with proper criteria.
* If provided maps or estimated poses for localization modules have different reference coordinate frames or distortion between each map or pose, valid transfer evaluation functions should be provided. This checks if the map has sufficient information to realize stable and reliable localization.
* If the map informations should be supplied to absorb them

<!-- -->

* KPI
  * If the vehicle keeps sufficient pose estimation for safe operation, we jointly consider these metrics
    * The percentage of distance which the vehicle pose estimation satisfies the required accuracy to the overall distances defined in ODD
    * The anomaly detection rate of when the localization module cannot estimate the pose within ODD
    * The accuracy of out-of-ODD detection
  * Additionally, we should save computational load

## 5. Interface and Data Structure

## 6. Concern, Assumption, and Limitation

* Prerequisites of sensors and inputs

* Sensor prerequisites
  * Input data is not defective. Internal sensor observation such as IMU continuously keeps proper frequency
  * Input data has correct and exact time stamps. Estimated poses might be inaccurate or unstable if the timestamps are not exact
  * Sensors are correctly mounted on exact positions and accessible from TF. If the sensor positions are inaccurate, estimation results may be incorrect or unstable. To properly obtain sensor positions, sensor calibration framework is going to be required.

* Map prerequisites
  * Pose estimation might be unstable If there is no sufficient information necessary on the map. Testing framework is demanded to check if the map has sufficient information for pose estimation.
  * Pose estimation might be unstable if the actual environment has different objects than the map. Maps need updates according to new objects and seasonal changes.
  * If multiple maps or maps that has different coordinate system each other are used, the mis-alignment between them can affect the localization performance. Maps have to be aligned to a uniform coordinate or require a system for alignment.

* Computational resources
  * Sufficient computational resources should be provided to keep the accuracy and computation speed.
