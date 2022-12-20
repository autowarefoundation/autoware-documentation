# Conclusions

As a conclusion, lidar odometry drifts accumulatively as time goes by and there is solutions to solve that problem such as graph optimization, loop closure and using gps sensor to decrease accumulative drift error. Because of that, a SLAM algorithm should have loop closure feature, graph optimization and should use gps sensor. Additionally, some of the algorithms are using IMU sensor to add another factor to graph for decreasing drift error. While some of the algorithms requires 9-axis IMU sensor strictly, some of them requires only 6-axis IMU sensor or not even using the IMU sensor. Before choosing an algorithm to create maps for Autoware please consider these factors depends on your sensor setup or expected quality of generated map.

<br>
<br>

**FAST_LIO_LC** is is based on FAST-LIO which is a computationally efficient and robust LiDAR-inertial odometry package. FAST-LIO fuses LiDAR feature points with IMU data using a tightly-coupled iterated extended Kalman filter. But it doesn't have a loop closure module to eliminate the accumulated drift. Therefore, **FAST_LIO_LC** implements the pose graph optimization with a radius-search-based loop closure module and the pose and map in the iterated extended Kalman filter of FAST-LIO will be updated according to the optimization which is a key difference with FAST_LIO. Additionally, **FAST_LIO_LC** has loop closure feature and using gps messages on graph optimization to decrease accumulative erros. Also it supports multiple lidar types and requires 6-axis or 9-axis imu which means that this algorithm can be used with both internal lidar imu or external imu.

<br>

**FAST_LIO_SLAM** is the integration of FAST_LIO and SC-PGO which is scan context based loop detection and GTSAM based pose-graph optimization. This algorithm supports multiple lidar types and works with both 6-axis and 9-axis imu sensors. Also GPS sensor is available to use for graph optimization (optional).

<br>

**hdl_graph_slam** is an open source ROS package for real-time 6DOF SLAM using a 3D LIDAR. It is based on 3D Graph SLAM with NDT scan matching-based odometry estimation and loop detection. It also supports several graph constraints, such as GPS, IMU acceleration (gravity vector), IMU orientation (magnetic sensor), and floor plane (detected in a point cloud). Additionally, it supports other scan registration methods such as ICP, GICP, FAST_VGICP, FAST_VGICP_CUDA. On the other hand, it requires too much parameters for scan registration and graph optimization which may assist the algorithm to generate high quality map, but also it might be confusing to set correct parameters.

<br>

**FD_SLAM** is Feature&Distribution-based 3D LiDAR SLAM method based on Surface Representation Refinement. In this algorithm novel feature-based Lidar odometry used for fast scan-matching, and used a proposed UGICP method for keyframe matching. Rest of the algorithm is based on hdl_graph_slam.

<br>

**IA_LIO_SLAM** is created for data acquisition in unstructured environment and it is a framework for Intensity and Ambient Enhanced Lidar Inertial Odometry via Smoothing and Mapping that achieves highly accurate robot trajectories and mapping. **IA_LIO_SLAM** utilizes a factor graph same as LIO-SAM. Enhancing the existing LIO-SAM, **IA_LIO_SLAM** leverages point’s intensity and ambient value to remove unnecessary feature points. Also this algorithm supports Velodyne and Ouster lidars. Additionally, it has loop closure feature and it uses gps messages for graph optimization to decrease accumulative errors (optionally).

<br>

**ISCLOAM** presents a robust loop closure detection approach by integrating both geometry and intensity information. Experiments in the original repository have been conducted including local run test with autonomous warehouse robot and public dataset test to evaluate this algorithm. The results show that this algorithm achieves competitive recall precision and recall rate compared to the state-of-the-art methods. Also this algorithm has loop closure feature and only requires lidar sensor to run. On the other hand, this algorithm is not using GPS sensor and this may cause altitude errors on the generated map.

<br>

**LeGO-LOAM-BOR** is improved version of the LeGO-LOAM by improving quality of the code, making it more readable and consistent. Also, performance is improved by converting processes to multi-threaded approach. As an additional feature it has loop closure feature but it only supports Velodyne VLP-16 lidar to work with.

<br>

**LIO-SAM** is a framework that achieves highly accurate, real-time mobile robot trajectory estimation and map-building. It formulates lidar-inertial odometry atop a factor graph, allowing a multitude of relative and absolute measurements, including loop closures, to be incorporated from different sources as factors into the system. The estimated motion from IMU sensor de-skews point clouds and produces an initial guess for lidar odometry optimization. Also the obtained lidar odometry solution is used to estimate the bias of the IMU. Additionally it has loop closure feature and supports multiple types of lidar sensors and gps sensor.

<br>

**Optimized-SC-F-LOAM** is an improved version of F-LOAM and uses an adaptive threshold to further judge the loop closure detection results and reducing false loop closure detections. Also it uses feature point-based matching to calculate the constraints between a pair of loop closure frame point clouds and decreases time consumption of constructing loop frame constraints. On the other hand, it only supports Velodyne lidars and not using gps sensor.

<br>

**SC-A-LOAM** is a real-time LiDAR SLAM package that integrates A-LOAM and ScanContext. A-LOAM is used for odometry with consecutive motion estimation. Also GTSAM is used for pose-graph optimization and ScanCOntext is used for coarse global localization that can deal with big drifts. It supports Velodyne lidars mainly, but other lidar sensors can be integrated easily.

<br>

**SC-LeGO-LOAM** integrated LeGO-LOAM for lidar odometry and 2 different loop closure methods: ScanContext and Radius search based loop closure. While ScanContext is correcting large drifts, Radius search based method is good for fine-stitching.
