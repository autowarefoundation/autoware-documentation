# FD-SLAM


## What is FD-SLAM?

- FD_SLAM is Feature&Distribution-based 3D LiDAR SLAM method based on Surface Representation Refinement. In this algorithm novel feature-based Lidar odometry used for fast scan-matching, and used a proposed UGICP method for keyframe matching.


## Repository Information

This is an open source ROS package for real-time 6DOF SLAM using a 3D LIDAR.

It is based on hdl_graph_slam and the steps to run our system are same with hdl-graph-slam.

### Original Repository link

[https://github.com/SLAMWang/FD-SLAM](https://github.com/SLAMWang/FD-SLAM)

### Required Sensors

- LIDAR[VLP-16, HDL-32, HDL-64, OS1-64]
- GPS
- IMU [Optional]

### ROS Compatibility

- ROS 1

### Dependencies

- [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [PCL](https://pointclouds.org/downloads/#linux)
- [g2o](http://wiki.ros.org/g2o)
- [Suitesparse](https://github.com/ethz-asl/suitesparse)

The following ROS packages are required:

- geodesy
- nmea_msgs
- pcl_ros
- [ndt_omp](https://github.com/koide3/ndt_omp)
- [U_gicp](https://github.com/SLAMWang/UGICP) This is modified based on [fast_gicp](https://github.com/SMRT-AIST/fast_gicp) by us. We use UGICP for keyframe matching.

## Build & Run

### 1) Build

```bash
cd ~/catkin_ws/src
git clone https://github.com/SLAMWang/FD-SLAM.git
cd ..
catkin_make
```

### 2) Services

```bash
/hdl_graph_slam/dump  (hdl_graph_slam/DumpGraph)
  - save all the internal data (point clouds, floor coeffs, odoms, and pose graph) to a directory.

/hdl_graph_slam/save_map (hdl_graph_slam/SaveMap)
  - save the generated map as a PCD file.
```

### 3) Set parameters

- All the configurable parameters are listed in _launch/\*\*\*\*.launch_ as ros params.

### 4) Run

```bash
source devel/setup.bash
roslaunch hdl_graph_slam hdl_graph_slam_400_ours.launch
```
