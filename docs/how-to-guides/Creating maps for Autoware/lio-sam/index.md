# LIO_SAM


## Repository Information


### Original Repository link
https://github.com/TixiaoShan/LIO-SAM

### Required Sensors
- LIDAR [Livox, Velodyne, Ouster]
- IMU [9-AXIS]
- GPS [OPTIONAL]

<p align='center'>
    <img src="images/system.png" alt="drawing" width="800"/>
</p>

### ROS Compatibility
- ROS 1
- [For ROS 2](https://github.com/TixiaoShan/LIO-SAM/tree/ros2)

### Dependencies
- Ubuntu 18.04
- ROS
- PCL
- Gtsam


  ```
  sudo apt-get install -y ros-melodic-navigation
  sudo apt-get install -y ros-melodic-robot-localization
  sudo apt-get install -y ros-melodic-robot-state-publisher
  ```
- [gtsam](https://gtsam.org/get_started/) (Georgia Tech Smoothing and Mapping library)
  ```
  sudo add-apt-repository ppa:borglab/gtsam-release-4.0
  sudo apt install libgtsam-dev libgtsam-unstable-dev
  

## Build & Run
### 1) Build
```
    mkdir -p ~/catkin_lio_sam/src
    cd ~/catkin_lio_sam/src
    git clone https://github.com/TixiaoShan/LIO-SAM.git
    cd .. 
    catkin_make
    source devel/setup.bash
```

### 2) Set parameters
- Set imu and lidar topic on `lio_sam/config/params.yaml`


### 3) Run
```
    # Run the Launch File
      roslaunch lio_sam run.launch

    # Play bag file in the other terminal
      rosbag play xxx.bag --clock 
```


## Example Result
<p align='center'>
    <img src="images/pcd-map.png" alt="drawing" width="600"/>
    <img src="images/mapping-demo.gif" alt="drawing" width="600"/>
</p>


## Other Examples
<p align='center'>
    <img src="images/livox-demo.gif" alt="drawing" width="600"/>


## Paper 

Thank you for citing [LIO-SAM (IROS-2020)](./config/doc/paper.pdf) if you use any of this code. 
```
@inproceedings{liosam2020shan,
  title={LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping},
  author={Shan, Tixiao and Englot, Brendan and Meyers, Drew and Wang, Wei and Ratti, Carlo and Rus Daniela},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={5135-5142},
  year={2020},
  organization={IEEE}
}
```

Part of the code is adapted from [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM).
```
@inproceedings{legoloam2018shan,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Shan, Tixiao and Englot, Brendan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```


## Acknowledgements 
  - LIO-SAM is based on LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time).
