# IA-LIO-SAM


## Repository Information


### Original Repository link
https://github.com/minwoo0611/IA_LIO_SAM

### Required Sensors
- LIDAR [Velodyne, Ouster]
- IMU [6-AXIS, 9-AXIS]
- GNSS
- Odometry Sensor

### ROS Compatibility
- ROS 1

### Dependencies
- [ROS](http://wiki.ros.org/ROS/Installation) (tested with Kinetic and Melodic) 
  - `for ROS melodic:`
    ```
    sudo apt-get install -y ros-melodic-navigation
    sudo apt-get install -y ros-melodic-robot-localization
    sudo apt-get install -y ros-melodic-robot-state-publisher
    ```
  - `for ROS kinetic:`
    ```
    sudo apt-get install -y ros-kinetic-navigation
    sudo apt-get install -y ros-kinetic-robot-localization
    sudo apt-get install -y ros-kinetic-robot-state-publisher
    ```
- [GTSAM](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library)

  ```
  wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
  cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
  cd ~/Downloads/gtsam-4.0.2/
  mkdir build && cd build
  cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
  sudo make install -j8
  ```

## Build & Run
### 1) Build
```
    mkdir -p ~/catkin_ia_lio/src
    cd ~/catkin_ia_lio/src
    git clone https://github.com/minwoo0611/IA_LIO_SAM
    cd ..
    catkin_make
```
### 2) Set parameters

- After downloading the repo, we will replace the lidar topic name from the config file (```~/catkin_ia_lio/src/IA_LIO_SAM/config/params.yaml```) with the lidar topic name in our bag file. For this, we first need to learn the name of the topic in our bag file.   
 
   ### Getting topic name from bag file
        # Open terminal
        - rosbag info ~/.../bagname.bag
        Look at the lidar directory under the topic title.

 <img src="images/bag_info.png" width=50%><img src="images/config_info.png" width=50% >

 - For imu-lidar compatibility, extrinsic matrices from calibration must be changed.

<p align="center"> <img src="images/extrinsic.gif" alt="Extrinsic Matrices GIF"></p>

  - To enable autosave, ```savePCD``` must be ```true``` from the ```params.yaml``` file (```~/catkin_ia_lio/src/IA_LIO_SAM/config/params.yaml```).

<p align="center"> <img src="images/auto-save.png" alt="Auto Save"></p>

(Do not forget to rebuild after setting parameters.)

### 3) Run

      # open new terminal: run IA_LIO
      source devel/setup.bash
      roslaunch lio_sam mapping_ouster64.launch 

      # play bag file in the other terminal
      rosbag play RECORDED_BAG.bag --clock  

<p align="center"> <img src="images/launch.gif" alt="Launch GIF"></p>

## Example Result
<p align="center"> <img src="images/ia_lio_example1.gif" alt="example_results1"></p>

## Sample dataset images


<p align='center'>
    <img src="images/Seq2.png" alt="drawing" width="800"/>
    <img src="images/Sejong_tunnel_data_lio_ai_lio.png" alt="drawing" width="800"/>
    <img src="images/Sejong_tunnel_data.png" alt="drawing" width="800"/>
</p>

## Example dataset 
Check original repo link for example dataset.   

## Contact 
- Maintainer: Kevin Jung (`Github: minwoo0611`)

## Paper 

Thank you for citing IA-LIO-SAM(./config/doc/KRS-2021-17.pdf) if you use any of this code. 


```
TODO
```

Part of the code is adapted from [LIO-SAM (IROS-2020)](https://github.com/TixiaoShan/LIO-SAM).
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
  - IA-LIO-SAM is based on LIO-SAM (T. Shan, B. Englot, D. Meyers, W. Wang, C. Ratti, and D. Rus. LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping).
- Thanks for [LIO-SAM (IROS-2020)](https://github.com/TixiaoShan/LIO-SAM) authors.
