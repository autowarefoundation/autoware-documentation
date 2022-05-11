# Advanced usage of colcon

This page demonstrates 

## 1. Prepare your real vehicle hardware

Prerequisites for the vehicle
- Supported vehicle type: car-like or diff-drive vehicle
- Ubuntu20.04 installed
- attached with the following devices:
  + actuator
  + lidar
  + imu (optional)
  + camera (optional)
  + gnss (optional)

TODO: どの機能を使うにはどのセンサが必要か、マトリクスみたいなものがアレば貼り付ける（あるいはリンクを貼る）などしたい
TODO: CUDAに関する記述

software interface for:
- the actuator, like serial, CAN, etc. for driving the vehicle
- the LiDAR, for getting the pointcloud data
-for the above-mentioned devices 

## 2. Create your meta-repository

A recommended way to integrate Autoware with your real robot is to create a meta-repository for the robot. 

Create a fork repository of autowarefoundation/autoware (we refer to this as meta-repository). 

Clone your forked repository
```bash
git clone git@github.com:YOUR_NAME/autoware.YOURS.git
```
Follow the instruction to build the autoware.

```bash
./setup-dev-env.sh
mkdir src
vcs import src < autoware.repos
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 3. Create the description packages of your own vehicle 

Next, you need to create description packages that define the vehicle and sensor configuration of your robot. Once you’re done, you can launch your own robot model by specifying vehicle_model:=YOUR_VEHICLE sensor_model:=SAMPLE_SENSOR_KIT in the autoware launchers.

Create the following two packages
- YOUR_VEHICLE_launch (see  )
- YOUR_SENSOR_KIT_launch (see )

It is recommended you write the above two packages in autoware.repos of your meta-repository.

### Adapt YOUR_VEHICLE for autoware launching system

#### At YOUR_VEHICLE_description

create `urdf/vehicle.xacro` [file name matters]

write “vehicle_info” parameter in config/vehicle_info.param.yaml in the same way as  [file name matters]

also prepare each parameter file following  [file name matters]

#### At YOUR_VEHICLE_launch

create `launch/vehicle_interface.launch` [file name matters]

### Adapt YOUR_SENSOR_KIT for autoware launching system

#### At YOUR_SENSOR_KIT_description

create `urdf/sensors.xacro` [file name matters]

write each parameter file under config/ directory [file name matters] (See )

note that you need to calibrate extrinsic parameters for all the sensors beforehand. calibration_tools from TIER IV support some of the parameter estimation (TODO: calibration_tools.ivへのリンクを貼る)

#### At YOUR_SENSOR_KIT_launch

create `launch/sensing.launch.xml` that launches all the sensors on the vehicle [file name matters]. Refer to  for example.

FYI, you are now able to run planning_simulator at this point. (TODO: put a link to psim here)

## 4. Create a vehicle_interface package

You need to create an interface package for your robot.

The package is expected to provide the following two functions.

1. Receive command messages from vehicle_cmd_gate and drive the robot accordingly

2. Send vehicle status information of the vehicle to autoware

You may refer to pacmod_interface as an example.

TODO: この部分の要件定義が詳細にほしい気がする。

## 5. Create maps

Create pointcloud map

Use 3rd party slam package and get .pcd file. 

Create vector map

Use Vector map builder and get .osm file.

## 6. Launch Autoware
### launch Autoware
```bash
ros2 launch autoware_launch autoware.launch.xml vehicle_model:=YOUR_VEHICLE sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP
```

### Set initial pose

If GNSS is available, it should automatically initialize its pose

If not, you can also set initial pose using GUI on RViz. Click “2D initial pose“ from the top bar, and set the arrow that indicates the robot’s initial pose on your map (x, y, and yaw).

### Set goal pose

One of the methods would be to use GUI to set the goal pose.

### Engage

In your terminal, execute 
```bash
ros2 topic pub /autoware/engage 型忘れた "engage: true" -1
```

You can also use an autoware state panel by adding… TODO：やり方を書く
Now the vehicle should drive the calculated path!

## 7. Tune parameters on your own vehicle & environment

You may need to tune your parameters depending on the domain in which you will operate your robot

Customize parameter files in tier4_*_launch

僕たちの例とかいいかも．obstacle_stop_plannerの例とか，

If you have any issues or questions, feel free to ask in Autoware Foundation Discussion!