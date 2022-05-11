# How to integrate Autoware with your own vehicle

## 1. Prepare your real vehicle hardware

Prerequisites for the vehicle:

- supported vehicle type: car-like or diff-drive vehicle
- Ubuntu20.04 installed
- attached with the following devices:
  - actuator
  - lidar
  - imu (optional)
  - camera (optional)
  - gnss (optional)
- NVidia GPU (optional)
  TODO: CUDA に関する記述

software interface for:

- the actuator, like serial, CAN, etc. for driving the vehicle
- the LiDAR, for getting the pointcloud data
  -for the above-mentioned devices

## 2. Create maps

You also need both pointcloud and vector map to take full advantage of Autoware. Since SLAM algorithm is not implemented in the current Autoware, you may need to use 3rd party tools for this step.

### Create pointcloud map

Use 3rd party tools such as LiDAR-based SLAM (Simultaneous Localization and Mapping) package to create pointcloud map. Autoware supports .pcd format for the map.

### Create vector map

Autoware supports lanelet2 format for a vector map. Use 3rd party tools or [Vector Map Builder](https://tools.tier4.jp/) and get .osm file.

## 3. Create your meta-repository

A recommended way to integrate Autoware with your real robot is to create a meta-repository for the robot. Create a fork repository of autowarefoundation/autoware (we refer to this as meta-repository).

Clone your forked repository

```bash
git clone git@github.com:YOUR_NAME/autoware.YOURS.git
```

## 4. Create the description packages of your own vehicle

Next, you need to create description packages that define the vehicle and sensor configuration of your robot. Once you’re done, you can launch your own robot model by specifying vehicle_model:=YOUR_VEHICLE sensor_model:=SAMPLE_SENSOR_KIT in the autoware launchers.

Create the following two packages:

- YOUR_VEHICLE_launch (see [here](https://github.com/autowarefoundation/sample_vehicle_launch) for example)
- YOUR_SENSOR_KIT_launch (see [here](https://github.com/autowarefoundation/sample_sensor_kit_launch) for example)

It is recommended you write the above two packages in `autoware.repos` file of your meta-repository.

### Adapt YOUR_VEHICLE for autoware launching system

#### At YOUR_VEHICLE_description

Create `urdf/vehicle.xacro`.

Write “vehicle_info” parameter in config/vehicle_info.param.yaml in the same way as...

Also, prepare each parameter file following [file name matters]

#### At YOUR_VEHICLE_launch

Create `launch/vehicle_interface.launch`

### Adapt YOUR_SENSOR_KIT for autoware launching system

#### At YOUR_SENSOR_KIT_description

Create `urdf/sensors.xacro` [file name matters]

Write each parameter file under config/ directory [file name matters] (See )

Note that you need to calibrate extrinsic parameters for all the sensors beforehand.

#### At YOUR_SENSOR_KIT_launch

Create `launch/sensing.launch.xml` that launches all the sensors on the vehicle [file name matters]. Refer to for example.

At this point, you are now able to run planning_simulator.
If you want to try, you may install Autoware (follow [here](https://autowarefoundation.github.io/autoware-documentation/pr-86/installation/autoware/)) and run the following command:

```bash
ros2 launch autoware_launch planning_simulator.launch.xml vehicle_model:=YOUR_VEHICLE sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP
```

TODO: Put pic here

## 5. Create a vehicle_interface package

You need to create an interface package for your robot.

The package is expected to provide the following two functions.

1. Receive command messages from vehicle_cmd_gate and drive the robot accordingly

2. Send vehicle status information of the vehicle to autoware

You may refer to [pacmod_interface](https://github.com/tier4/pacmod_interface) as an example.

## 6. Launch Autoware

### Install Autoware

Follow the step [here](https://autowarefoundation.github.io/autoware-documentation/pr-86/installation/autoware/).

```bash
./setup-dev-env.sh
mkdir src
vcs import src < autoware.repos
source /opt/ros/galactic/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Launch Autoware

Launch autoware with the following command:

```bash
ros2 launch autoware_launch autoware.launch.xml vehicle_model:=YOUR_VEHICLE sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP
```

### Set initial pose

If GNSS is available, it should automatically initialize its pose.
If not, you can also set initial pose using GUI on RViz. Click “2D initial pose“ from the top bar, and set the arrow that indicates the robot’s initial pose on your map (x, y, and yaw).

### Set goal pose

One of the methods would be to use GUI to set the goal pose.

### Engage

In your terminal, execute the following command.

```bash
ros2 topic pub /autoware/engage 型忘れた "engage: true" -1
```

You can also use an autoware state panel by adding… TODO：やり方を書く

Now the vehicle should drive the calculated path!

## 7. Tune parameters on your own vehicle & environment

You may need to tune your parameters depending on the domain in which you will operate your robot

Customize parameter files in tier4\_\*\_launch

僕たちの例とかいいかも．obstacle_stop_planner の例とか，

If you have any issues or questions, feel free to ask in Autoware Foundation Discussion!
