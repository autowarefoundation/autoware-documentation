# How to integrate Autoware with your vehicle

This page demonstrates how to intagrate Autoware with the real vehicle.

## 1. Prepare your real vehicle hardware

Prerequisites for the vehicle:

- car-like or diff-drive vehicle
- onboard computer that satisfies the prerequisites (see [here](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/#prerequisites))
- the following devices attached
  - actuator
  - lidar
  - imu (optional)
  - camera (optional)
  - gnss (optional)

## 2. Create maps

You need both a pointcloud and a vector map to take full advantage of Autoware. Since mapping algorithm such as SLAM (simultaneous localization and mapping) is not implemented in the current Autoware, you may need to use 3rd party tools for this step.

### Create a pointcloud map

Use 3rd party tools such as the LiDAR-based SLAM package to create a pointcloud map. Autoware supports the .pcd format for this map.

### Create vector map

Autoware supports lanelet2 format for a vector map. Use 3rd party tools or [Vector Map Builder](https://tools.tier4.jp/) to get the .osm file.

## 3. Create your meta-repository

A recommended way to integrate Autoware with your real robot is to create a meta-repository for the robot. Create a forked repository of [autowarefoundation/autoware](https://github.com/autowarefoundation/autoware) (we refer to this as meta-repository) and clone the repository.

```bash
git clone git@github.com:YOUR_NAME/autoware.YOURS.git
```

## 4. Create the description packages of your vehicle

Next, you need to create description packages that define the vehicle and sensor configuration of your robot.
Once it is done, you can launch your robot model by specifying vehicle_model:=YOUR_VEHICLE sensor_model:=SAMPLE_SENSOR_KIT in the autoware launchers.

Create the following two packages:

- YOUR_VEHICLE_launch (see [here](https://github.com/autowarefoundation/sample_vehicle_launch) for example)
- YOUR_SENSOR_KIT_launch (see [here](https://github.com/autowarefoundation/sample_sensor_kit_launch) for example)

It is recommended you write the above two packages in `autoware.repos` file of your meta-repository.

### Adapt YOUR_VEHICLE for autoware launching system

#### At YOUR_VEHICLE_description

Define URDF and parameters in the package (see [here](https://github.com/autowarefoundation/sample_vehicle_launch/sample_vehicle_description) for example).

#### At YOUR_VEHICLE_launch

Create a launch file (see [here](https://github.com/autowarefoundation/sample_vehicle_launch/sample_vehicle_launch) for example).
If you have multiple vehicles with similar hardware setup, you can specify `vehicle_id` to distinguish them.

### Adapt YOUR_SENSOR_KIT for autoware launching system

#### At YOUR_SENSOR_KIT_description

Define URDF and extrinsic parameters for all the sensors here (see [here](https://github.com/autowarefoundation/sample_sensor_kit_launch/sample_sensor_kit_description) for example).
Note that you need to calibrate extrinsic parameters for all the sensors beforehand.

#### At YOUR_SENSOR_KIT_launch

Create `launch/sensing.launch.xml` that launches the interfaces of all the sensors on the vehicle. (see [here](https://github.com/autowarefoundation/sample_sensor_kit_launch/sample_sensor_kit_launch) for example).



At this point, you are now able to run planning_simulator.
If you want to try, you may install Autoware (follow [here](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/)) and run the following command:

```bash
ros2 launch autoware_launch planning_simulator.launch.xml vehicle_model:=YOUR_VEHICLE sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP
```

## 5. Create a `vehicle_interface` package

You need to create an interface package for your robot.
The package is expected to provide the following two functions.

1. receive command messages from `vehicle_cmd_gate` and drive the robot accordingly
2. send vehicle status information of the vehicle to Autoware

You can find detailed information about the requirements of `vehicle_interface` [here](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-interface/).
You can also refer to [pacmod_interface](https://github.com/tier4/pacmod_interface) as an example.

## 6. Launch Autoware

This section briefly explains how to run your vehicle with Autoware.

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
If not, you can also set the initial pose using GUI on RViz.

1. Click the 2D Pose estimate button in the toolbar, or hit the P key
2. In the 3D View pane, click and hold the left mouse button, and then drag to set the direction for the initial pose.

### Set goal pose

Set a goal pose for the ego vehicle.

1. Click the 2D Nav Goal button in the toolbar, or hit the G key
2. In the 3D View pane, click and hold the left mouse button, and then drag to set the direction for the goal pose.
   If successful, you will see the calculated planning path on RViz.

### Engage

In your terminal, execute the following command.

```bash
source ~/autoware/install/setup.bash
ros2 topic pub /autoware/engage autoware_auto_vehicle_msgs/msg/Engage "engage: true" -1
```

You can also engage via RViz with "AutowareStatePanel".
The panel can be found in Panels > Add New Panel > tier4_state_rviz_plugin > AutowareStatePanel.

Now the vehicle should drive the calculated path!

## 7. Tune parameters for your vehicle & environment

You may need to tune your parameters depending on the domain in which you will operate your robot.

If you have any issues or questions, feel free to ask in [Autoware Foundation Discussion](https://github.com/orgs/autowarefoundation/discussions)!
