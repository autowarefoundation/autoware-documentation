# AWSIM simulator setup

## Preparation

- Install Autoware following the [official guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)
- Download and extract the [AWSIM binary](https://drive.google.com/file/d/1jgB_A5cSso9dzcNffQYthe07GG07QLQh/view?usp=sharing)
- Download and extract the [sample map folder](https://drive.google.com/file/d/1vGFI0o0zQ-gRZYqKrPbnrtCN3c3-92Fy/view?usp=sharing)

## Running steps

### AWSIM Simulator

To run the AWSIM simulator, open a terminal window, navigate to the simulation directory and run:

```bash
source /opt/ros/galactic/setup.bash
cd <PATH_TO_EXTRACTED_FOLDER_WITH_AWSIM>
./AWSIM.x86_64
```


If the simulator is working properly, Lidar rays should be visible and the [expected data topics](https://github.com/autowarefoundation/autoware-projects/wiki/Bus-ODD-Simulation-requirements#integration-interface-with-autoware-universe) should appear when `ros2 topic list` is run.

![correctly working simulator](./images/workingSim.png)

#### Simulator shortcuts

It is possible to adjust/control the simulation through the shortcuts listed below:

| Shortcut | Description |
|---|---|
| v | turn on/off lidar points visualization |
| m | enter/exit manual operation mode (Ego listens to keyboard) |
| | <b> In manual mode: </b> |
| d | drive |
| p | parking brake |
| r | reverse |
| arrows | acceleration and steering |
| 1 | left blinker |
| 2 | right blinker |
| 3 | emergency lights |
| 4 | turning the lights off |

### Running Autoware to control AWSIM

To run Autoware, open a terminal and run the following commands:

```bash
cd <YOUR_WORKSPACE>/autoware
source install/setup.bash
ros2 launch autoware_launch autoware.launch.xml map_path:=<PATH_TO_EXTRACTED_FOLDER_WITH_MAP> vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit use_sim_time:=true launch_sensing_driver:=false
```

!!! note

    Note that `map_path` must be a full path. Relative paths will not work.

- RViz should appear with the Lanelet2 map displayed. Although the ego vehicle can localize itself at the beginning, the localization can be inaccurate. It is recommended to explicitly set the vehicle's initial pose using the `2D Pose Estimate` button as shown in the image below.

![how to initialize autoware](./images/initializeAutoware.png)

- To make Autoware plan the route, use the `2D Goal Pose` button to set a goal pose somewhere within the Lanelet2 road lanes.

- To make Autoware execute the planned route, open a new terminal and send the engage message:

```bash
cd <YOUR_WORKSPACE>/autoware
source install/setup.bash
ros2 topic pub /autoware/engage autoware_auto_vehicle_msgs/msg/Engage "engage: true"
```

From now on, Autoware will execute the planned route and operate the AWSIM Simulator.

![AWSIM with autoware](./images/awf_awsim.png)

### NPC Control (optional)

Currently the only supported way to add NPCs to the simulation is using prerecorded rosbags.

#### Preparation

- Download and extract [npc_msgs](https://drive.google.com/file/d/1fMvjOnz7Z0cGXotwdOAhBfLPYO52ssp0/view?usp=sharing)
- Download and extract [sample rosbags](https://drive.google.com/file/d/1BSoIq82DW_RIyeSR6OinhKmA6lD6jmWe/view?usp=sharing)

#### Run NPCs from rosbag

To run NPCs from rosbag, follow the steps:

- navigate to the directory with extracted npc_msgs
- build and source npc_msgs:

```bash
colcon build
source install/setup.bash
```

- play sample rosbag while AWSIM is running:

```bash
ros2 bag play <ROSBAG_NAME>
```

![AWSIM with autoware and NPCs](./images/awsim_npc.png)

### Troubleshooting

If the data on topics is not produced or the poincloud is not visible on the simulation screen please report a problem in autoware [Q&A discussion](https://github.com/autowarefoundation/autoware/discussions/). To make analysis of the problem easier, please share a `Player.log` file (the file can be found under `~/.config/unity3d/Tier\ IV/E2ESimulator/Player.log` directory).
