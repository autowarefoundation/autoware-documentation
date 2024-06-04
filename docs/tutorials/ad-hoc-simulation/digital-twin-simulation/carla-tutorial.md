# CARLA simulator

[CARLA](https://carla.org) is a famous open-source simulator for the autonomous driving research.
Now there is no official support to Autoware.universe, but some projects from communities support it.
The document is to list these projects for anyone who wants to run Autoware with Carla.
You can report issues to each project if there is any problem.

## Project lists (In alphabetical order)

### carla_autoware_bridge

An addition package to `carla_ros_bridge` to connect CARLA simulator to Autoware Universe software.

- Project Link: [carla_autoware_bridge](https://github.com/Robotics010/carla_autoware_bridge)
- Tutorial: [https://github.com/Robotics010/carla_autoware_bridge/blob/master/getting-started.md](https://github.com/Robotics010/carla_autoware_bridge/blob/master/getting-started.md)

### open_planner

Integrated open source planner and related tools for autonomous navigation of autonomous vehicle and mobile robots

- Project Link: [open_planner](https://github.com/ZATiTech/open_planner/tree/humble)
- Tutorial: [https://github.com/ZATiTech/open_planner/blob/humble/op_carla_bridge/README.md](https://github.com/ZATiTech/open_planner/blob/humble/op_carla_bridge/README.md)

### zenoh_carla_bridge

The project is mainly for controlling multiple vehicles in Carla.
It uses [Zenoh](https://zenoh.io/) to bridge the Autoware and Carla and is able to distinguish different messages for different vehicles.
Feel free to ask questions and report issues to [autoware_carla_launch](https://github.com/evshary/autoware_carla_launch).

- Project Link:
  - [autoware_carla_launch](https://github.com/evshary/autoware_carla_launch): Integrated environment to run the bridge and Autoware easily.
  - [zenoh_carla_bridge](https://github.com/evshary/zenoh_carla_bridge): The bridge implementation.
- Tutorial: [Running Multiple Autoware-Powered Vehicles in Carla using Zenoh](https://autoware.org/running-multiple-autoware-powered-vehicles-in-carla-using-zenoh)
