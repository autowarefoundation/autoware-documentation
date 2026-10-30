# CARLA simulator

[CARLA](https://carla.org) is a famous open-source simulator for the autonomous driving research.
Autoware Universe includes a bridge package for CARLA, `autoware_carla_interface`. Projects from the community also connect Autoware to CARLA.
The document is to list these projects for anyone who wants to run Autoware with Carla.
You can report issues to each project if there is any problem.

## Project lists (In alphabetical order)

### autoware_carla_interface

Autoware ROS package that enables communication between Autoware and CARLA simulator for autonomous driving simulation.
It is integrated in autoware_universe and actively maintained to stay compatible with the latest Autoware updates.

- Package Link and Tutorial: [autoware_carla_interface](https://github.com/autowarefoundation/autoware_universe/tree/main/simulator/autoware_carla_interface).
- Docker Compose demo: [`docker/examples/demos/carla`](https://github.com/autowarefoundation/autoware/tree/main/docker/examples/demos/carla) in the Autoware repository. It runs CARLA 0.9.16, `autoware_carla_interface`, and Autoware (`e2e_simulator.launch.xml` with `simulator_type:=carla`) in separate containers.

### autoware_tensorrt_vad (end-to-end planning)

TensorRT-optimized Vectorized Autonomous Driving ([VAD](https://github.com/hustvl/VAD)) node that replaces the traditional perception/localization/planning stack with a single end-to-end model trained on CARLA ([Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive)). Ships with a CARLA-focused launch file and integrates with `autoware_launch` through `e2e_simulator.launch.xml`.

- Project Link: [autoware_tensorrt_vad](https://github.com/autowarefoundation/autoware_universe/tree/main/e2e/autoware_tensorrt_vad) (README includes parameters, topics, and model details)
- Usage (CARLA E2E mode):
  1. Build the package and deps: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to autoware_tensorrt_vad`.
  2. Prepare CARLA following `autoware_carla_interface` (use `carla_sensor_kit` so camera topics match the VAD training order: FRONT, BACK, FRONT_LEFT, BACK_LEFT, FRONT_RIGHT, BACK_RIGHT).
  3. Make sure that the VAD models are downloaded. The [`artifacts`](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts) ansible role downloads them to `~/autoware_data/ml_models/vad/v0.1`.
  4. Start CARLA server (example headless/GPU-friendly): `./CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen`.
  5. Launch Autoware in E2E mode:

     ```bash
     ros2 launch autoware_launch e2e_simulator.launch.xml \
       map_path:=$HOME/autoware_data/maps/Town01 \
       vehicle_model:=sample_vehicle \
       sensor_model:=carla_sensor_kit \
       simulator_type:=carla \
       use_e2e_planning:=true
     ```

     The `use_e2e_planning:=true` argument makes `autoware_carla_interface` publish the ground-truth localization and start a minimal control stack (a trajectory follower and a vehicle command gate).

  6. Start the VAD node in another terminal. The package provides the `vad_carla_tiny.launch.xml` launch file for it. In Autoware 2.0.0, `e2e_simulator.launch.xml` does not include this file.

     ```bash
     ros2 launch autoware_tensorrt_vad vad_carla_tiny.launch.xml
     ```

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
  - [autoware_carla_launch](https://github.com/evshary/autoware_carla_launch): The integrated environment to run the bridge and Autoware easily.
  - [zenoh_carla_bridge](https://github.com/evshary/zenoh_carla_bridge): The bridge implementation.
- Tutorial:
  - [The documentation of autoware_carla_launch](https://autoware-carla-launch.readthedocs.io/en/latest/): The official documentation, including installation and several usage scenarios.
  - [Running Multiple Autoware-Powered Vehicles in Carla using Zenoh](https://autoware.org/running-multiple-autoware-powered-vehicles-in-carla-using-zenoh): The introduction on Autoware Tech Blog.
