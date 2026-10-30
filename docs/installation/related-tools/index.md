# Installation of related tools

Autoware itself does not need the tools on this page. Install them when your evaluation or your vehicle integration needs them. Each section assumes that you have a source workspace from the [source installation](../autoware/source-installation.md) in the `autoware` directory.

## Autoware tools

The [autoware_tools](https://github.com/autowarefoundation/autoware_tools) repository holds packages that Autoware does not need at runtime: tools for benchmarking, debugging, tuning, calibration, and map work. The `repositories/tools.repos` file pins its version (0.8.0 for Autoware 2.0.0).

```bash
cd autoware
vcs import src < repositories/tools.repos
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Simulators

The [planning simulation](../../simulation/planning-simulation/index.md) needs no extra simulator. Its simple simulator (`autoware_simple_planning_simulator`) is part of the Autoware Core repositories in `autoware.repos`. The planning simulation page explains how to download its sample map. For other simulators, use the guides below:

- [scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2): runs scenarios against Autoware. The `repositories/simulator.repos` file pins its version (22.0.0 for Autoware 2.0.0). For the build steps, see [Scenario simulator installation](../../simulation/scenario-simulation/scenario-simulator/installation.md).
- [AWSIM](https://github.com/autowarefoundation/AWSIM): a Unity-based simulator for end-to-end simulation. See the [AWSIM tutorial](../../simulation/digital-twin-simulation/awsim-tutorial.md).
- [CARLA](https://carla.org/): an Unreal-based simulator that connects to Autoware through `autoware_carla_interface`. See the [CARLA tutorial](../../simulation/digital-twin-simulation/carla-tutorial.md).

The `autoware` repository also has Docker Compose stacks for the AWSIM, CARLA, and scenario simulator demos. See [Pre-configured demo scenarios](../autoware/docker-installation.md#pre-configured-demo-scenarios).

## Calibration tools

### Calibration tools provided by TIER IV

Once the Autoware is installed, you can install the calibration tools provided by TIER IV as follows:

```bash
cd autoware
wget https://raw.githubusercontent.com/tier4/CalibrationTools/refs/heads/tier4/universe/calibration_tools_autoware.repos
vcs import src < calibration_tools_autoware.repos
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Check the [README.md file](https://github.com/tier4/CalibrationTools/blob/tier4/universe/README.md) for a more detailed explanation.
