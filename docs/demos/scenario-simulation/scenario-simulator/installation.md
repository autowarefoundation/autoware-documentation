# Installation

This document contains step-by-step instruction on how to build [AWF Autoware Core/Universe](https://github.com/autowarefoundation/autoware) with `scenario_simulator_v2`.

## Prerequisites

1. [Autoware has been built and installed](../../../installation/index.md)

## How to build

1. Navigate to the Autoware workspace:

   ```bash
   cd autoware
   ```

2. Import Simulator dependencies:

   ```bash
   vcs import src < repositories/simulator.repos
   ```

3. Install dependent ROS packages:

   ```bash
   source /opt/ros/humble/setup.bash
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

4. Build the workspace:

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
