# Installation

This document contains step-by-step instruction on how to build [AWF Autoware Core/Universe](https://github.com/autowarefoundation/autoware) with `scenario_simulator_v2`.

## Prerequisites

1. [Autoware has been built and installed](https://autowarefoundation.github.io/autoware-documentation/main/installation/)

## How to build

1. Navigate to the Autoware workspace:

   ```bash
   cd autoware
   ```

2. Import Simulator dependencies:

   ```bash
   vcs import src < simulator.repos
   ```

5. Install dependent ROS packages.

   ```bash
   source /opt/ros/galactic/setup.bash
   rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

6. Build the workspace.

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
