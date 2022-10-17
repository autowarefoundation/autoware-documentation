# Installation

This document contains step-by-step instruction on how to build [AWF Autoware Core/Universe](https://github.com/autowarefoundation/autoware) with `driving_log_replayer`.

## Prerequisites

1. [Autoware has been built and installed](https://autowarefoundation.github.io/autoware-documentation/main/installation/)
2. [Install pipx](https://pypa.github.io/pipx/)
3. [Install zstd](https://github.com/facebook/zstd)

   ```bash
   sudo apt install zstd
   ```

## How to build

1. Navigate to the Autoware workspace:

   ```bash
   cd autoware
   ```

2. Add dependency packages

   ```bash
   nano simulator.repos
   # add repositories below.
   ```

   ```shell
    simulator/perception_eval:
      type: git
      url: https://github.com/tier4/autoware_perception_evaluation.git
      version: main
    simulator/driving_log_replayer:
      type: git
      url: https://github.com/tier4/driving_log_replayer.git
      version: main
    simulator/vendor/ros2_numpy:
      type: git
      url: https://github.com/Box-Robotics/ros2_numpy.git
      # version humble is available for both humble and galactic
      version: humble
    simulator/vendor/ros2bag_extensions:
      type: git
      url: https://github.com/tier4/ros2bag_extensions.git
      version: main
   ```

3. Import Simulator dependencies:

   ```bash
   vcs import src < simulator.repos
   ```

4. Update rosdep

   ```bash
   rosdep update
   ```

5. Install dependent ROS packages:

   ```bash
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

6. Build the workspace:

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

7. Install cli to run driving_log_replayer

   ```bash
   pipx install git+https://github.com/tier4/driving_log_replayer.git
   ```
