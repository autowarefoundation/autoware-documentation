# Compatibility test

Compatibility testing tool is to check whether the digital twin simulator is compatible to Autoware or not.

## Preliminaries

The following compatibility testing tool is based on ROS2 humple.

As Jan 26th 2023, it is only tested with [MORAI Sim: Drive](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/digital-twin-simulation/MORAI_Sim-tutorial/)

cd ~/${autoware installed directory}
source /opt/ros/humble/setup.bash
source install/setup.bash

cd ~/${autoware installed directory}/src/universe/autoware.universe/tools/simulator_test/simulator_compatibility_test/test_sim_common_manual_testing/
python3 -m pytest .
