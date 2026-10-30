# PanoSim simulator demo

## Overview

PanoSim is an integrated testing simulation platform for autonomous driving, and its technologies and product development.

[PanoSim and Autoware co-simulation tutorial](https://gitee.com/wobuzhuchele/panosim-autoware/blob/master/README.en.md).
Based on PanoSim V32 and Autoware Universe, it includes bridge, map ,scenarios, installation tutorial.

## Instructions

1. [PanoSim-Autoware Framework](https://gitee.com/wobuzhuchele/panosim-autoware/blob/master/Document/PanoSim-Autoware.en.md)
2. [Co-simulation PanoSim Configuration Tutorial](https://gitee.com/wobuzhuchele/panosim-autoware/blob/master/Document/PanoSim-Autoware%20PanoSim.en.md), [ROS 2 Configuration Tutorial](https://gitee.com/wobuzhuchele/panosim-autoware/blob/master/Document/ROS2%20Bridge.en.md)
3. [Co-simulation AutoWare Configuration Tutorial](https://gitee.com/wobuzhuchele/panosim-autoware/blob/master/Document/PanoSim-Autoware%20Autoware.en.md)

## Development

The tutorial repository ships its own copy of the Autoware repository. Its `autoware.repos` file pins an old Autoware version: `v1.0`, with old repository names such as `autoware.universe`. The tutorial repository was last updated in January 2025. It does not target Autoware 2.0.0, so expect to adapt the bridge and the launch steps for a current Autoware version.

!!! note

    If you encounter any issues or have suggestions, please submit them under [PanoSim Autoware Issues](https://gitee.com/wobuzhuchele/panosim-autoware/issues).
