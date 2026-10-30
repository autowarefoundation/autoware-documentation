# Autoware Core Debian package installation guide

## Prerequisites

| Item | Requirement                                                                                                                                            |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| OS   | [Ubuntu 22.04](https://releases.ubuntu.com/22.04/) or [Ubuntu 24.04](https://releases.ubuntu.com/24.04/)                                               |
| ROS  | ROS 2 Humble on Ubuntu 22.04, ROS 2 Jazzy on Ubuntu 24.04 (For ROS 2 system dependencies, refer to [REP-2000](https://www.ros.org/reps/rep-2000.html)) |

The ROS 2 apt repository must be set up on the computer. If ROS 2 is not installed yet, follow the ROS 2 installation guide for [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

## How to set up

1. Update package information.

   ```bash
   sudo apt update
   ```

2. Install Autoware Core.

   ```bash
   # Ubuntu 24.04 (ROS 2 Jazzy)
   sudo apt install ros-jazzy-autoware-core

   # Ubuntu 22.04 (ROS 2 Humble)
   sudo apt install ros-humble-autoware-core
   ```

3. Source the ROS 2 environment. On Ubuntu 22.04, replace `jazzy` with `humble`.

   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

!!! note

    The version of `autoware_core` on the ROS build farm can be older than the version that `repositories/autoware.repos` pins for an Autoware release. For example, Autoware 2.0.0 pins `autoware_core` 1.10.0. To see the version that apt installs, run `apt policy ros-jazzy-autoware-core`. If you need the exact version of an Autoware release, use the [source installation](core-source-installation.md) or the [Docker installation](core-docker-installation.md).
