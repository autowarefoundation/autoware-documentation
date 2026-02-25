# Autoware Core source installation guide

## Prerequisites

| Item                        | Requirement                                                                                               |
| --------------------------- | --------------------------------------------------------------------------------------------------------- |
| OS                          | [Ubuntu 22.04](https://releases.ubuntu.com/22.04/)                                                        |
| ROS                         | ROS 2 Humble (For ROS 2 system dependencies, refer to [REP-2000](https://www.ros.org/reps/rep-2000.html)) |
| [Git](https://git-scm.com/) | [Registering SSH keys to GitHub](https://github.com/settings/keys) is preferable.                         |

## How to set up

1. Install dependent tools.

   ```bash
   sudo apt -y update
   sudo apt -y install git python3-colcon-common-extensions python3-rosdep
   sudo rosdep init
   ```

2. Create workspace and clone repository into it.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git $HOME/autoware_core_workspace
   cd $HOME/autoware_core_workspace
   mkdir -p src
   vcs import src < repositories/core.repos
   ```

3. Install dependent ROS packages.

   ```bash
   cd $HOME/autoware_core_workspace
   sudo apt update && sudo apt -y upgrade
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro humble
   ```

4. Build the workspace.

   ```bash
   cd $HOME/autoware_core_workspace
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
