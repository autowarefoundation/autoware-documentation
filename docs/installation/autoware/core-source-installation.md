# Autoware Core source installation guide

## Prerequisites

| Item                        | Requirement                                                                                                                                            |
| --------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| OS                          | [Ubuntu 22.04](https://releases.ubuntu.com/22.04/) or [Ubuntu 24.04](https://releases.ubuntu.com/24.04/)                                               |
| ROS                         | ROS 2 Humble on Ubuntu 22.04, ROS 2 Jazzy on Ubuntu 24.04 (For ROS 2 system dependencies, refer to [REP-2000](https://www.ros.org/reps/rep-2000.html)) |
| [Git](https://git-scm.com/) | [Registering SSH keys to GitHub](https://github.com/settings/keys) is preferable.                                                                      |

!!! tip

    If ROS 2 is not installed yet, the Ansible playbook of the [source installation](source-installation.md#how-to-set-up-a-development-environment) can install it together with the other Autoware Core dependencies. The `core` tag selects only the parts for Autoware Core (ROS 2, the ROS 2 development tools, the build tools, the RMW implementation, Agnocast, the recommended development tools, and the RViz theme). Clone the `autoware` repository first (step 2 below), then run these commands from its root folder:

    ```bash
    bash ansible/scripts/install-ansible.sh
    source ~/.bashrc
    ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml
    ansible-playbook autoware.dev_env.install_dev_env --tags core --ask-become-pass
    ```

## How to set up

In the commands below, replace `jazzy` with `humble` on Ubuntu 22.04.

1. Install dependent tools.

   ```bash
   sudo apt -y update
   sudo apt -y install git python3-colcon-common-extensions python3-rosdep python3-vcs2l
   sudo rosdep init
   ```

   If `rosdep` is already initialized, `sudo rosdep init` prints an error. You can ignore it.

2. Create workspace and clone repository into it.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git $HOME/autoware_core_workspace
   cd $HOME/autoware_core_workspace
   git checkout 2.0.0  # optional: use a release tag instead of the main branch
   mkdir -p src
   vcs import src < repositories/autoware.repos
   ```

3. This step is optional. If you want the latest branch, switch to the nightly.

   ```bash
   cd $HOME/autoware_core_workspace
   vcs import src < repositories/autoware-nightly.repos
   ```

4. Install dependent ROS packages.

   ```bash
   cd $HOME/autoware_core_workspace
   source /opt/ros/jazzy/setup.bash
   sudo apt update && sudo apt -y upgrade
   rosdep update
   rosdep install -y --from-paths src/core --ignore-src --rosdistro $ROS_DISTRO
   ```

5. Build the workspace.

   ```bash
   cd $HOME/autoware_core_workspace
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --base-paths src/core --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

The `src/core` folder holds the repositories under `core/` in `autoware.repos`: `autoware_core`, the message and utility repositories, `autoware_lanelet2_extension`, `autoware_rviz_plugins`, `autoware_simple_planning_simulator`, `autoware_system_designer`, and `agnocast`. The `--base-paths src/core` option builds only these repositories.
