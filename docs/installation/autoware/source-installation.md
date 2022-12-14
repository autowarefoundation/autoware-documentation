# Source installation

## Prerequisites

- OS

  - [Ubuntu 22.04](https://releases.ubuntu.com/22.04/)

- ROS

  - ROS 2 Humble

  For ROS 2 system dependencies, refer to [REP-2000](https://www.ros.org/reps/rep-2000.html).

- [Git](https://git-scm.com/)
  - [Registering SSH keys to GitHub](https://github.com/settings/keys) is preferable.

```bash
sudo apt-get -y update
sudo apt-get -y install git
```

> Note: If you wish to use ROS 2 Galactic on Ubuntu 20.04, refer to installation instruction from [galactic](https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/) branch, but be aware that Galactic version of Autoware might not have latest features.

## How to set up a development environment

1. Clone `autowarefoundation/autoware` and move to the directory.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git
   cd autoware
   ```

2. You can install the dependencies either manually or using the provided Ansible script.

> Note: Before installing NVIDIA libraries, confirm and agree with the licenses.

- [CUDA](https://docs.nvidia.com/cuda/eula/index.html)
- [cuDNN](https://docs.nvidia.com/deeplearning/cudnn/sla/index.html)
- [TensorRT](https://docs.nvidia.com/deeplearning/tensorrt/sla/index.html)

### Installing dependencies manually

- [Install ROS 2](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/ros2#manual-installation)
- [Install ROS 2 Dev Tools](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/ros2_dev_tools#manual-installation)
- [Install the RMW Implementation](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/rmw_implementation#manual-installation)
- [Install pacmod](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/pacmod#manual-installation)
- [Install Autoware Core dependencies](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/autoware_core#manual-installation)
- [Install Autoware Universe dependencies](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/autoware_universe#manual-installation)
- [Install pre-commit dependencies](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/pre_commit#manual-installation)
- [Install Nvidia CUDA](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/cuda#manual-installation)
- [Install Nvidia cuDNN and TensorRT](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/tensorrt#manual-installation)

### Installing dependencies using Ansible

Be very careful with this method. Make sure you read and confirmed all the steps in the Ansible configuration before using it.

If you've manually installed the dependencies, you can skip this section.

```bash
./setup-dev-env.sh
```

## How to set up a workspace

1. Create the `src` directory and clone repositories into it.

   Autoware uses [vcstool](https://github.com/dirk-thomas/vcstool) to construct workspaces.

   ```bash
   cd autoware
   mkdir src
   vcs import src < autoware.repos
   ```

2. Install dependent ROS packages.

   Autoware requires some ROS 2 packages in addition to the core components.
   The tool `rosdep` allows an automatic search and installation of such dependencies.
   You might need to run `rosdep update` before `rosdep install`.

   ```bash
   source /opt/ros/humble/setup.bash
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

3. Build the workspace.

   Autoware uses [colcon](https://github.com/colcon) to build workspaces.
   For more advanced options, refer to the [documentation](https://colcon.readthedocs.io/).

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

<<<<<<< HEAD
   If there is any build issue, refer to [Troubleshooting](https://autowarefoundation.github.io/autoware-documentation/main/support/troubleshooting/#build-issues).
=======
   If there is any build issue, refer to [Troubleshooting](../../support/troubleshooting.md#build-issues).
>>>>>>> main

## How to update a workspace

1. Update the `.repos` file.

   ```bash
   cd autoware
   git pull
   ```

2. Update the repositories.

   ```bash
   vcs import src < autoware.repos
   vcs pull src
   ```

   For Git users:

   - `vcs import` is similar to `git checkout`.
     - Note that it doesn't pull from the remote.
   - `vcs pull` is similar to `git pull`.
     - Note that it doesn't switch branches.

   For more information, refer to the [official documentation](https://github.com/dirk-thomas/vcstool).

3. Install dependent ROS packages.

   ```bash
   source /opt/ros/humble/setup.bash
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

4. Build the workspace.

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
