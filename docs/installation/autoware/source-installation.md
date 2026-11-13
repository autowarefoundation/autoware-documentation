# Source installation

## Prerequisites

- OS
  - [Ubuntu 22.04](https://releases.ubuntu.com/22.04/)
  - [Ubuntu 24.04](https://releases.ubuntu.com/24.04/)

- ROS
  - ROS 2 Humble (on Ubuntu 22.04)
  - ROS 2 Jazzy (on Ubuntu 24.04)

  For ROS 2 system dependencies, refer to [REP-2000](https://www.ros.org/reps/rep-2000.html).

- [Git](https://git-scm.com/)
  - [Registering SSH keys to GitHub](https://github.com/settings/keys) is preferable.

```bash
sudo apt-get -y update
sudo apt-get -y install git
```

## How to set up a development environment

1. Clone `autowarefoundation/autoware` and move to the directory.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git
   cd autoware
   ```

   By default, this checks out the `main` branch, which contains the latest in-development changes. If you want to use a stable release, check out the corresponding release tag. For example, to use the `2.0.0` release (compatible with both ROS 2 Humble and Jazzy):

   ```bash
   git checkout 2.0.0
   ```

   The list of available tags can be found on the [autoware releases page](https://github.com/autowarefoundation/autoware/releases).

2. If you are installing Autoware for the first time, you can automatically install the dependencies by using the provided Ansible playbook.

   ```bash
   bash ansible/scripts/install-ansible.sh
   source ~/.bashrc
   ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml
   ansible-playbook autoware.dev_env.install_dev_env --ask-become-pass
   ```

   To install without **NVIDIA GPU** support:

   ```bash
   ansible-playbook autoware.dev_env.install_dev_env --skip-tags nvidia --ask-become-pass
   ```

   The playbook reads your Ubuntu version and selects the versions of ROS 2 and the NVIDIA libraries:

   | Ubuntu | ROS 2  | CUDA | TensorRT                                   |
   | ------ | ------ | ---- | ------------------------------------------ |
   | 22.04  | Humble | 12.8 | 10.8 on amd64, 10.3 on arm64 (Jetson Orin) |
   | 24.04  | Jazzy  | 13.0 | 10.13                                      |

   The playbook stops on other Ubuntu versions.

   If you encounter any build issues, please consult the [Troubleshooting](../../community/support/troubleshooting/index.md#build-issues) section for assistance.

!!! info

    Before installing NVIDIA libraries, please ensure that you have reviewed and agreed to the licenses.

    - [CUDA](https://docs.nvidia.com/cuda/eula/index.html)
    - [cuDNN](https://docs.nvidia.com/deeplearning/cudnn/sla/index.html)
    - [TensorRT](https://docs.nvidia.com/deeplearning/tensorrt/sla/index.html)

!!! note

    The following items will be automatically installed. If the ansible script doesn't work or if you already have different versions of dependent libraries installed, please install the following items manually.

    - [Install Ansible](https://github.com/autowarefoundation/autoware/tree/main/ansible#ansible-installation)
    - [Install Build Tools](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/build_tools#manual-installation)
    - [Install Dev Tools](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/dev_tools#manual-installation)
    - [Install geographiclib](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/geographiclib#manual-installation)
    - [Install the RMW Implementation](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/rmw_implementation#manual-installation)
    - [Install ROS 2](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/ros2#manual-installation)
    - [Install ROS 2 Dev Tools](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/ros2_dev_tools#manual-installation)
    - [Install Nvidia CUDA](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/cuda#manual-installation)
    - [Install Nvidia cuDNN and TensorRT](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/tensorrt#manual-installation)
    - [Install spconv](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/spconv#manual-installation) (for sparse convolutions)
    - [Install acados](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/acados#readme)
    - [Install Agnocast](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/agnocast#installation)
    - [Install the Autoware RViz Theme](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/qt5ct_setup#readme) (only affects Autoware RViz)
    - [Download the Artifacts](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts#readme) (for perception inference)

## How to set up a workspace

1. Create the `src` directory and clone repositories into it.

   Autoware uses [vcs2l](https://github.com/ros-infrastructure/vcs2l) to construct workspaces.

   ```bash
   cd autoware
   mkdir -p src
   vcs import src < repositories/autoware.repos
   ```

   If you are an active developer, you may also want to pull the nightly repositories, which contain the latest updates:

   ```bash
   vcs import src < repositories/autoware-nightly.repos
   ```

   > ⚠️ Note: The nightly repositories are unstable and may contain bugs. Use them with caution.

   Optionally, you may also download the extra repositories that contain drivers for specific hardware, but they are not necessary for building and running Autoware:

   ```bash
   vcs import src < repositories/extra-packages.repos
   ```

   > ⚠️ You might need to install the dependencies of the extra packages manually.
   >
   > ➡️ Check the readme of the extra packages for more information.

   The `tools.repos` file adds [autoware_tools](https://github.com/autowarefoundation/autoware_tools), and the `simulator.repos` file adds [scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2). Import them only if you need them. For a description of each file, see [`repositories/*.repos` files](../../design/repos-files.md).

2. Add community packages to your workspace. _(optional)_ <span class="aw-badge-new">NEW</span>

   The [Autoware Index](autoware-index.md) is a registry of community packages that extend Autoware, each built and tested against the latest Autoware release.
   Pick packages on its [browse site](https://index.autoware.org/) or with `aw-index-cli` to generate `repositories/autoware-index.repos`, then import it the same way:

   ```bash
   vcs import src < repositories/autoware-index.repos
   ```

   > ➡️ See the [Autoware Index](autoware-index.md) page for the full guide.

3. Install dependent ROS packages.

   Autoware requires some ROS 2 packages in addition to the core components.
   The tool `rosdep` allows an automatic search and installation of such dependencies.
   You might need to run `rosdep update` before `rosdep install`.

   On Ubuntu 22.04, replace `jazzy` with `humble`.

   ```bash
   source /opt/ros/jazzy/setup.bash
   # Make sure all previously installed ros-$ROS_DISTRO-* packages are upgraded to their latest version
   sudo apt update && sudo apt upgrade
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

4. [Install and set up ccache to speed up consecutive builds](../../tutorials/others/advanced-usage-of-colcon.md#using-ccache-to-speed-up-recompilation). _(optional but highly recommended)_

5. Build the workspace.

   Autoware uses [colcon](https://github.com/colcon) to build workspaces.
   For more advanced options, refer to the [documentation](https://colcon.readthedocs.io/).

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   If there is any build issue, refer to [Troubleshooting](../../community/support/troubleshooting/index.md#build-issues).

6. Follow the steps in [Network Configuration](../additional-settings-for-developers/network-configuration/index.md) before running Autoware.

7. Apply the settings recommended in [Console settings for ROS 2](../additional-settings-for-developers/console-settings.md) for a better development experience. _(optional)_

## How to update a workspace

!!! warning "Upgrade from 1.9.0 or older to 2.0.0"

    Autoware 2.0.0 downloads the AI models from [Hugging Face](https://huggingface.co/AutowareFoundation). The models use one folder per variant, for example `lidar_centerpoint/tiny/`. The 2.0.0 launch files read only this layout.

    After you update the workspace, download the models again:

    ```bash
    ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml
    ansible-playbook autoware.dev_env.install_dev_env --tags artifacts --ask-become-pass
    ```

    `autoware_livox_tag_filter` moved from `autoware_universe` to the [Autoware Index](autoware-index.md). If you use it, add it to your workspace with `aw-index-cli`.

1. Update the `.repos` file.

   ```bash
   cd autoware
   git pull <remote> <your branch>
   ```

   `<remote>` is usually `git@github.com:autowarefoundation/autoware.git`

2. Update the repositories.

   ```bash
   vcs import src < repositories/autoware.repos
   ```

   > ⚠️ If you are using nightly repositories, you can also update them.
   >
   > ```bash
   > vcs import src < repositories/autoware-nightly.repos
   > ```

   ```bash
   vcs pull src
   ```

   For Git users:
   - `vcs import` is similar to `git checkout`.
     - Note that it doesn't pull from the remote.
   - `vcs pull` is similar to `git pull`.
     - Note that it doesn't switch branches.

   For more information, refer to the [official documentation](https://github.com/ros-infrastructure/vcs2l).

   It might be the case that dependencies imported via `vcs import` have been moved/removed.
   Vcs2l does not currently handle those cases, so if builds fail after `vcs import`, cleaning
   and re-importing all dependencies may be necessary:

   ```bash
   rm -rf src/*
   vcs import src < repositories/autoware.repos
   ```

   > ⚠️ If you are using nightly repositories, import them as well.
   >
   > ```bash
   > vcs import src < repositories/autoware-nightly.repos
   > ```

3. Install dependent ROS packages.

   On Ubuntu 22.04, replace `jazzy` with `humble`.

   ```bash
   source /opt/ros/jazzy/setup.bash
   # Make sure all previously installed ros-$ROS_DISTRO-* packages are upgraded to their latest version
   sudo apt update && sudo apt upgrade
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

4. Build the workspace.

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
