# Docker installation

Autoware publishes prebuilt multi-arch (amd64, arm64) Docker images on GHCR. There are runtime images for trying things out, devel images for building locally, and CUDA variants for GPU workloads.

The full image catalog has twelve images that form a build graph. The canonical Docker reference next to the Dockerfiles in the `autoware` repository documents it. Bookmark these sections, since they track the implementation:

- [`docker/README.md` → Image Graph](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#image-graph): a diagram of how the images depend on each other.
- [`docker/README.md` → Images](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#images): a table that describes each image and when to use it.
- [`docker/README.md` → Pull from GHCR](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#pull-from-ghcr): the tag pattern (`<stage>-<ros_distro>[-<date>|-<version>]`) and `docker pull` examples for every variant.

The two images most users will care about:

- `ghcr.io/autowarefoundation/autoware:universe-cuda-jazzy`: the full Autoware runtime with NVIDIA CUDA, cuDNN, and TensorRT bundled.
- `ghcr.io/autowarefoundation/autoware:universe-jazzy`: the full Autoware runtime, without GPU support.

Replace `jazzy` with `humble` for ROS 2 Humble.

These tags track the latest build. For a stable release, append the version to the tag. For example, use `ghcr.io/autowarefoundation/autoware:universe-jazzy-2.0.0` for the `2.0.0` release on ROS 2 Jazzy.

For the broader containerized-deployment story (deployment patterns, integrations, edge use cases), see Open AD Kit:

- <https://github.com/autowarefoundation/openadkit>
- <https://autowarefoundation.github.io/openadkit/>

!!! info

    Before proceeding, confirm and agree with the [NVIDIA Deep Learning Container license](https://developer.nvidia.com/ngc/nvidia-deep-learning-container-license). By pulling and using Autoware's CUDA images, you accept the terms and conditions of the license.

## Prerequisites

- Docker
- NVIDIA Container Toolkit (preferred)
- NVIDIA GPU driver (preferred): version 580 or newer for the Jazzy CUDA images (CUDA 13.0), and version 570 or newer for the Humble CUDA images (CUDA 12.8). For details, see the [CUDA Toolkit release notes](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html).

1. Clone `autowarefoundation/autoware` and move to the directory.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git
   cd autoware
   ```

   By default, this checks out the `main` branch, which contains the latest in-development changes. If you want to use a stable release, check out the corresponding release tag (compatible with both ROS 2 Humble and Jazzy):

   ```bash
   git checkout 2.0.0
   ```

   The list of available tags can be found on the [autoware releases page](https://github.com/autowarefoundation/autoware/releases).

2. Install Ansible and run the Docker setup playbook:

   ```bash
   bash ansible/scripts/install-ansible.sh
   source ~/.bashrc
   ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml
   ansible-playbook autoware.dev_env.install_docker --ask-become-pass
   ```

   To install without **NVIDIA GPU** support:

   ```bash
   ansible-playbook autoware.dev_env.install_docker --skip-tags nvidia --ask-become-pass
   ```

   To download only the artifacts:

   ```bash
   ansible-playbook autoware.dev_env.install_dev_env --tags artifacts --ask-become-pass
   ```

!!! info

    GPU acceleration is required for some features such as object detection and traffic light detection/classification. For details of how to enable these features without a GPU, refer to the [Running Autoware without CUDA](../../tutorials/others/running-autoware-without-cuda.md).

## Quick start

### Launching the runtime container

The runtime images open a Bash shell on container start. The entrypoint sources `/opt/ros/<distro>/setup.bash` and `/opt/autoware/setup.bash`, so you can run `ros2 launch autoware_launch autoware.launch.xml` directly. The canonical `docker run` command is in [`docker/README.md` → Usage](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#usage). It mounts the map and model volumes, forwards X11, passes the GPU through, and explains each flag in a table. The same section covers the no-GPU variant (drop the NVIDIA-related flags) and how to override the default CycloneDDS configuration when ROS 2 nodes must reach other hosts. For the network side of a multi-host setup, see [Communicating across multiple computers with CycloneDDS](../additional-settings-for-developers/network-configuration/multiple-computers.md).

### Pre-configured demo scenarios

Rather than crafting your own `docker run` command, the [`docker/examples/demos/`](https://github.com/autowarefoundation/autoware/tree/main/docker/examples/demos) folder ships ready-to-run Compose stacks. Each demo has its own README with prerequisites and run commands:

- [**planning-simulator**](https://github.com/autowarefoundation/autoware/tree/main/docker/examples/demos/planning-simulator): Planning simulator with the sample map, vehicle, and sensor kit. Three rendering paths via Compose overlays: software rendering by default, `docker-compose.dri.yaml` for Intel/AMD/Nouveau hosts, `docker-compose.nvidia.yaml` for NVIDIA proprietary.
- [**awsim**](https://github.com/autowarefoundation/autoware/tree/main/docker/examples/demos/awsim): Bridges Autoware to the [AWSIM](https://autowarefoundation.github.io/AWSIM/) Unity simulator over `network_mode: host` and launches `e2e_simulator.launch.xml`. Requires NVIDIA + the Container Toolkit.
- [**scenario-simulator**](https://github.com/autowarefoundation/autoware/tree/main/docker/examples/demos/scenario-simulator): Runs a `scenario_simulator_v2` scenario against a live Autoware planning stack as two services that share a generated CycloneDDS configuration.
- [**carla**](https://github.com/autowarefoundation/autoware/tree/main/docker/examples/demos/carla): Runs the [CARLA](https://carla.org/) 0.9.16 simulator, the `autoware_carla_interface` bridge, and Autoware (`e2e_simulator.launch.xml` with `simulator_type:=carla`) as separate services over `network_mode: host`. Requires NVIDIA + the Container Toolkit.

### Running Autoware tutorials

Inside the container, run the Autoware tutorials by following these links:

[Planning Simulation](../../simulation/planning-simulation/index.md)

[Rosbag Replay Simulation](../../simulation/rosbag-replay-simulation.md).

## Deployment

Open AD Kit provides different deployment options for Autoware, so that you can deploy Autoware on different platforms and scenarios easily. Refer to the [Open AD Kit Documentation](https://autowarefoundation.github.io/openadkit/) for more details.

## Development

For developing against Autoware (rather than just running it), the [`docker/examples/basic/`](https://github.com/autowarefoundation/autoware/tree/main/docker/examples/basic) folder ships three Compose files. Each one is a "drop me into a shell" container built on the `universe-devel-*` images, with `~/autoware_data` (containing `maps/` and `ml_models/`) and the autoware source tree mounted. Pick the flavor that matches your host:

| Host GPU / driver            | Compose file                                |
| ---------------------------- | ------------------------------------------- |
| NVIDIA + proprietary driver  | `dev-nvidia.compose.yaml` (recommended)     |
| NVIDIA + Nouveau open driver | `dev-dri.compose.yaml`                      |
| Intel / AMD                  | `dev-dri.compose.yaml`                      |
| No GPU / headless            | `dev-cpu.compose.yaml` (software rendering) |

From the autoware repo root:

```bash
xhost +local:docker
HOST_UID=$(id -u) HOST_GID=$(id -g) \
  docker compose -f docker/examples/basic/dev-nvidia.compose.yaml run --rm autoware
```

[`docker/examples/basic/README.md`](https://github.com/autowarefoundation/autoware/blob/main/docker/examples/basic/README.md) goes deeper: how to verify you actually got hardware acceleration (`glxinfo -B`), why `dev-dri` silently falls back to software rendering on NVIDIA proprietary, and how to attach a second terminal to a running dev container.

### How to set up a workspace

1. Create the `src` directory and clone repositories into it.

   ```bash
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

3. Update dependent ROS packages.

   The dependencies of Autoware may have changed after the Docker image was created.
   In that case, you need to run the following commands to update the dependencies.

   ```bash
   # Make sure all ros-$ROS_DISTRO-* packages are upgraded to their latest version
   sudo apt update && sudo apt upgrade
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

4. Build the workspace.

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   If there is any build issue, refer to [Troubleshooting](../../community/support/troubleshooting/index.md#build-issues).

### Update the workspace

!!! warning "Upgrade from 1.9.0 or older to 2.0.0"

    Autoware 2.0.0 downloads the AI models from [Hugging Face](https://huggingface.co/AutowareFoundation). The models use one folder per variant, for example `lidar_centerpoint/tiny/`. The 2.0.0 launch files read only this layout.

    After you update the workspace, download the models again on the host:

    ```bash
    ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml
    ansible-playbook autoware.dev_env.install_dev_env --tags artifacts --ask-become-pass
    ```

    `autoware_livox_tag_filter` moved from `autoware_universe` to the [Autoware Index](autoware-index.md). If you use it, add it to your workspace with `aw-index-cli`.

```bash
cd autoware
git pull
vcs import src < repositories/autoware.repos

# If you are using nightly repositories, also run the following command:
vcs import src < repositories/autoware-nightly.repos

vcs pull src
# Make sure all ros-$ROS_DISTRO-* packages are upgraded to their latest version
sudo apt update && sudo apt upgrade
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

It might be the case that dependencies imported via `vcs import` have been moved/removed.
Vcs2l does not currently handle those cases, so if builds fail after `vcs import`, cleaning
and re-importing all dependencies may be necessary:

```bash
rm -rf src/*
vcs import src < repositories/autoware.repos
# If you are using nightly repositories, import them as well.
vcs import src < repositories/autoware-nightly.repos
```

### Using `devcontainer` for development

Using the [Visual Studio Code](https://code.visualstudio.com/) with the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension, you can develop Autoware in the containerized environment with ease.

Get the Visual Studio Code's [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
And reopen the workspace in the container by selecting `Dev Containers: Reopen in Container` from the Command Palette (`F1`).

The `.devcontainer/` folder of the `autoware` repository has three configurations. Pick one when Visual Studio Code asks:

| Configuration         | Image                       | Use                                    |
| --------------------- | --------------------------- | -------------------------------------- |
| `core-devel`          | `core-devel-jazzy`          | Develop Autoware Core packages         |
| `universe-devel`      | `universe-devel-jazzy`      | Develop Autoware Universe without CUDA |
| `universe-devel-cuda` | `universe-devel-cuda-jazzy` | Develop Autoware Universe with CUDA    |

Each configuration mounts the repository at `/home/aw/autoware` and `~/autoware_data` at `/home/aw/autoware_data`. Its `initializeCommand` creates `~/autoware_data` on the host if the folder does not exist.

`devcontainer` is also supported by [devcontainer cli](https://github.com/devcontainers/cli) and can be started from the CLI as well. At the repository root, select the configuration with the `--config` option:

```console
devcontainer up --workspace-folder . --config .devcontainer/universe-devel-cuda/devcontainer.json
devcontainer exec --workspace-folder . --config .devcontainer/universe-devel-cuda/devcontainer.json bash
```

To update the base image or reflect the changes in `devcontainer.json`, add `--remove-existing-container` option to `devcontainer up`.

### Building Docker images from scratch

The build pipeline uses [`docker buildx bake`](https://docs.docker.com/build/bake/) driven by [`docker/docker-bake.hcl`](https://github.com/autowarefoundation/autoware/blob/main/docker/docker-bake.hcl). Building any target beyond `base` requires the autoware source repositories checked out under `src/`:

```bash
cd autoware/
vcs import src < repositories/autoware.repos
docker buildx bake -f docker/docker-bake.hcl
```

That builds the `default` group of `docker-bake.hcl`, which contains all twelve images of the [image graph](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#image-graph). To build a specific stage (for example `core-devel` or `base-cuda-runtime`), pass it as an argument. Its dependencies are resolved automatically. To build for ROS 2 Humble, prefix the command with `ROS_DISTRO=humble`. To pin the base image and the package versions to the lockfiles, prefix it with `USE_LOCKFILE=true`. See [`docker/README.md` → Build locally](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#build-locally) for the full target list and multi-arch flow.

Pinned date- and release-tagged versions of every image are published on [GHCR](https://github.com/autowarefoundation/autoware/pkgs/container/autoware). Use them when you need a fixed version of the image.
