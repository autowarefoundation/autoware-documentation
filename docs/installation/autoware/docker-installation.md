# Docker installation

Autoware publishes prebuilt multi-arch (amd64, arm64) Docker images on GHCR. There are runtime images for trying things out, devel images for building locally, and CUDA variants for GPU workloads.

The full image catalog — twelve images organized as a build graph — is documented in the canonical Docker reference next to the Dockerfiles in the `autoware` repository. Bookmark these sections, since they track the implementation:

- [`docker/README.md` → Image Graph](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#image-graph) — visual diagram of how the images depend on each other.
- [`docker/README.md` → Images](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#images) — table describing each image and when to use it.
- [`docker/README.md` → Pull from GHCR](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#pull-from-ghcr) — tag pattern (`<stage>-<ros_distro>[-<date>|-<version>]`) and `docker pull` examples for every variant.

The two images most users will care about:

- `ghcr.io/autowarefoundation/autoware:universe-cuda-jazzy` — full Autoware runtime with NVIDIA CUDA, cuDNN, and TensorRT bundled.
- `ghcr.io/autowarefoundation/autoware:universe-jazzy` — full Autoware runtime, no GPU.

Replace `jazzy` with `humble` for ROS 2 Humble.

These tags track the latest build. For a stable release, append the version to the tag — for example, `ghcr.io/autowarefoundation/autoware:universe-jazzy-1.8.0` for the `1.8.0` release on ROS 2 Jazzy.

For the broader containerized-deployment story (deployment patterns, integrations, edge use cases), see Open AD Kit:

- <https://github.com/autowarefoundation/openadkit>
- <https://autowarefoundation.github.io/openadkit/>

!!! info

    Before proceeding, confirm and agree with the [NVIDIA Deep Learning Container license](https://developer.nvidia.com/ngc/nvidia-deep-learning-container-license). By pulling and using Autoware's CUDA images, you accept the terms and conditions of the license.

## Prerequisites

- Docker
- NVIDIA Container Toolkit (preferred)
- NVIDIA CUDA 12 compatible GPU Driver (preferred)

1. Clone `autowarefoundation/autoware` and move to the directory.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git
   cd autoware
   ```

   By default, this checks out the `main` branch, which contains the latest in-development changes. If you want to use a stable release, check out the corresponding release tag (compatible with both ROS 2 Humble and Jazzy):

   ```bash
   git checkout 1.8.0
   ```

   The list of available tags can be found on the [autoware releases page](https://github.com/autowarefoundation/autoware/releases).

2. Install Ansible and run the Docker setup playbook:

   ```bash
   bash ansible/scripts/install-ansible.sh
   ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml
   ansible-playbook autoware.dev_env.install_docker
   ```

   To install without **NVIDIA GPU** support:

   ```bash
   ansible-playbook autoware.dev_env.install_docker --skip-tags nvidia
   ```

   To download only the artifacts:

   ```bash
   ansible-playbook autoware.dev_env.install_dev_env --tags artifacts
   ```

!!! info

    GPU acceleration is required for some features such as object detection and traffic light detection/classification. For details of how to enable these features without a GPU, refer to the [Running Autoware without CUDA](../../tutorials/others/running-autoware-without-cuda.md).

## Quick Start

### Launching the runtime container

The runtime image runs `ros2 launch autoware_launch autoware.launch.xml` on container start. The canonical `docker run` invocation — with map and data volumes, X11 forwarding, CUDA passthrough, and a flag-by-flag rationale table — is in [`docker/README.md` → Usage](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#usage). The same section covers the no-GPU variant (drop the NVIDIA-related flags) and how to override the default CycloneDDS config when you need ROS 2 nodes to reach across hosts.

### Pre-configured demo scenarios

Rather than crafting your own `docker run` command, the [`docker/examples/demos/`](https://github.com/autowarefoundation/autoware/tree/main/docker/examples/demos) folder ships ready-to-run Compose stacks. Each demo has its own README with prerequisites and run commands:

- [**planning-simulator**](https://github.com/autowarefoundation/autoware/tree/main/docker/examples/demos/planning-simulator) — Planning simulator with the sample map, vehicle, and sensor kit. Three rendering paths via Compose overlays: software rendering by default, `docker-compose.dri.yaml` for Intel/AMD/Nouveau hosts, `docker-compose.nvidia.yaml` for NVIDIA proprietary.
- [**awsim**](https://github.com/autowarefoundation/autoware/tree/main/docker/examples/demos/awsim) — Bridges Autoware to the [AWSIM](https://tier4.github.io/AWSIM/) Unity simulator over `network_mode: host` and launches `e2e_simulator.launch.xml`. Requires NVIDIA + the Container Toolkit.
- [**scenario-simulator**](https://github.com/autowarefoundation/autoware/tree/main/docker/examples/demos/scenario-simulator) — Runs a `scenario_simulator_v2` scenario against a live Autoware planning stack as two services that share a generated CycloneDDS config.

### Running Autoware tutorials

Inside the container, run the Autoware tutorials by following these links:

[Planning Simulation](../../demos/planning-sim/index.md)

[Rosbag Replay Simulation](../../demos/rosbag-replay-simulation.md).

## Deployment

Open AD Kit provides different deployment options for Autoware, so that you can deploy Autoware on different platforms and scenarios easily. Refer to the [Open AD Kit Documentation](https://autowarefoundation.github.io/openadkit/) for more details.

## Development

For developing against Autoware (rather than just running it), the [`docker/examples/basic/`](https://github.com/autowarefoundation/autoware/tree/main/docker/examples/basic) folder ships three Compose files — each a "drop me into a shell" container built on the `universe-devel-*` images, with `~/autoware_data` (containing `maps/` and `ml_models/`) and the autoware source tree mounted. Pick the flavor that matches your host:

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

2. Update dependent ROS packages.

   The dependencies of Autoware may have changed after the Docker image was created.
   In that case, you need to run the following commands to update the dependencies.

   ```bash
   # Make sure all ros-$ROS_DISTRO-* packages are upgraded to their latest version
   sudo apt update && sudo apt upgrade
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

3. Build the workspace.

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   If there is any build issue, refer to [Troubleshooting](../../community/support/troubleshooting/index.md#build-issues).

### Update the Workspace

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

### Using VS Code remote containers for development

Using the [Visual Studio Code](https://code.visualstudio.com/) with the [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension, you can develop Autoware in the containerized environment with ease.

Get the Visual Studio Code's [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
And reopen the workspace in the container by selecting `Remote-Containers: Reopen in Container` from the Command Palette (`F1`).

You can choose the `universe-devel-jazzy` or `universe-devel-cuda-jazzy` image to develop without or with CUDA support.

### Building Docker images from scratch

The build pipeline uses [`docker buildx bake`](https://docs.docker.com/build/bake/) driven by [`docker/docker-bake.hcl`](https://github.com/autowarefoundation/autoware/blob/main/docker/docker-bake.hcl). Building any target beyond `base` requires the autoware source repositories checked out under `src/`:

```bash
cd autoware/
vcs import src < repositories/autoware.repos
docker buildx bake -f docker/docker-bake.hcl
```

That builds the default targets (`universe` and `universe-cuda`); dependencies in the [image graph](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#image-graph) are resolved automatically. To target a specific stage (e.g. `core-devel`, `base-cuda-runtime`), pass it as an argument; to build for ROS 2 Humble, prefix with `ROS_DISTRO=humble`. See [`docker/README.md` → Build locally](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#build-locally) for the full target list and multi-arch flow.

Pinned date- and release-tagged versions of every image are published on [GHCR](https://github.com/autowarefoundation/autoware/pkgs/container/autoware) — use them when you need a fixed version of the image.
