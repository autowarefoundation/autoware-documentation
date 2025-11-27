# Open AD Kit: Containerized workloads for Autoware

Open AD Kit offers two types of Docker image to let you get started with Autoware quickly: `devel` and `runtime`.

1. The `devel` image enables you to develop Autoware without setting up the local development environment.
2. The `runtime` image contains only runtime executables and enables you to try out Autoware quickly.

!!! info

    Before proceeding, confirm and agree with the [NVIDIA Deep Learning Container license](https://developer.nvidia.com/ngc/nvidia-deep-learning-container-license). By pulling and using the Autoware Open AD Kit images, you accept the terms and conditions of the license.

## Prerequisites

- Docker
- NVIDIA Container Toolkit (preferred)
- NVIDIA CUDA 12 compatible GPU Driver (preferred)

1. Clone `autowarefoundation/autoware` and move to the directory.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git
   cd autoware
   ```

2. The [setup script](https://github.com/autowarefoundation/autoware/blob/main/setup-dev-env.sh) will install all required dependencies with:

   ```bash
   ./setup-dev-env.sh -y docker
   ```

   To install without **NVIDIA GPU** support:

   ```bash
   ./setup-dev-env.sh -y --no-nvidia docker
   ```

   To download only the artifacts:

   ```bash
   ./setup-dev-env.sh -y download_artifacts
   ```

!!! info

    GPU acceleration is required for some features such as object detection and traffic light detection/classification. For details of how to enable these features without a GPU, refer to the [Running Autoware without CUDA](../../how-to-guides/others/running-autoware-without-cuda.md).

## Quick Start

### Launching the runtime container

You can use `run.sh` to run the Autoware runtime container with the map and data (artifacts) paths:

```bash
./docker/run.sh --map-path path_to_map --data-path path_to_data
```

For more launch options, you can append a custom launch command instead of using the default launch command which is `ros2 launch autoware_launch autoware.launch.xml`.

Here is an example of running the runtime container with a custom launch command:

```bash
./docker/run.sh --map-path ~/autoware_map/sample-map-rosbag --data-path ~/autoware_data ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

!!! info

    Use `--no-nvidia` to run without NVIDIA GPU support, and `--headless` to run without display (no RViz visualization).

### Running Autoware tutorials

Inside the container, run the Autoware tutorials by following these links:

[Planning Simulation](../../tutorials/ad-hoc-simulation/planning-simulation.md)

[Rosbag Replay Simulation](../../tutorials/ad-hoc-simulation/rosbag-replay-simulation.md).

## Deployment

Open AD Kit provides different deployment options for Autoware, so that you can deploy Autoware on different platforms and scenarios easily. Refer to the [Open AD Kit Documentation](https://autowarefoundation.github.io/openadkit/) for more details.

## Development

```bash
./docker/run.sh --devel
```

!!! info

    By default workspace mounted on the container will be current directory(pwd), you can change the workspace path by `--workspace path_to_workspace`. For development environments without NVIDIA GPU support use `--no-nvidia`.

### How to set up a workspace

1. Create the `src` directory and clone repositories into it.

   ```bash
   mkdir src
   vcs import src < autoware.repos
   ```

   If you are an active developer, you may also want to pull the nightly repositories, which contain the latest updates:

   ```bash
   vcs import src < autoware-nightly.repos
   ```

   > ⚠️ Note: The nightly repositories are unstable and may contain bugs. Use them with caution.

   Optionally, you may also download the extra repositories that contain drivers for specific hardware, but they are not necessary for building and running Autoware:

   ```bash
   vcs import src < extra-packages.repos
   ```

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

   If there is any build issue, refer to [Troubleshooting](../../support/troubleshooting/index.md#build-issues).

### Update the Workspace

```bash
cd autoware
git pull
vcs import src < autoware.repos

# If you are using nightly repositories, also run the following command:
vcs import src < autoware-nightly.repos

vcs pull src
# Make sure all ros-$ROS_DISTRO-* packages are upgraded to their latest version
sudo apt update && sudo apt upgrade
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

It might be the case that dependencies imported via `vcs import` have been moved/removed.
VCStool does not currently handle those cases, so if builds fail after `vcs import`, cleaning
and re-importing all dependencies may be necessary:

```bash
rm -rf src/*
vcs import src < autoware.repos
# If you are using nightly repositories, import them as well.
vcs import src < autoware-nightly.repos
```

### Using VS Code remote containers for development

Using the [Visual Studio Code](https://code.visualstudio.com/) with the [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension, you can develop Autoware in the containerized environment with ease.

Get the Visual Studio Code's [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
And reopen the workspace in the container by selecting `Remote-Containers: Reopen in Container` from the Command Palette (`F1`).

You can choose Autoware or Autoware-cuda image to develop with or without CUDA support.

### Building Docker images from scratch

If you want to build these images locally for development purposes, run the following command:

```bash
cd autoware/
./docker/build.sh
```

To build without CUDA, use the `--no-cuda` option:

```bash
./docker/build.sh --no-cuda
```

To build only development image, use the `--devel-only` option:

```bash
./docker/build.sh --devel-only
```

To specify the platform, use the `--platform` option:

```bash
./docker/build.sh --platform linux/amd64
./docker/build.sh --platform linux/arm64
```

#### Using Docker images other than `latest`

There are also images versioned based on the `date` or `release tag`.  
Use them when you need a fixed version of the image.

The list of versions can be found [here](https://github.com/autowarefoundation/autoware/packages).
