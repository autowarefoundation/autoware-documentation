# Docker installation for development

## Prerequisites

- [Git](https://git-scm.com/)

- For NVIDIA Jetson devices, install [JetPack](https://docs.nvidia.com/jetson/jetpack/install-jetpack/index.html#how-to-install-jetpack) >= 5.0

## How to set up a development environment

1. Clone `autowarefoundation/autoware` and move to the directory.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git
   cd autoware
   ```

2. You can install the dependencies either manually or using the provided Ansible script.

> Note: Before installing NVIDIA libraries, confirm and agree with the licenses.

- [CUDA](https://docs.nvidia.com/cuda/eula/index.html)

### Installing dependencies manually

- [Install Nvidia CUDA](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/cuda#manual-installation)
- [Install Docker Engine](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/docker_engine#manual-installation)
- [Install NVIDIA Container Toolkit](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/nvidia_docker#manual-installation)
- [Install rocker](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/rocker#manual-installation)

### Installing dependencies using Ansible

Be very careful with this method. Make sure you read and confirmed all the steps in the Ansible configuration before using it.

If you've manually installed the dependencies, you can skip this section.

```bash
./setup-dev-env.sh docker
```

You might need to log out and log back to make the current user able to use docker.

## How to set up a workspace

!!! warning

    Before proceeding, confirm and agree with the [NVIDIA Deep Learning Container license](https://developer.nvidia.com/ngc/nvidia-deep-learning-container-license).
    By pulling and using the Autoware Universe images, you accept the terms and conditions of the license.

1. Create the `autoware_map` directory for map data later.

   ```bash
   mkdir ~/autoware_map
   ```

2. Pull the Dokcer image
   ```bash
   docker pull ghcr.io/autowarefoundation/autoware-universe:latest-cuda
   ```

3. Launch a Docker container.

   - For amd64 architecture computers with NVIDIA GPU:

     ```bash
     rocker --nvidia --x11 --user --volume $HOME/autoware --volume $HOME/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
     ```

   - If you want to run container without using NVIDIA GPU, or for arm64 architecture computers:

     ```bash
     rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/autoware --volume $HOME/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
     ```

     For detailed reason could be found [here](./docker-installation.md#docker-with-nvidia-gpu-fails-to-start-autoware-on-arm64-devices)

   For more advanced usage, see [here](https://github.com/autowarefoundation/autoware/tree/main/docker/README.md).

   After that, move to the workspace in the container:

   ```bash
   cd autoware
   ```

4. Create the `src` directory and clone repositories into it.

   ```bash
   mkdir src
   vcs import src < autoware.repos
   ```

5. Update dependent ROS packages.

   The dependency of Autoware may change after the Docker image was created.
   In that case, you need to run the following commands to update the dependency.

   ```bash
   sudo apt update
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

6. Build the workspace.

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   If there is any build issue, refer to [Troubleshooting](../../support/troubleshooting.md#build-issues).

## How to update a workspace

1. Update the Docker image.

   ```bash
   docker pull ghcr.io/autowarefoundation/autoware-universe:latest-cuda
   ```

2. Launch a Docker container.

   - For amd64 architecture computers:

     ```bash
     rocker --nvidia --x11 --user --volume $HOME/autoware -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
     ```

   - If you want to run container without using NVIDIA GPU, or for arm64 architecture computers:

     ```bash
     rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/autoware -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
     ```

3. Update the `.repos` file.

   ```bash
   cd autoware
   git pull
   ```

4. Update the repositories.

   ```bash
   vcs import src < autoware.repos
   vcs pull src
   ```

5. Build the workspace.

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
