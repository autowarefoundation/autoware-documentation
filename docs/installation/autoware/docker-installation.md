# Docker installation

!!! info

    Since this page explains Docker-specific information, it is recommended to see [Source installation](./source-installation.md) as well if you need detailed information.

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

### Installing dependencies manually

- [Install Docker Engine](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/docker_engine#manual-installation)
- [Install Docker Compose](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/docker_compose#manual-installation)
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

1. Launch a Docker container.

   - For amd64 architecture computers with NVIDIA GPU:

     ```bash
     rocker --nvidia --x11 --user --volume $HOME/autoware -- ghcr.io/autowarefoundation/autoware-universe:latest
     ```

   - If you want to run container without using NVIDIA GPU, or for arm64 architecture computers:

     ```bash
     rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/autoware -- ghcr.io/autowarefoundation/autoware-universe:latest
     ```

     For detailed reason could be found [here](#docker-with-nvidia-gpu-fails-to-start-autoware-on-arm64-devices)

   See [here](https://github.com/autowarefoundation/autoware/tree/main/docker/README.md) for more advanced usage.

   After that, move to the workspace in the container:

   ```bash
   cd autoware
   ```

2. Create the `src` directory and clone repositories into it.

   ```bash
   mkdir src
   vcs import src < autoware.repos
   ```

3. Build the workspace.

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

## How to update a workspace

1. Update the Docker image.

   ```bash
   docker pull ghcr.io/autowarefoundation/autoware-universe:latest
   ```

2. Launch a Docker container.

   - For amd64 architecture computers:

     ```bash
     rocker --nvidia --x11 --user --volume $HOME/autoware -- ghcr.io/autowarefoundation/autoware-universe:latest
     ```

   - If you want to run container without using NVIDIA GPU, or for arm64 architecture computers:

     ```bash
     rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/autoware -- ghcr.io/autowarefoundation/autoware-universe:latest
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

## Troubleshooting

Here are solutions for a few specific errors:

### cuda error: forward compatibility was attempted on non supported hw

When starting Docker with GPU support enabled for NVIDIA graphics, you may sometimes receive the following error:

```bash
docker: Error response from daemon: OCI runtime create failed: container_linux.go:349: starting container process caused "process_linux.go:449: container init caused \"process_linux.go:432: running prestart hook 0 caused \\\"error running hook: exit status 1, stdout: , stderr: nvidia-container-cli: initialization error: cuda error: forward compatibility was attempted on non supported hw\\\\n\\\"\"": unknown.
ERROR: Command return non-zero exit code (see above): 125
```

This usually indicates that a new NVIDIA graphics driver has been installed (usually via `apt`) but the system has not yet been restarted. A similar message may appear if the graphics driver is not available, for example because of resuming after suspend.

To fix this, restart your system after installing the new NVIDIA driver.

### Docker with NVIDIA gpu fails to start Autoware on arm64 devices

When starting Docker with GPU support enabled for NVIDIA graphics on arm64 devices, e.g. NVIDIA jetson AGX xavier, you may receive the following error:

```bash
nvidia@xavier:~$ rocker --nvidia --x11 --user --volume $HOME/autoware -- ghcr.io/autowarefoundation/autoware-universe:latest-arm64
...

Collecting staticx==0.12.3
Downloading https://files.pythonhosted.org/packages/92/ff/d9960ea1f9db48d6044a24ee0f3d78d07bcaddf96eb0c0e8806f941fb7d3/staticx-0.12.3.tar.gz (68kB)
Complete output from command python setup.py egg_info:
Traceback (most recent call last):
File "", line 1, in
File "/tmp/pip-install-m_nm8mya/staticx/setup.py", line 4, in
from wheel.bdist_wheel import bdist_wheel
ModuleNotFoundError: No module named 'wheel'

Command "python setup.py egg_info" failed with error code 1 in /tmp/pip-install-m_nm8mya/staticx/
...
```

This error exists in current version of rocker tool, which relates to the os_detection function of rocker.

To fix this error, temporary modification of rocker source code is required, which is not recommended.

At current stage, it is recommended to run docker without NVIDIA gpu enabled for arm64 devices:

```bash
rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/autoware -- ghcr.io/autowarefoundation/autoware-universe:latest
```

This tutorial will be updated after official fix from rocker.

## Tips

### Non-native arm64 System

This section describes a process to run `arm64` systems on `amd64` systems using [`qemu-user-static`](https://github.com/multiarch/qemu-user-static).

Initially, your system is usually incompatible with `arm64` systems.
To check that:

```sh-session
$ docker run --rm -t arm64v8/ubuntu uname -m
WARNING: The requested image's platform (linux/arm64/v8) does not match the detected host platform (linux/amd64) and no specific platform was requested
standard_init_linux.go:228: exec user process caused: exec format error
```

Installing `qemu-user-static` enables us to run `arm64` images on `amd64` systems.

```sh-session
$ sudo apt-get install qemu-user-static
$ docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
$ docker run --rm -t arm64v8/ubuntu uname -m
WARNING: The requested image's platform (linux/arm64/v8) does not match the detected host platform (linux/amd64) and no specific platform was requested
aarch64
```

To run Autoware's Docker images of `arm64` architecture, add the suffix `-arm64`.

```sh-session
$ docker run --rm -it ghcr.io/autowarefoundation/autoware-universe:latest-arm64
WARNING: The requested image's platform (linux/arm64) does not match the detected host platform (linux/amd64) and no specific platform was requested
root@5b71391ad50f:/autoware#
```
