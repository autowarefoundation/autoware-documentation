# Docker installation

!!! info

    Since this page explains Docker-specific information, it is recommended to see [Source installation](./source-installation.md) as well if you need detailed information.

## Prerequisites

- [Git](https://git-scm.com/)

## How to set up a development environment

1. Clone `autowarefoundation/autoware` and move to the directory.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git
   ```

2. Install the dependencies.

   ```bash
   ./setup-dev-env.sh docker
   ```

## How to set up a workspace

1. Launch a Docker container

   ```bash
   rocker --nvidia --x11 --user --volume $HOME/autoware -- ghcr.io/autowarefoundation/autoware-universe:latest
   ```

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
   rocker --nvidia --x11 --user --volume $HOME/autoware -- ghcr.io/autowarefoundation/autoware-universe:latest
   ```

2. Update the `.repos` file.

   ```bash
   cd autoware
   git pull
   ```

3. Update the repositories.

   ```bash
   vcs import src < autoware.repos
   vcs pull
   ```

4. Build the workspace.

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
