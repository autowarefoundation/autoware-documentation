# Autoware Core Docker installation guide

## Prerequisites

| Item   | Requirement                                                                                                                                       |
| ------ | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| Docker | [Docker Engine](https://docs.docker.com/engine/install/ubuntu/). Core images do not use the GPU, so the NVIDIA Container Toolkit is not required. |

To install Docker Engine with the Ansible playbook of the `autoware` repository, see step 2 of the [Docker installation](docker-installation.md#prerequisites) and add `--skip-tags nvidia`.

## Select docker image

| Image                                                | Description                                                                                                             |
| ---------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| ghcr.io/autowarefoundation/autoware:core-jazzy       | This is a runtime image. It contains the compiled Autoware Core packages and their runtime dependencies only.           |
| ghcr.io/autowarefoundation/autoware:core-devel-jazzy | This is a develop image. It contains the build dependencies and the compiled Autoware Core packages in `/opt/autoware`. |

Replace `jazzy` with `humble` for ROS 2 Humble. Tag scheme: `<stage>-<ros_distro>[-<date>|-<version>]`. For example, `core-jazzy-2.0.0` is the image of the Autoware 2.0.0 release. The full image catalog and tag list lives in [`autoware/docker/README.md`](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md).

## How to set up

1. Pull the Docker image you want to use. The following command is for `core-jazzy`:

   ```bash
   docker pull ghcr.io/autowarefoundation/autoware:core-jazzy
   ```

2. Start a container from the image.

   ```bash
   docker run --rm -it --net host ghcr.io/autowarefoundation/autoware:core-jazzy
   ```

   The entrypoint tries to enable multicast on `lo` and to apply the kernel settings for DDS. Without `--privileged`, these steps fail and the entrypoint prints `[entrypoint] WARN` lines. With `--net host`, the container uses the network of the host. In this case, apply the [DDS settings](../additional-settings-for-developers/network-configuration/dds-settings.md) and [enable multicast for `lo`](../additional-settings-for-developers/network-configuration/enable-multicast-for-lo.md) on the host, or add `--privileged` to the command.

   The container opens a Bash shell. In the runtime image, the entrypoint sources `/opt/ros/<distro>/setup.bash` and `/opt/autoware/setup.bash`. In the develop image, run `source /opt/autoware/setup.bash` yourself before you use the Autoware Core packages.

For the volume, display, and network options of `docker run`, see [`docker/README.md` → Usage](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#usage). To develop Autoware Core in Visual Studio Code, use the `core-devel` configuration of the [devcontainer](docker-installation.md#using-devcontainer-for-development).
