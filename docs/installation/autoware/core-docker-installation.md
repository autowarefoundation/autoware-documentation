# Autoware Core Docker installation guide

## Prerequisites

| Item   | Requirement                                                  |
| ------ | ------------------------------------------------------------ |
| Docker | NVIDIA Container Toolkit (preferred, not currently required) |

## Select docker image

| Image                                                | Description                                                               |
| ---------------------------------------------------- | ------------------------------------------------------------------------- |
| ghcr.io/autowarefoundation/autoware:core-jazzy       | This is a runtime image that contains only the executables.               |
| ghcr.io/autowarefoundation/autoware:core-devel-jazzy | This is a develop image that contains source code and build dependencies. |

Replace `jazzy` with `humble` for ROS 2 Humble. Tag scheme: `<stage>-<ros_distro>[-<date>|-<version>]`. The full image catalog and tag list lives in [`autoware/docker/README.md`](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md).

## How to set up

Pull the Docker image you want to use. The following command is for `core-jazzy`:

```bash
docker pull ghcr.io/autowarefoundation/autoware:core-jazzy
```
