# Autoware Core Docker installation guide

## Prerequisites

| Item   | Requirement                                                  |
| ------ | ------------------------------------------------------------ |
| Docker | NVIDIA Container Toolkit (preferred, not currently required) |

## Select docker image

| Image                                          | Description                                                               |
| ---------------------------------------------- | ------------------------------------------------------------------------- |
| ghcr.io/autowarefoundation/autoware:core       | This is a runtime image that contains only the executables.               |
| ghcr.io/autowarefoundation/autoware:core-devel | This is a develop image that contains source code and build dependencies. |

## How to set up

Pull the Docker image you want to use. The following command is for autoware:core.

```bash
docker pull ghcr.io/autowarefoundation/autoware:core
```
