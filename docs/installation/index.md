# Installation

## Target platforms

Autoware targets the platforms listed below. It may change in future versions of Autoware.

The Autoware Foundation provides no support on other platforms than those listed below.

### Architecture

- amd64
- arm64

### Minimum hardware requirements

!!! info

    Autoware is scalable and can be customized to work with distributed or less powerful hardware.
    The minimum hardware requirements given below are just a general recommendation.
    However, performance will be improved with more cores, RAM and a higher-spec graphics card or GPU core.

    Although GPU is not required to run basic functionality, it is mandatory to enable the following neural network related functions:
        - LiDAR based object detection
        - Camera based object detection
        - Traffic light detection and classification

- CPU with 8 cores
- 16GB RAM
- [Optional] NVIDIA GPU (4GB RAM)

For details of how to enable object detection and traffic light detection/classification without a GPU, refer to the [Running Autoware without CUDA](../how-to-guides/others/running-autoware-without-cuda.md).

## Installing Autoware

There are two ways to set up Autoware. Choose one according to your preference.

If any issues occur during installation, refer to the [Support page](../support/index.md).

### 1. Docker installation

Autoware's Open AD Kit containers enables you to
run Autoware easily on your host machine ensuring same environment for all deployments without installing any dependencies. Full Guide on [Docker Installation Setup](autoware/docker-installation.md).

[Open AD Kit](https://autoware.org/open-ad-kit/) is also the First [SOAFEE Blueprint](https://www.soafee.io/about/charter) for autonomous driving that offers extensible modular containers for making it easier to run Autoware's AD stack on distributed systems. Full Guide on [Open AD Kit Setup](https://autowarefoundation.github.io/open-ad-kit-docs/openadkit_v3/version-3.0/).

Developer containers are also available for developers making it easier to build Autoware from source and ensuring same environment for all developers.

### 2. Source installation

Source installation is for the cases where more granular control of the installation environment is needed.
It is recommended for experienced users or people who want to customize their environment.
Note that some problems may occur depending on your local environment.

For more information, refer to the [source installation guide](autoware/source-installation.md).

## Installing related tools

Some other tools are required depending on the evaluation you want to do.
For example, to run an end-to-end simulation you need to install an appropriate simulator.

For more information, see [here](related-tools/index.md).

## Additional settings for developers

There are also tools and settings for developers, such as Shells or IDEs.

For more information, see [here](additional-settings-for-developers/index.md).
