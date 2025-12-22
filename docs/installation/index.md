# Installation

## Target platforms

Autoware officially supports the platforms listed below.
Support for additional platforms may be introduced in future releases.

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

For details of how to enable object detection and traffic light detection/classification without a GPU, refer to the [Running Autoware without CUDA](../tutorials/others/running-autoware-without-cuda.md).

## Installing Autoware

There are multiple ways to install Autoware depending on your use case and experience level. You may also choose to install Autoware Core which contains only the essential Autoware packages, or Autoware Universe which contains the full Autoware stack.
For installation types, see the subsections below. If any issues occur during installation, refer to the [Support page](../community/support/index.md).

???+ abstract "Summary"

    <div class="cards" markdown>

    -   **Autoware Core**

        ---

        - contains only the essential Autoware packages
        - basic functionality for lane driving and obstacle detection
        - no NVidia dependency

    -   **Autoware Universe**

        ---

        - contains full stack Autoware and 3rd party packages
        - ML-based perception/planning capabilities

    </div>

### 1. Docker installation

Autoware's Open AD Kit containers enable you to run and develop Autoware easily on your host machine, ensuring the same environment for development and deployment without installing dependencies.

[Open AD Kit](https://autoware.org/open-ad-kit/) is the first [SOAFEE Blueprint](https://www.soafee.io/about/charter) for autonomous driving, offering extensible modular containerized workloads to simplify running Autoware's AD stack on distributed systems. Refer to the [Open AD Kit Documentation](https://autowarefoundation.github.io/openadkit/) for more details.


[:fa-cl-s fa-circle-arrow-right: Autoware Universe Docker Installation](autoware/docker-installation.md){ .md-button } [:fa-cl-s fa-circle-arrow-right: Autoware Core Docker Installation](autoware/core-docker-installation.md){ .md-button }

### 2. Source installation

Source installation is for the cases where more granular control of the installation environment is needed.
It is recommended for experienced users or people who want to customize their environment.
Note that some problems may occur depending on your local environment.

[:fa-cl-s fa-circle-arrow-right: Autoware Universe Source Installation](autoware/source-installation.md){ .md-button } [:fa-cl-s fa-circle-arrow-right: Autoware Core Source Installation](autoware/core-source-installation.md){ .md-button }


### 3. Debian Package installation

Autoware Core packages are available on the ROS build farm.
If you have an environment where ROS is set up, Autoware packages can be easily installed and used with other packages in the ROS ecosystem.

[:fa-cl-s fa-circle-arrow-right: Autoware Core Debian Package Installation](autoware/core-debian-installation.md){ .md-button }

## Installing related tools

Some other tools are required depending on the evaluation you want to do.
For example, to run an end-to-end simulation you need to install an appropriate simulator.

For more information, see [here](related-tools/index.md).

## Additional settings for developers

There are also tools and settings for developers, such as Shells or IDEs.

For more information, see [here](additional-settings-for-developers/index.md).
