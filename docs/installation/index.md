# Installation

## Target platforms

Autoware targets the platforms listed below. It may change in future versions of Autoware.

The Autoware Foundation provides no support on other platforms than those listed below.

### Architecture

- amd64
- arm64

### Minimum hardware requirements

!!! info

    Autoware is scalable and may work with less powerful hardware or distributed hardware under customization. The below minimum hardware requirements are just recommendation in general.   

- CPU with 8 cores
- 16GB RAM
> Performance will be improved with more cores, RAM and a higher-spec graphics card or GPU core.

- [Optional] NVIDIA GPU (4GB RAM)
> Although GPU is not required to run basic functionality, it is mandatory to enable the following neural network related functions:
> - LiDAR based object detection
> - Camera based object detection
> - Traffic light detection and classification

## Installing Autoware

There are two ways to set up Autoware. Choose one according to your preference.

If any issues occur during installation, refer to the [Support page](https://autowarefoundation.github.io/autoware-documentation/main/support).

### 1. Docker installation

Docker can ensure that all developers in a project have a common, consistent development environment.
It is recommended for beginners, casual users, people who are unfamiliar with Ubuntu.

For detailed steps, refer to the [Docker installation guide](autoware/docker-installation.md) .

### 2. Source installation

Source installation is for the cases where more granular control of the installation environment is needed.
It is recommended for experienced users or people who want to customize their environment.  
Note that some problems may occur depending on your local environment.

For detailed steps，refer to the [Source installation guide](autoware/source-installation.md) .

## Installing related tools

Some other tools are required depending on the evaluation you want to do.
For example, to run an end-to-end simulation you need to install an appropriate simulator.

To install the tools, refer to [Related tools](related-tools).

## Additional settings for developers

There are also tools and settings for developers, such as Shells or IDEs.

For the tools and settings, refer to [Additional settings for developers](additional-settings-for-developers).
