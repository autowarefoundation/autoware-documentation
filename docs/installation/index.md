# Installation

## Target platforms

Autoware 2.0.0 officially supports Ubuntu 22.04 with ROS 2 Humble and Ubuntu 24.04 with ROS 2 Jazzy, on the hardware listed below.
Support for additional platforms may be introduced in future releases.

<div class="aw-specs" markdown>

- :fa-cl-s fa-server: <span>**amd64, arm64**<br>Architecture</span>
- :fa-cl-s fa-microchip: <span>**8 cores**<br>CPU</span>
- :fa-cl-s fa-memory: <span>**16GB**<br>RAM</span>
- :fa-cl-s fa-bolt: <span>**4GB RAM, optional**<br>NVIDIA GPU</span>

</div>

!!! info "Minimum hardware requirements"

    Autoware is scalable and can be customized to work with distributed or less powerful hardware.
    The minimum hardware requirements given above are just a general recommendation.
    However, performance will be improved with more cores, RAM and a higher-spec graphics card or GPU core.

    Although GPU is not required to run basic functionality, it is mandatory to enable the following neural network related functions:

    - LiDAR based object detection
    - Camera based object detection
    - Traffic light detection and classification

    For details of how to enable object detection and traffic light detection/classification without a GPU, refer to the [Running Autoware without CUDA](../tutorials/others/running-autoware-without-cuda.md).

## Installing Autoware

There are multiple ways to install Autoware depending on your use case and experience level. You may also choose to install Autoware Core which contains only the essential Autoware packages, or Autoware Universe which contains the full Autoware stack.
For installation types, see the subsections below. If any issues occur during installation, refer to the [Support page](../community/support/index.md).

<div class="aw-editions" markdown>

<div class="aw-edition aw-edition--featured" markdown>

<span class="aw-edition__badge">:fa-cl-s fa-star: Full stack</span>

### Autoware Universe

- Contains full stack Autoware and 3rd party packages
- ML-based perception/planning capabilities

</div>

<div class="aw-edition" markdown>

<span class="aw-edition__badge">Essentials</span>

### Autoware Core

- Contains only the essential Autoware packages
- Basic functionality for lane driving and obstacle detection
- No NVIDIA dependency
- No neural networks for detecting objects (yet)

</div>

</div>

<div class="aw-method" markdown>

<span class="aw-method__icon">:fontawesome-brands-docker:</span>

<div class="aw-method__body" markdown>

### 1. Docker installation

Autoware publishes prebuilt Docker images on GHCR that let you run and develop Autoware easily on your host machine, ensuring the same environment for development and deployment without installing dependencies.

For containerized deployment patterns and integrations beyond local development, see [Open AD Kit](https://autoware.org/open-ad-kit/) (the first [SOAFEE Blueprint](https://www.soafee.io/charter/) for autonomous driving) and the [Open AD Kit Documentation](https://autowarefoundation.github.io/openadkit/).

</div>

<div class="aw-method__actions" markdown>

[:fa-cl-s fa-circle-arrow-right: Autoware Universe](autoware/docker-installation.md){ .md-button .md-button--primary }
[:fa-cl-s fa-circle-arrow-right: Autoware Core](autoware/core-docker-installation.md){ .md-button }

</div>

</div>

<div class="aw-method" markdown>

<span class="aw-method__icon">:fa-cl-s fa-code:</span>

<div class="aw-method__body" markdown>

### 2. Source installation

Source installation is for the cases where more granular control of the installation environment is needed.
It is recommended for experienced users or people who want to customize their environment.
Note that some problems may occur depending on your local environment.

</div>

<div class="aw-method__actions" markdown>

[:fa-cl-s fa-circle-arrow-right: Autoware Universe](autoware/source-installation.md){ .md-button .md-button--primary }
[:fa-cl-s fa-circle-arrow-right: Autoware Core](autoware/core-source-installation.md){ .md-button }

</div>

</div>

<div class="aw-method" markdown>

<span class="aw-method__icon">:fa-cl-s fa-box-archive:</span>

<div class="aw-method__body" markdown>

### 3. Debian package installation

Autoware Core packages are available on the ROS build farm.
If you have an environment where ROS is set up, Autoware packages can be easily installed and used with other packages in the ROS ecosystem.

</div>

<div class="aw-method__actions" markdown>

[:fa-cl-s fa-circle-arrow-right: Autoware Core](autoware/core-debian-installation.md){ .md-button }

</div>

</div>

<div class="aw-callout" markdown>

## Extending Autoware with community packages

The [Autoware Index](autoware/autoware-index.md) is a registry of community ROS 2 packages that extend Autoware.
Once you have a source workspace set up, you can pull additional packages from the index with a generated `.repos` file.

[:fa-cl-s fa-cubes: Autoware Index (Community Packages)](autoware/autoware-index.md){ .md-button }

</div>

## Related tools and settings

<div class="grid cards aw-cards aw-cards--small" markdown>

- :fa-cl-s fa-toolbox: **[Installing related tools](related-tools/index.md)**

  Some other tools are required depending on the evaluation you want to do. For example, to run an end-to-end simulation you need to install an appropriate simulator.

- :fa-cl-s fa-sliders: **[Additional settings for developers](additional-settings-for-developers/index.md)**

  There are also settings for developers, such as the ROS 2 console output and the network configuration for DDS and Zenoh.

</div>
