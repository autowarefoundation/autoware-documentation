# Troubleshooting

- [Setup script](#setup-script)
  - [CUDA-related errors](#cuda-related-errors)
- [Build errors](#build-errors)
  - [Insufficient memory](#insufficent-memory)
  - [Errors when using the latest version of Autoware](#errors-when-using-the-latest-version-of-autoware)
  - [Errors when using a fixed version of Autoware](#errors-when-using-a-fixed-version-of-autoware)  
- [Docker/rocker issues](#dockerrocker-issues)
- [Runtime problems](#runtime-problems)
  - [Map does not display when running the Planning Simulator](#map-does-not-display-when-running-the-planning-simulator)

## Setup script

### CUDA-related errors

When installing CUDA, errors may occur because of version conflicts. To resolve these types of errors, try one of the following methods:

- Unhold all CUDA-related libraries and rerun the setup script.

  ```bash
  sudo apt-mark unhold  \
    "cuda*"             \
    "libcudnn*"         \
    "libnvinfer*"       \
    "libnvonnxparsers*" \
    "libnvparsers*"     \
    "nvidia*"

  ./setup-dev-env.sh
  ```

- Uninstall all CUDA-related libraries and rerun the setup script.

  ```bash
  sudo apt purge        \
    "cuda*"             \
    "libcudnn*"         \
    "libnvinfer*"       \
    "libnvonnxparsers*" \
    "libnvparsers*"     \
    "nvidia*"

  sudo apt autoremove

  ./setup-dev-env.sh
  ```

!!! warning

    Note that this may break your system and run carefully.

- Run the setup script without installing CUDA-related libraries.

  ```bash
  ./setup-dev-env.sh --no-nvidia
  ```

!!! warning

    Note that some components in Autoware require CUDA.

    Also, although Autoware could work with different versions of CUDA libraries as well,
    remember it is not officially supported and may sometimes be broken.

## Build errors

### Insufficent memory

Building Autoware requires a lot of memory, and your machine can freeze or crash if memory runs out during a build. To avoid this problem, 16-32GB of swap should be configured. For details on how to configure a swap file, refer to [this Digital Ocean guide](https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-20-04).

### Errors when using the latest version of Autoware

If you are working with the latest version of Autoware, issues can occur due to out-of-date software or old build files.

To resolve these types of problems, first try cleaning your build artifacts and rebuilding:

```bash
rm -rf build/ install/ log/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If the error is not resolved, remove `src/` and update your workspace according to installation type ([Docker](../installation/autoware/docker-installation.md#how-to-update-a-workspace) / [source](../installation/autoware/source-installation.md#how-to-update-a-workspace)).

!!! Warning

    Before removing `src/`, confirm that there are no modifications in your local environment that you want to keep!

If errors still persist after trying the steps above, delete the entire workspace, clone the repository once again and restart the installation process.

```bash
rm -rf autoware/
git clone https://github.com/autowarefoundation/autoware.git
```

### Errors when using a fixed version of Autoware

In principle, errors should not occur when using a fixed version. That said, possible causes include:

- ROS 2 has been updated with breaking changes.
  - For confirmation, check the [Packaging and Release Management](https://discourse.ros.org/c/release/16) tag on ROS Discourse.
- Your local environment is broken.
  - Confirm your `.bashrc` file, environment variables, and library versions.

In addition to the causes listed above, there are two common misunderstandings around the use of fixed versions.

1. You used a fixed version for `autowarefoundation/autoware` only.
   All of the repository versions in the `.repos` file must be specified in order to use a completely fixed version.

2. You didn't update the workspace after changing the branch of `autowarefoundation/autoware`.
   Changing the branch of `autowarefoundation/autoware` does not affect the files under `src/`. You have to run the `vcs import` command to update them.

## Docker/rocker issues

If any errors occur when running Autoware with Docker or rocker, first confirm that your Docker installation is working correctly by running the following commands:

```bash
docker run --rm -it hello-world
docker run --rm -it ubuntu:latest
```

Next, confirm that you are able to access the base Autoware image that is stored on the GitHub Packages website

```bash
docker run --rm -it ghcr.io/autowarefoundation/autoware-universe:latest
```

## Runtime errors

### Map does not display when running the Planning Simulator

When running the Planning Simulator, the most common reason for the map not being displayed in RViz is because [the map path has not been specified correctly in the launch command](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/#how-to-run-a-planning-simulation). You can confirm if this is the case by searching for `Could not find lanelet map under {path-to-map-dir}/lanelet2_map.osm` errors in the log.

Another possible reason is that map loading is taking a long time due to poor DDS performance. To address this issue, first enable localhost-only communication to reduce network traffic, and then [tune DDS settings](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html) if the problem continues to occur.

1. [Enable localhost-only communication](https://autowarefoundation.github.io/autoware-documentation/main/installation/tools-for-developers/#enabling-localhost-only-communication)

2. Tune DDS settings

Add the following lines to `/etc/sysctl.conf`

```bash
net.ipv4.ipfrag_time=3  // generic DDS setting
net.ipv4.ipfrag_high_thresh=134217728 // generic DDS setting
net.core.rmem_max=2147483647 // only add if CycloneDDS is configured
net.core.rmem_default=8388608 // only add if CycloneDDS is confgured
```

!!! note

    DDS configuration can be determined by running the following command.

    ```bash
    echo $RMW_IMPLEMENTATION  // if Cyclone DDS is configured, this command will return "rmw_cyclonedds_cpp"
    ```
