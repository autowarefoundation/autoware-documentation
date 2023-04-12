# Troubleshooting

## Setup issues

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
    "tensorrt*"         \
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
    "tensorrt*"         \
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

    Note that some components in Autoware Universe require CUDA, and only the CUDA version in the [env file](https://github.com/autowarefoundation/autoware/blob/main/amd64.env) is supported at this time.
    Autoware may work with other CUDA versions, but those versions are not supported and functionality is not guaranteed.

## Build issues

### Insufficient memory

Building Autoware requires a lot of memory, and your machine can freeze or crash if memory runs out during a build. To avoid this problem, 16-32GB of swap should be configured.

```bash
# Optional: Check the current swapfile
free -h

# Remove the current swapfile
sudo swapoff /swapfile
sudo rm /swapfile

# Create a new swapfile
sudo fallocate -l 32G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Optional: Check if the change is reflected
free -h
```

For more detailed configuration steps, along with an explanation of swap, refer to Digital Ocean's ["How To Add Swap Space on Ubuntu 20.04" tutorial](https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-20-04)

If there are too many CPU cores (more than 64) in your machine, it might requires larger memory.
A workaround here is to limit the job number while building.

```bash
MAKEFLAGS="-j4" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

You can adjust `-j4` to any number based on your system.

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

### Error when building python package

During building the following issue can occurs

```bash
pkg_resources.extern.packaging.version.InvalidVersion: Invalid version: '0.23ubuntu1'
```

The error is due to the fact that for versions between 66.0.0 and 67.5.0 `setuptools` enforces the python packages to be
[PEP-440](https://peps.python.org/pep-0440/) conformant.
Since version 67.5.1 `setuptools` has a [fallback](https://github.com/pypa/setuptools/commit/1640731114734043b8500d211366fc941b741f67) that makes it possible to work with old packages again.

The solution is to update `setuptools` to the newest version with the following command

```bash
pip install --upgrade setuptools
```

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

## Runtime issues

### Map does not display when running the Planning Simulator

When running the Planning Simulator, the most common reason for the map not being displayed in RViz is because [the map path has not been specified correctly in the launch command](../tutorials/ad-hoc-simulation/planning-simulation.md#how-to-run-a-planning-simulation). You can confirm if this is the case by searching for `Could not find lanelet map under {path-to-map-dir}/lanelet2_map.osm` errors in the log.

Another possible reason is that map loading is taking a long time due to poor DDS performance. To address this issue, first enable localhost-only communication to reduce network traffic, and then [tune DDS settings](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html) if the problem continues to occur.

Simply put, add the following settings to `.bashrc` and reboot the terminal. In many cases this is not a problem.

```bash
export ROS_LOCALHOST_ONLY=1
if [ ! -e /tmp/cycloneDDS_configured ]; then
  sudo sysctl -w net.core.rmem_max=2147483647
  sudo ip link set lo multicast on
  touch /tmp/cycloneDDS_configured
fi
```

!!! note

    DDS configuration can be determined by running the following command.

    ```bash
    echo $RMW_IMPLEMENTATION  // if Cyclone DDS is configured, this command will return "rmw_cyclonedds_cpp"
    ```

If that does not work or you need more information, read the following documents.

1. [Enable localhost-only communication](../installation/additional-settings-for-developers/index.md#enabling-localhost-only-communication)
2. [DDS settings](../installation/additional-settings-for-developers/index.md#tuning-dds)

### Multicast is disabled

If you get the error message `selected interface "{your-interface-name}" is not multicast-capable: disabling multicast`, run the following command to allow multicast.

```bash
sudo ip link set multicast on {your-interface-name}
```

### Node performance degradation

If you notice a decrease in the running performance of a node, such as [issue2597](https://github.com/autowarefoundation/autoware.universe/issues/2597#issuecomment-1491789081), you need to check if your compilation instructions use `Release` or `RelWithDebInfo` tags. If not, recompile the project using the following instructions:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
