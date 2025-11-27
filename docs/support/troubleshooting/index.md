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
For more details, see the [manual page of GNU make](https://www.gnu.org/software/make/manual/make.html#Parallel-Disable).

By reducing the number of packages built in parallel, you can also reduce the amount of memory used.
In the following example, the number of packages built in parallel is set to 1, and the number of jobs used by `make` is limited to 1.

```bash
MAKEFLAGS="-j1" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
```

!!! note

    By lowering both the number of packages built in parallel and the number of jobs used by `make`, you can reduce the memory usage.
    However, this also means that the build process takes longer.

### Errors when using the latest version of Autoware

If you are working with the latest version of Autoware, issues can occur due to out-of-date software or old build files.

To resolve these types of problems, first try cleaning your build artifacts and rebuilding:

```bash
rm -rf build/ install/ log/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If the error is not resolved, remove `src/` and update your workspace according to installation type ([Docker](../../installation/autoware/docker-installation.md#update-the-workspace) / [source](../../installation/autoware/source-installation.md#how-to-update-a-workspace)).

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

### Performance related issues

Symptoms:

- Autoware is running slower than expected
- Messages show up late in RViz2
- Point clouds are lagging
- Camera images are lagging behind
- Point clouds or markers flicker on RViz2
- When multiple subscribers use the same publishers, the message rate drops

If you have any of these symptoms, please the [Performance Troubleshooting](performance-troubleshooting.md) page.

### Map does not display when running the Planning Simulator

When running the Planning Simulator, the most common reason for the map not being displayed in RViz is because [the map path has not been specified correctly in the launch command](../../tutorials/ad-hoc-simulation/planning-simulation.md#lane-driving-scenario). You can confirm if this is the case by searching for `Could not find lanelet map under {path-to-map-dir}/lanelet2_map.osm` errors in the log.

Another possible reason is that map loading is taking a long time due to poor DDS performance. For this, please visit the [Performance Troubleshooting](performance-troubleshooting.md) page.

### Died process issues

Some modules may not be launched properly at runtime, and you may see "process has died" in your terminal.
You can use the gdb tool to locate where the problem is occurring.

Debug build the module you wish to analyze under your autoware workspace

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-up-to <the modules you wish to analyze> --catkin-skip-building-tests --symlink-install
```

In this state, when a died process occurs when you run the autoware again, a core file will be created.
Remeber to remove the size limit of the core file.

```bash
ulimit -c unlimited
```

Rename the core file as `core.<PID>`.

```bash
echo core | sudo tee /proc/sys/kernel/core_pattern
echo -n 1 | sudo tee /proc/sys/kernel/core_uses_pid
```

Launch the autoware again. When a died process occurs, a core file will be created.
`ll -ht` helps you to check if it was created.

Invoke gdb tool.

```bash
gdb <executable file> <core file>
#You can find the `<executable file>` in the error message.
```

`bt` backtraces the stack of callbacks where a process dies.
`f <frame number>` shows you the detail of a frame, and `l` shows you the code.
