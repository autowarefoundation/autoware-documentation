# Installation without ADE

## Goals

This article demonstrates how to successfully build [Autoware.Core](https://www.autoware.core/) applications without the ade framework.

## Installation Requirements

To compile [Autoware.Core project](https://www.autoware.core/) from sources, the following tools must be installed in the system.

- Apt packages

```{bash}
sudo apt install -y git cmake python3-pip
```

- Python modules

```{bash}
pip3 install -U colcon-common-extensions vcstool
```

## ROS 2 core

First, the [ROS 2](https://index.ros.org/doc/ros2/) core components and tools must be installed. The full guide is available at [ROS 2 Installation](https://index.ros.org/doc/ros2/Installation/).
Once installed source the setup file:

```{bash}
source /opt/ros/$ROS_DISTRO/setup.bash
```

where `ROS_DISTRO` is one of the supported version mentioned in [Target Software Platforms](target-environments.md#target-software-platforms).

## ROS 2 package dependencies

[Autoware.Core project](https://www.autoware.core/) requires some [ROS 2](https://index.ros.org/doc/ros2/) packages in addition to the core components.
The tool `rosdep` allows an automatic search and installation of such dependencies.

```{bash}
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
```

Once installed, dependencies can be deduced from the sources of the [Autoware.Core project](https://www.autoware.core/).

```{bash}
git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
cd AutowareAuto
vcs import < autoware.auto.$ROS_DISTRO.repos
export ROS_VERSION=2
rosdep install -y -i --from-paths src
```

Checkout the [latest release](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/releases) by checking out the corresponding tag or release branch.
Alternatively, when not checking out any specific tag, the latest `master` branch will be used
which may include features that are still being developed. For example:

```{bash}
git checkout tags/1.0.0 -b release-1.0.0
```

Next, to compile the source code, see @ref building.

## Troubleshooting

In case where `ros-foxy-autoware-auto-msgs` is installed on the system, colcon uses it instead of
the one in the `AutowareAuto/src/external/` folder. This may cause errors.
To prevent this, please remove the package by:

```{bash}
sudo apt purge -y ros-foxy-autoware-auto-msgs
```

In case an error occurs related to `acado` package, please run:

```{bash}
sudo apt purge -y ros-foxy-acado-vendor
sudo apt install -y ros-foxy-acado-vendor
```
