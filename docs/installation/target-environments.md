System Dependencies and Target Environments {#target-environments}
===========================================

@tableofcontents

Autoware.Auto targets the environments and applications listed below, for which automated tests ensure the code works properly. The target environments may change in future versions of Autoware.Auto.

The Autoware foundation provides no support on other platforms than those listed below.

# Target Hardware Platforms {#target-environments-hardware}

- `amd64` / `x86_64` (Intel/AMD)
- `arm64` / `aarch64` / `arm64v8` (ARM v8, 64-bit)

# Target Software Platforms {#target-environments-software}

| ROS Version                         | Operating System | System Dependencies
|-------------------------------------|------------------|------------------------------------------------------------------------------------------------|
| ROS2 Foxy (**active development**)  | Ubuntu 20.04 LTS | [REP-2000 section](https://www.ros.org/reps/rep-2000.html#foxy-fitzroy-may-2020-may-2023)      |
| ROS2 Dashing (**maintenance only**) | Ubuntu 18.04 LTS | [REP-2000 section](https://www.ros.org/reps/rep-2000.html#dashing-diademata-may-2019-may-2021) |
