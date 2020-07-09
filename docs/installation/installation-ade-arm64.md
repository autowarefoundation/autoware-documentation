Installation with ADE for arm64 Systems {#installation-ade-arm64}
=======================================

@tableofcontents

# Goals {#installation-and-development-goals-arm64}

This article demonstrates how to launch AutowareAuto using ADE for `arm64` systems and those
wishing to develop using `arm64`. 
This document will cover both native and non-native systems using ADE.

# Native arm64 System {#native-arm64-installation}

The following section describes the process required to launch the ade environment on an `arm64`
based system.

## Prerequisites

Ensure that ADE has been installed and the `.adehome` file has been created. To complete this,
follow the instructions in @ref installation-and-development-install-ade then subsequently @ref
installation-and-development-setup-ade-home-and-project-checkout.

## Launch ADE arm64 Docker

Enter the development directory and launch the ADE docker:

```{bash}
$ cd ~/adehome/AutowareAuto
$ ade --rc .aderc-arm64 start --update --enter
```

# Non-native arm64 System {#non-native-arm64-installation}

The following section describes the process to run multi-architecture systems using Docker,
`binfmt`, and `qemu`.

## Prerequisites

Before alternative architectures can be run on a system, ensure that one can run ADE and
AutowareAuto on the native architecture.
Those with `amd64` systems should follow the instructions in @ref installation-ade and ensure
all dependencies are properly installed.
The following will assume that all ADE and AutowareAuto dependencies have been installed.

@note The emulation library used for this section is currently only compatible with `x86_64`.
Check your system architecture using the following command:
```{bash}
$ uname -m
```

To check the systems which Docker is compatible with run the following command:
```{bash}
$ docker buildx ls
```

The output the following should look like this:
```{bash}
$ docker buildx ls
NAME/NODE DRIVER/ENDPOINT STATUS  PLATFORMS
default * docker
  default default         running linux/amd64, linux/386  
```

To check that your system is currently incompatible with `arm64` systems is by running:
```{bash}
$ docker run --rm -t arm64v8/ubuntu uname -m
```
The output of this should error and indicate that libraries were not found.

## Configuring Docker for Multi-architecture Emulation

First, install emulation and binary support libraries that will allow Docker to run multiple
architectures.
The libraries [`qemu`](https://www.qemu.org/) and [`qemu-user-static`](https://github.com/multiarch/qemu-user-static)
provide emulation support allowing Docker to interpret alternative architectures on an `x86_64` environment.
The kernel module [`binfmt-support`](http://binfmt-support.nongnu.org/) allows for the registry
and invocation of binary interpreters at the system administrator level. 
```{bash}
$ sudo apt-get install qemu binfmt-support qemu-user-static
```

Finally, invoke the `qemu-user-static` docker image to install and link the interpreters and
architectures for various architectures.
```{bash}
$ docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

To check that the installation and registry was successful, run the following command and ensure
that it exits cleanly:
```{bash}
$ docker run --rm -t arm64v8/ubuntu uname -m
...
aarch64
```

Additional checks include running the `buildx` option with Docker.
This should output a larger variety of build types available to Docker.

@note There will be an initial warning that the architecture of the image Docker is trying to bring
up is different to the architecture of the system.
```{bash}
WARNING: The requested image's platform (linux/arm64) does not match the detected host platform (linux/amd64) and no specific platform was requested
```

## Launching ADE

Now that the set-up is complete, the `arm64` ADE image can be launched with no issues
```{bash}
$ ade --rc .aderc-arm64 start --update --enter
```

@warning Launching ADE in the non-native environment leads to incredibly reduced performance as
compared to a native configuration due to the binary translation that is happening within Docker;
approximately 5x that of a native system.
