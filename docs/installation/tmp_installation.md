# Building

## Prerequisites

You need to be inside an ADE container, or have installed the dependencies manually. See [installation](installation.md).

If you haven't done so already, get the source code with

```{bash}
cd AutowareCore
vcs import < autoware.core.$ROS_DISTRO.repos
```

Optionally, you can choose a DDS implementation other than the default Cyclone DDS: [choosing-a-dds-vendor](choosing-a-dds-vendor.md).

If running tests or demos, also pull binary files with

```{bash}
git lfs pull --exclude="" --include="*"
```

## Use colcon defaults

It is strongly advised to use the colcon defaults file provided by Autoware.Core to ensure that
binaries are consistently built with the same flags.

Test if the required environment variable
[`COLCON_DEFAULTS_FILE`](https://colcon.readthedocs.io/en/released/reference/global-arguments.html?highlight=DEFAULTS#environment-variables)
has been properly set by typing the following:

```{bash}
echo $COLCON_DEFAULTS_FILE
```

If the output is empty, follow the instruction below to configure it.

### Inside ADE

After creating a fresh new ADE home according to [Installation with ADE](installation-ade.md), the `.bashrc` will be
populated to set the `COLCON_DEFAULTS_FILE` environment variable.

To use the colcon defaults configuration file with an existing ADE home, type the following from within ADE

```{bash}
ade$ echo "export COLCON_DEFAULTS_FILE=/usr/local/etc/colcon-defaults.yaml" >> ~/.bashrc
```

### Outside ADE

Take the defaults file that comes with the Autoware.Core source checkout and add it to your shell startup; e.g.

```{bash}
echo "export COLCON_DEFAULTS_FILE=/path/to/AutowareAuto/tools/ade_image/colcon-defaults.yaml" >> .bashrc
```

or manually export the variable in the terminal in which compilation commands are issued; e.g.

```{bash}
export COLCON_DEFAULTS_FILE=/path/to/AutowareAuto/tools/ade_image/colcon-defaults.yaml
```

## How to build the code

To build all packages in Autoware.Core, navigate into the `AutowareCore` directory and run

```{bash}
ade$ colcon build
```

It's important that you always run `colcon build` from the repository root. If everything went well, you should _not_ see "failed" on your screen, although "packages had stderr output" is okay.

See [Compilation Optimization And Debugging Parameters](#compilation-optimization-and-debugging-parameters) for further details to influence how the code is built.

To verify that everything works as expected, see if all tests pass:

```{bash}
ade$ colcon test
ade$ colcon test-result --verbose
```

The first command will run the tests attached to the packages in your workspace.
The second command gives you detailed output from the tests on which ones passed and which failed.

### Advanced options

ROS 2 uses the `colcon` build system. For more information and details about options and flags, take a look at

```{bash}
colcon build --help
```

and see [the colcon documentation](https://colcon.readthedocs.io/en/released/user/quick-start.html). In the following, a few of the most useful options are listed.
Note that `colcon` options are spelled with an underscore instead of a dash – this is a common cause of typos.

#### Selecting packages to build

To just build a single package:

```{bash}
colcon build --packages-select <package_name>
```

Note that this does not automatically also build or rebuild its dependencies recursively. To do that:

```{bash}
colcon build --packages-up-to <package_name>
```

These options are also accepted by `colcon test`.

##### Compilation Optimization And Debugging Parameters

To add a compiler flag to all packages, e.g. for enabling the undefined behavior sanitizer:

```{bash}
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-fsanitize=undefined"
```

While building Autoware.Core, here are some common options for compilation build types:

- `Release`: Optimized and fast
- `Debug`: With debug flags but slow because not all compilation optimizations are applied
- `RelWithDebInfo`: Fast and allows debugging to a fair-enough degree

!!! note

    When using the [colcon defaults configuration file](#use-colcon-defaults) that ships with Autoware.Core, the default build type is `RelWithDebInfo`.

```{bash}
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

!!! warning

    If no optimization flags are set, then the Autoware.Core stack may be too slow to be useful. This can happen e.g. when the CMake build type is `Debug` or not set at all.

Edit the [colcon defaults configuration file](#building-colcon-defaults) to permanently pass extra `-cmake-args` to `colcon build` or select a different default build type. These settings can be overridden on demand by passing command-line arguments to `colcon`.

C++ targets in Autoware.Core are built with additional options set centrally by the directives in
[`autoware_auto_cmake.cmake`](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/tools/autoware_auto_cmake/cmake/autoware_auto_cmake.cmake)
and the `autoware_set_compile_options()` function defined in that file. The settings are imported to
a package through depending on the `autoware_auto_cmake` package in a `package.xml`

```{xml}
<buildtool_depend>autoware_auto_cmake</buildtool_depend>
```

and importing the build dependencies in `CMakeLists.txt`

```{cmake}
ament_auto_find_build_dependencies()
```

See [Seeing compiler commands](#seeing-compiler-commands) to check the compiler flags.

!!! note

    [A small number of targets](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/696) deactivate optimization flags regardless of the release type in order to reduce the build time. To override this, call `colcon build --cmake-args -DAUTOWARE_OPTIMIZATION_OF_SLOW_TARGETS=ON`.

##### Compilation Database Generation

In order to let IDEs analyze the build dependencies and symbol relationships, [a compilation database](https://colcon.readthedocs.io/en/released/user/how-to.html#cmake-packages-generating-compile-commands-json)
can be generated with the following flag:

```{bash}
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

#### Configuration variables

The Autoware build system defines global variables that can be set to change the build configuration.
For example, to enable downloading artifacts at build time:

```{bash}
colcon build --cmake-args -DDOWNLOAD_ARTIFACTS=ON
```

See the [CMake variables](autoware-auto-cmake-design.md#cmake-variables).

#### Cleaning the build output

`colcon` isn't very good at being stateless, so when you build, make changes, and build again, you can sometimes end up with a different result than when you build from scratch. To make sure you're getting a fresh build of a package, just do

```{bash}
rm -rf {build,install}/my_package
```

to remove all build artifacts associated with that package. Alternatively, if you don't want to delete the old binaries, you can specify custom build and install directories:

```{bash}
colcon build --build-base build_mybranch --install-base install_mybranch
```

#### Seeing compiler commands

To see the compiler and linker invocations for a package, use

```{bash}
VERBOSE=1 colcon build --packages-up-to <package_name> --event-handlers console_direct+
```

### Starting from a clean slate

Most issues with building Autoware.Core are caused by out-of-date software or old build files.
To update `ade` and the Docker containers it manages as well as clear old builds, run the following in your `adehome/AutowareCore` folder:

```{bash}
$ ade stop
$ sudo ade update-cli
$ ade start --update --enter
ade$ cd AutowareCore
ade$ rm -rf build/ install/ log/ src/external/
ade$ git pull
ade$ vcs import < autoware.core.$ROS_DISTRO.repos
```

If you are using Autoware.Core outside of `ade`, try updating your system and running the following in your `AutowareCore` folder and re-building (where `$ROS_DISTRO` is the current distro, e.g. `foxy`):

```{bash}
rm -rf build/ install/ log/ src/external/
git pull
source /opt/ros/$ROS_DISTRO/setup.bash
vcs import < autoware.core.$ROS_DISTRO.repos
```

If you are still having trouble after these commands have been run, please see the [Support Gudelines](support-guidelines.md) for where to ask questions.
