# Advanced usage of colcon

It's important that you always run `colcon build` from the repository root. If everything went well, you should _not_ see "failed" on your screen, although "packages had stderr output" is okay.

Autoware uses the `colcon` build system.
In the followings, a few of the most useful options are listed.

For more information and details about options and flags, see the [colcon documentation](https://colcon.readthedocs.io/).

### Selecting packages to build

To just build specified packages:

```bash
colcon build --packages-select <package_name1> <package_name2> ...
```

To build specified packages and their dependencies recursively:

```bash
colcon build --packages-up-to <package_name1> <package_name2> ...
```

These options are also accepted by `colcon test`.

---

### Using ccache

Using `ccache`

1. Install `ccache`:

   ```bash
   sudo apt update && sudo apt install ccache
   ```

2. Write the following in your `.bashrc`:

   ```bash
   export CC="/usr/lib/ccache/gcc"
   export CXX="/usr/lib/ccache/g++"
   ```

### Changing optimization level

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

!!! warning

    If you specify `DCMAKE_BUILD_TYPE=Debug` or no `DCMAKE_BUILD_TYPE` is given for building the entire Autoware, it may be too slow to use.

### Changing the default configuration of colcon

https://colcon.readthedocs.io/en/released/user/configuration.html#defaults-yaml

!!! note

### Generating compile_commands.json for Clang-Tidy

##### Compilation Database Generation

In order to let IDEs analyze the build dependencies and symbol relationships, [a compilation database](https://colcon.readthedocs.io/en/released/user/how-to.html#cmake-packages-generating-compile-commands-json)
can be generated with the following flag:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

#### Cleaning the build output

`colcon` isn't very good at being stateless, so when you build, make changes, and build again, you can sometimes end up with a different result than when you build from scratch. To make sure you're getting a fresh build of a package, just do

```bash
rm -rf {build,install}/my_package
```

to remove all build artifacts associated with that package. Alternatively, if you don't want to delete the old binaries, you can specify custom build and install directories:

```bash
colcon build --build-base build_mybranch --install-base install_mybranch
```

#### Seeing compiler commands

To see the compiler and linker invocations for a package, use

```bash
VERBOSE=1 colcon build --packages-up-to <package_name> --event-handlers console_direct+
```
