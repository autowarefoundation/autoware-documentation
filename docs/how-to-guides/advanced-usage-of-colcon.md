# Advanced usage of colcon

This page shows some advanced and useful usage of `colcon`.
If you need more detailed information, refer to the [colcon documentation](https://colcon.readthedocs.io/).

## Common mistakes

### Do not run from other than the workspace root

It is important that you always run `colcon build` from the workspace root because `colcon` builds only under the current directory.
If you have mistakenly built in a wrong directory, run `rm -rf build/ install/ log/` to clean the generated files.

### Do not unnecessarily overlay workspaces

`colcon` overlays workspaces if you have sourced the `setup.bash` of other workspaces before building a workspace.
You should take care of this especially when you have multiple workspaces.

Run `echo $COLCON_PREFIX_PATH` to check whether workspaces are overlaid.
If you find some workspaces are unnecessarily overlaid, remove all built files, restart the terminal to clean environment variables, and re-build the workspace.

For more details about `workspace overlaying`, refer to the [ROS2 documentation](https://docs.ros.org/en/rolling/Tutorials/Workspace/Creating-A-Workspace.html#source-the-overlay).

## Cleaning up the build artifacts

`colcon` sometimes causes errors of because of the old cache.
To remove the cache and rebuild the workspace, run the following command:

```bash
rm -rf build/ install/
```

In case you know what packages to remove:

```bash
rm -rf {build,install}/{package_a,package_b}
```

## Selecting packages to build

To just build specified packages:

```bash
colcon build --packages-select <package_name1> <package_name2> ...
```

To build specified packages and their dependencies recursively:

```bash
colcon build --packages-up-to <package_name1> <package_name2> ...
```

You can also use these options for `colcon test`.

## Changing the optimization level

Set `DCMAKE_BUILD_TYPE` to change the optimization level.

!!! warning

    If you specify `DCMAKE_BUILD_TYPE=Debug` or no `DCMAKE_BUILD_TYPE` is given for building the entire Autoware, it may be too slow to use.

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Changing the default configuration of colcon

Create `$COLCON_HOME/defaults.yaml` to change the default configuration.

```bash
mkdir -p ~/.colcon
cat << EOS > ~/.colcon/defaults.yaml
{
    "build": {
        "symlink-install": true
    }
}
```

See [here](https://colcon.readthedocs.io/en/released/user/configuration.html#defaults-yaml) for more details.

## Generating compile_commands.json

[compile_commands.json](https://colcon.readthedocs.io/en/released/user/how-to.html#cmake-packages-generating-compile-commands-json) is used by IDEs/tools to analyze the build dependencies and symbol relationships.

You can generate it with the flag `DCMAKE_EXPORT_COMPILE_COMMANDS=1`:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## Seeing compiler commands

To see the compiler and linker invocations for a package, use `VERBOSE=1` and `--event-handlers console_cohesion+`:

```bash
VERBOSE=1 colcon build --packages-up-to <package_name> --event-handlers console_cohesion+
```

See [here](https://colcon.readthedocs.io/en/released/reference/event-handler-arguments.html) for other options.

## Using Ccache

[Ccache](https://ccache.dev/) can speed up recompilation.
It is recommended to use it to save your time unless you have a specific reason not to do so.

1. Install `Ccache`:

   ```bash
   sudo apt update && sudo apt install ccache
   ```

2. Write the following in your `.bashrc`:

   ```bash
   export CC="/usr/lib/ccache/gcc"
   export CXX="/usr/lib/ccache/g++"
   ```
