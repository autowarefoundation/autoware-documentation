# Console settings for ROS 2

## Colorizing logger output

By default, the ROS 2 logger colorizes the output only when it writes directly to a terminal.
The nodes that `ros2 launch` starts do not write to a terminal, so their output has no color.
To always colorize it, add the following to your `~/.bashrc`:

```bash
export RCUTILS_COLORIZED_OUTPUT=1
```

## Customizing the format of logger output

By default, ROS 2 logger doesn't output detailed information such as file name, function name, or line number.
To customize it, add the following to your `~/.bashrc`:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
```

For more options, see [Console output formatting](https://docs.ros.org/en/jazzy/Tutorials/Demos/Logging-and-logger-configuration.html#console-output-formatting) in the ROS 2 documentation.

## Colorized GoogleTest output

Add `export GTEST_COLOR=1` to your `~/.bashrc`.

For more details, refer to [Advanced GoogleTest Topics: Colored Terminal Output](https://google.github.io/googletest/advanced.html#colored-terminal-output).

This is useful when running tests with `colcon test`.
