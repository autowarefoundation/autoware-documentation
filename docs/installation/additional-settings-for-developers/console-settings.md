# Console settings for ROS 2

## Colorizing logger output

By default, ROS 2 logger doesn't colorize the output.
To colorize it, add the following to your `~/.bashrc`:

```bash
export RCUTILS_COLORIZED_OUTPUT=1
```

## Customizing the format of logger output

By default, ROS 2 logger doesn't output detailed information such as file name, function name, or line number.
To customize it, add the following to your `~/.bashrc`:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
```

For more options, see [here](https://docs.ros.org/en/rolling/Tutorials/Demos/Logging-and-logger-configuration.html#console-output-formatting).

## Colorized GoogleTest output

Add `export GTEST_COLOR=1` to your `~/.bashrc`.

For more details, refer to [Advanced GoogleTest Topics: Colored Terminal Output](https://google.github.io/googletest/advanced.html#colored-terminal-output).

This is useful when running tests with `colcon test`.
