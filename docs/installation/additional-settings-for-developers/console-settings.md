# Console settings for ROS 2

## Colorizing logger output

By default, ROS 2 logger doesn't colorize the output.
To colorize it, write the following in your `.bashrc`:

```bash
export RCUTILS_COLORIZED_OUTPUT=1
```

## Customizing the format of logger output

By default, ROS 2 logger doesn't output detailed information such as file name, function name, or line number.
To customize it, write the following in your `.bashrc`:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
```

For more options, see [here](https://docs.ros.org/en/rolling/Tutorials/Logging-and-logger-configuration.html#console-output-formatting).
