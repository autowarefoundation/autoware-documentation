# Installation of tools for developers

## ROS 2 settings

### Colorizing logger output

By default, ROS 2 logger doesn't colorize the output.
To colorize it, write the following in your `.bashrc`:

```bash
export RCUTILS_COLORIZED_OUTPUT=1
```

### Customizing the format of logger output

By default, ROS 2 logger doesn't output detailed information such as file name, function name, or line number.
To customize it, write the following in your `.bashrc`:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
```

See [here](https://docs.ros.org/en/rolling/Tutorials/Logging-and-logger-configuration.html#console-output-formatting) for more options.

### Enabling localhost-only communication

By default, ROS 2 communicates using multi-cast, which may unnecessarily increase the network traffic.
To avoid it, write the following in your `.bashrc`:

```bash
export ROS_LOCALHOST_ONLY=1
```

### Setting up `ROS_DOMAIN_ID`

ROS 2 nodes on the same domain can freely discover and send messages to each
other, while ROS 2 nodes on different domains cannot. All ROS 2 nodes use
domain ID 0 by default. To avoid interference between different groups of
computers running ROS 2 on the same network, a different domain ID should
be set for each group.

See [here](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html#the-ros-domain-id)
for more information.

To set it, write the following in your `.bashrc`:

```bash
# Replace X with the Domain ID you want to use
# Domain ID should be a number in range [0, 101] (inclusive)
export ROS_DOMAIN_ID=X
```
