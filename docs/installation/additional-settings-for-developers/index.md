# Additional settings for developers

The pages in this section change the configuration of your computer and your shell, not of Autoware itself. Apply them after you install Autoware.

## Console settings

By default, ROS 2 prints log messages without the source location, and without color when `ros2 launch` starts the nodes. This page sets environment variables in `~/.bashrc` that color the logger output, add the function, file, and line number to each message, and color the GoogleTest output of `colcon test`. These settings are optional. See [Console settings for ROS 2](console-settings.md).

## Network configuration

Autoware sends large messages, such as point clouds and images, between its nodes. Without network tuning, these messages do not arrive. The [network configuration pages](network-configuration/index.md) explain how to:

- Tune the Linux kernel buffers and configure CycloneDDS, the default middleware of Autoware.
- Enable multicast on the loopback interface for communication on one computer.
- Use Zenoh as an alternative middleware.
- Connect several computers and synchronize their clocks.

The [source installation](../autoware/source-installation.md) requires the network configuration before you run Autoware.
