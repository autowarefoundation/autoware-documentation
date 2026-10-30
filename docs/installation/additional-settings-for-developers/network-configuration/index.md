# Network settings for ROS 2 and Autoware

Autoware nodes communicate through a ROS 2 middleware (RMW, the ROS middleware interface). The Ansible playbook of the [source installation](../../autoware/source-installation.md) installs CycloneDDS (`rmw_cyclonedds_cpp`) and adds `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to `~/.bashrc`. The default kernel and middleware settings drop large messages such as point clouds and images, so you must apply the settings on these pages before you run Autoware.

The Autoware Docker images include a CycloneDDS configuration file for the `lo` interface. Their entrypoint also tries to enable multicast on `lo` and to apply the kernel settings of the DDS page. These two steps work only when the container runs with `--privileged` or `--cap-add=NET_ADMIN`.

## Single computer setup

When all Autoware nodes run on one computer, configure one of these middlewares:

- [DDS settings for ROS 2 and Autoware](./dds-settings.md): tune the kernel network buffers and configure CycloneDDS to use `lo`, the loopback interface. This is the default and most tested setup. It needs [multicast on `lo`](./enable-multicast-for-lo.md), because DDS discovery uses multicast and Linux disables it on `lo` by default.
- [Zenoh settings for ROS 2 and Autoware](./zenoh-settings.md): use `rmw_zenoh_cpp` and a Zenoh router instead of DDS.

## Multi computer setup

Do these steps when Autoware runs on more than one computer, or when a simulator runs on another computer:

1. Apply the [DDS settings](./dds-settings.md) on every computer.
2. Follow [Communicating across multiple computers with CycloneDDS](./multiple-computers.md) to select the network interface and to synchronize the clocks.
