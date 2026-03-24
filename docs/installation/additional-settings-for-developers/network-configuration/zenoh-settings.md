# Zenoh settings for ROS 2 and Autoware

Autoware uses CycloneDDS as its default communication middleware, but it's also compatible with other protocols like Zenoh. The ROS community [has selected](https://discourse.openrobotics.org/t/ros-2-alternative-middleware-report/33771) Zenoh as a promising new middleware alternative due to its key advantages:

- **Internet Communication**: Unlike DDS, which is limited to a local area network (LAN), Zenoh can seamlessly communicate with the cloud, eliminating the need for a separate bridge.
- **Namespace Support**: Zenoh allows for the use of namespaces for each vehicle. This feature simplifies managing multiple vehicles and isolating network traffic.
- **Non-Multicast Support**: Zenoh functions in non-multicast environments like 5G, a key limitation for DDS.
- **Reduced Discovery Overhead**: Zenoh significantly reduces discovery packet overhead, a known issue with DDS in wireless environments.
- **Superior Performance**: [A study](https://zenoh.io/blog/2023-03-21-zenoh-vs-mqtt-kafka-dds/) has shown that Zenoh generally outperforms other protocols such as DDS, MQTT, and Kafka.

The following sections provide a step-by-step tutorial for running Autoware with Zenoh. We recommend using **ROS 2 Jazzy**, supported since Autoware 1.7.1, as it includes a fix for the GuardCondition use-after-free issue and eliminates the need for any workaround patches.

## Install rmw_zenoh

1. Install rmw_zenoh

   ```bash
   sudo apt update && sudo apt install ros-jazzy-rmw-zenoh-cpp
   ```

2. Set rmw_zenoh as the default RMW implementation

   Add the following line to your `~/.bashrc` file:

   ```bash
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   ```

3. Reload your shell configuration (or open a new terminal):

   ```bash
   source ~/.bashrc
   ```

For more details, see the [rmw_zenoh repository](https://github.com/ros2/rmw_zenoh).

## Launch Autoware with Zenoh

1. Start the Zenoh router:

   ```bash
   # terminal 1
   ros2 run rmw_zenoh_cpp rmw_zenohd
   ```

2. Launch Autoware:

   ```bash
   # terminal 2
   source <YOUR-AUTOWARE-DIR>/install/setup.bash
   ros2 launch autoware_launch autoware.launch.xml ...
   ```

## Logging

Zenoh is implemented in Rust and uses a logging library configurable via the `RUST_LOG` environment variable.
You can specify different levels (such as `info`, `debug`, or `trace`) for more or less verbosity.

### Example

Start the Zenoh router with debug logs enabled:

```bash
export RUST_LOG=zenoh=debug
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Or launch autoware with info log in a single command:

```bash
RUST_LOG=zenoh=info ros2 launch autoware_launch autoware.launch.xml ...
```

For more information, see the [rmw_zenoh logging section](https://github.com/ros2/rmw_zenoh?tab=readme-ov-file#logging).
