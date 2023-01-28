# Additional settings for developers

## Console settings for ROS 2

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

For more options, see [here](https://docs.ros.org/en/rolling/Tutorials/Logging-and-logger-configuration.html#console-output-formatting).

## Network settings for ROS 2

This section describes the network settings.
ROS 2 employs DDS, and the configuration of ROS2 and DDS will be described separately.
Please refer to [the official documentation](http://design.ros2.org/articles/ros_on_dds.html) for ROS2 networking concepts.

### ROS 2 network setting

ROS2 multicasts data on the local network by default. When developing in a company, etc., data flows over the local network and there is a possibility of collision.

- Localhost-only communication
- Same domain only communication on the local network

Unless you plan to use multiple host computers on the local network, localhost-only communication is recommended.

### Enabling localhost-only communication

By default, ROS 2 communicates using multi-cast, which may unnecessarily increase the network traffic.
To avoid it, write the following in your `.bashrc`:
For more information, see [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#the-ros-localhost-only-variable).

```bash
export ROS_LOCALHOST_ONLY=1
```

If exported `ROS_LOCALHOST_ONLY=1`, `MULTICAST` must be enabled at the loopback address. Use the following command to verify that `MULTICAST` is included.

```bash
ip link show lo
# Result sample
# 1: lo: <LOOPBACK,MULTICAST,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000
```

If `MULTICAST` is not included, use the following command to enable it.

```bash
sudo ip link set lo multicast on
```

### Same domain only communication on the local network

ROS 2 uses `ROS_DOMAIN_ID` to create groups and communicate between machines in the groups.
Since all ROS 2 nodes use domain ID `0` by default, it may cause unintended interference.

To avoid it, set a different domain ID for each group in your `.bashrc`:

```bash
# Replace X with the Domain ID you want to use
# Domain ID should be a number in range [0, 101] (inclusive)
export ROS_DOMAIN_ID=X
```

Also confirm that `ROS_LOCALHOST_ONLY` is `0` by using the following command.

```bash
echo $ROS_LOCALHOST_ONLY # If output is 1, localhost has priority.
```

For more information, see [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#the-ros-domain-id-variable).

## DDS settings

Autoware uses DDS for inter-node communication. [ROS 2 documentation](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html) recommends users to tune DDS to utilize its capability. Especially, receive buffer size is the critical parameter for Autoware. If the parameter is not large enough, Autoware will failed in receiving large data like point cloud or image.

### Tuning DDS

Unless customized, CycloneDDS is adopted by default. For example, to execute Autoware with CycloneDDS, prepare config file. A sample config file, named as `cyclonedds_config.xml`, is given below.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config
https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <Internal>
            <SocketReceiveBufferSize min="10MB"/>
        </Internal>
    </Domain>
</CycloneDDS>
```

Set the config file path and enlarge the Linux kernel maximum buffer size before launching Autoware.

```bash
export CYCLONEDDS_URI=file:///absolute/path/to/cyclonedds_config.xml
sudo sysctl -w net.core.rmem_max=2147483647
```

Refer to [ROS 2 documentation](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html) for more information. Reading user guide for chosen DDS is helpful for more understanding.

### Tuning DDS for multiple host computers (for advanced users)

When Autoware runs on multiple host computers, IP Fragmentation should be taken into account. As [ROS 2 documentation](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html#cross-vendor-tuning) recommends, parameters for IP Fragmentation should be set as shown in the following example.

```bash
sudo sysctl net.ipv4.ipfrag_time=3
sudo sysctl net.ipv4.ipfrag_high_thresh=134217728     # (128 MB)
```
