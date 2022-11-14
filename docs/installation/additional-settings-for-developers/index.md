# Additional settings for developers

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

For more options, see [here](https://docs.ros.org/en/rolling/Tutorials/Logging-and-logger-configuration.html#console-output-formatting).

### Enabling localhost-only communication

By default, ROS 2 communicates using multi-cast, which may unnecessarily increase the network traffic.
To avoid it, write the following in your `.bashrc`:

```bash
export ROS_LOCALHOST_ONLY=1
```

### Setting up `ROS_DOMAIN_ID`

ROS 2 uses `ROS_DOMAIN_ID` to create groups and communicate between machines in the groups.
Since all ROS 2 nodes use domain ID `0` by default, it may cause unintended interference.

To avoid it, set a different domain ID for each group in your `.bashrc`:

```bash
# Replace X with the Domain ID you want to use
# Domain ID should be a number in range [0, 101] (inclusive)
export ROS_DOMAIN_ID=X
```

For more information, see [here](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html#the-ros-domain-id).

### Tuning DDS

Autoware uses DDS for inter-node communication. [ROS 2 documentation](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html) recommends users to tune DDS to utilize its capability. Especially, receive buffer size is the critical parameter for Autoware. If the parameter is not large enough, Autoware will failed in receiving large data like point cloud or image.

For example, to execute Autoware with CycloneDDS, prepare config file. A sample config file, named as `cyclonedds_config.xml`, is given below.

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
