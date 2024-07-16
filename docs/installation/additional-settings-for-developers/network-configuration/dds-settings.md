# DDS settings for ROS 2 and Autoware

## Enable localhost-only communication

1. [Enable `multicast` for `lo`](./enable-multicast-for-lo.md)
2. Make sure `export ROS_LOCALHOST_ONLY=1` **is NOT** present in `.bashrc`.

- See [About `ROS_LOCALHOST_ONLY` environment variable](#about-ros_localhost_only-environment-variable) for more information.

## Tune DDS settings

Autoware uses DDS for internode communication. [ROS 2 documentation](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html) recommends users to tune DDS to utilize its capability.

!!! note

    CycloneDDS is the recommended and most tested DDS implementation for Autoware.

!!! warning

    If you don't tune these settings, Autoware will fail to receive large data like point clouds or images.

### Tune system-wide network settings

Set the config file path and enlarge the Linux kernel maximum buffer size before launching Autoware.

```bash
# Increase the maximum receive buffer size for network packets
sudo sysctl -w net.core.rmem_max=2147483647  # 2 GiB, default is 208 KiB

# IP fragmentation settings
sudo sysctl -w net.ipv4.ipfrag_time=3  # in seconds, default is 30 s
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB, default is 256 KiB
```

To make it permanent,

```bash
sudo nano /etc/sysctl.d/10-cyclone-max.conf
```

Paste the following into the file:

```bash
# Increase the maximum receive buffer size for network packets
net.core.rmem_max=2147483647  # 2 GiB, default is 208 KiB

# IP fragmentation settings
net.ipv4.ipfrag_time=3  # in seconds, default is 30 s
net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB, default is 256 KiB
```

Details of each parameter here is explained in the [ROS 2 documentation](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html#cross-vendor-tuning).

#### Validate the sysctl settings

```console
user@pc$ sysctl net.core.rmem_max net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh
net.core.rmem_max = 2147483647
net.ipv4.ipfrag_time = 3
net.ipv4.ipfrag_high_thresh = 134217728
```

### CycloneDDS Configuration

Save the following file as `~/cyclonedds.xml`.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="false" name="lo" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>default</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
```

Then add the following line to your `~/.bashrc` file.

```bash
export CYCLONEDDS_URI=file:///absolute/path/to/cyclonedds.xml
# Replace `/absolute/path/to/cyclonedds.xml` with the actual path to the file.
# Example: export CYCLONEDDS_URI=file:///home/user/cyclonedds.xml
```

This configuration is mostly taken from [Eclipse Cyclone DDS:Run-time configuration documentation](https://github.com/eclipse-cyclonedds/cyclonedds/tree/a10ced3c81cc009e7176912190f710331a4d6caf#run-time-configuration).
You can see why each value is set as such under the documentation link.

## Additional information

### About `ROS_LOCALHOST_ONLY` environment variable

Previously, we used to set `export ROS_LOCALHOST_ONLY=1` to enable localhost-only communication.
But because of [an ongoing issue](https://github.com/ros2/rmw_cyclonedds/issues/370), this method doesn't work.

!!! warning

    Do not set `export ROS_LOCALHOST_ONLY=1` in `~/.bashrc`.
    If you do so, it will cause an error with RMW.
    Remove it from `~/.bashrc` if you have set it.

### About `ROS_DOMAIN_ID` environment variable

We can also set `export ROS_DOMAIN_ID=3(or any number 1 to 255)` (`0` by default) to avoid interference with other ROS 2 nodes on the same network.

But since `255` is a very small number, it might interfere with other computers on the same network unless you make sure everyone has a unique domain ID.

Another problem is that if someone runs a test that uses ROS 2 [launch_testing](https://github.com/ros2/launch/blob/a317c54bbbf2dfeec35fbb6d2b5913939d02750d/launch_testing/README.md) framework,
by default it will use a random domain ID to isolate between tests even on the same machine.
See [this PR](https://github.com/ros2/launch/pull/251) for more details.
