# Network settings for ROS 2

ROS 2 employs DDS, and the configuration of ROS 2 and DDS is described separately.

For ROS 2 networking concepts, refer to the [official documentation](http://design.ros2.org/articles/ros_on_dds.html).

ROS 2 multicasts data on the local network by default.

Therefore, when you develop in an office, the data flows over the local network of your office.

It may cause collisions of packets or increases in network traffic.

To avoid these, you should limit the DDS communication to the loopback interface within the host computer.

Unless you plan to use multiple host computers on the local network, localhost-only communication is recommended.
For details, refer to the sections below.

## Enabling localhost-only communication

### Bad methods ❌

#### `ROS_LOCALHOST_ONLY` parameter

Previously, we used to set `export ROS_LOCALHOST_ONLY=1` to enable localhost-only communication.
But because of [an ongoing issue](https://github.com/ros2/rmw_cyclonedds/issues/370), this method doesn't work.

#### `ROS_DOMAIN_ID` parameter

We can also set `export ROS_DOMAIN_ID=3(or any number 1 to 255)` (`0` by default) to avoid interference with other ROS 2 nodes on the same network.

But since `255` is a very small number, there is no guarantee that it will not interfere with other computers on the same network unless you check everyones' domain ID.

Another problem is that if someone runs a test that uses ROS 2 [launch_testing](https://github.com/ros2/launch/blob/a317c54bbbf2dfeec35fbb6d2b5913939d02750d/launch_testing/README.md) framework,
by default it will use a random domain ID to isolate between tests even on the same machine.
See [this PR](https://github.com/ros2/launch/pull/251) for more details.

### Good method ✅

!!! warning

    Do not set `export ROS_LOCALHOST_ONLY=1` in `.bashrc` because it will cause an error.
    Remove it from `.bashrc` if you have set it.

If you export `ROS_LOCALHOST_ONLY=1`, `MULTICAST` must be enabled at the loopback address.
To verify that `MULTICAST` is enabled, use the following command.

```console
$ ip link show lo
1: lo: <LOOPBACK,MULTICAST,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000
```

If the word `MULTICAST` is not printed, use the following command to enable it.

```bash
sudo ip link set lo multicast on
```

## Enable `multicast` on `lo` on startup with a service

```bash
sudo nano /etc/systemd/system/multicast-lo.service
```

Paste the following into the file:

```service
[Unit]
Description=Enable Multicast on Loopback

[Service]
Type=oneshot
ExecStart=/usr/sbin/ip link set lo multicast on

[Install]
WantedBy=multi-user.target
```

Press following in order to save with nano:

1. `Ctrl+X`
2. `Y`
3. `Enter`

```bash
# Make it recognized
sudo systemctl daemon-reload

# Make it run on startup
sudo systemctl enable multicast-lo.service

# Start it now
sudo systemctl start multicast-lo.service
```

### Validate

```console
mfc@mfc-leo:~$ sudo systemctl status multicast-lo.service
○ multicast-lo.service - Enable Multicast on Loopback
     Loaded: loaded (/etc/systemd/system/multicast-lo.service; enabled; vendor preset: enabled)
     Active: inactive (dead) since Mon 2024-07-08 12:54:17 +03; 4s ago
    Process: 22588 ExecStart=/usr/bin/ip link set lo multicast on (code=exited, status=0/SUCCESS)
   Main PID: 22588 (code=exited, status=0/SUCCESS)
        CPU: 1ms

Tem 08 12:54:17 mfc-leo systemd[1]: Starting Enable Multicast on Loopback...
Tem 08 12:54:17 mfc-leo systemd[1]: multicast-lo.service: Deactivated successfully.
Tem 08 12:54:17 mfc-leo systemd[1]: Finished Enable Multicast on Loopback.
```

```console
mfc@mfc-leo:~$ ip link show lo
1: lo: <LOOPBACK,MULTICAST,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
```

## DDS settings

Autoware uses DDS for internode communication. [ROS 2 documentation](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html) recommends users to tune DDS to utilize its capability. Especially, receive buffer size is the critical parameter for Autoware. If the parameter is not large enough, Autoware will fail receiving large data like point clouds or images.

When using Autoware, we recommend using CycloneDDS as it is the most tested DDS implementation for Autoware.

We need to configure it with a config file is given below. Save it as `~/cyclonedds.xml`.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="lo" priority="default" multicast="default" />
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

This configuration is mostly taken from [Eclipse Cyclone DDS:Run-time configuration documentation](https://github.com/eclipse-cyclonedds/cyclonedds/tree/a10ced3c81cc009e7176912190f710331a4d6caf#run-time-configuration).
You can see why each value is set as such under the documentation link.

Set the config file path and enlarge the Linux kernel maximum buffer size before launching Autoware.

```bash
export CYCLONEDDS_URI=file:///absolute/path/to/cyclonedds_config.xml
sudo sysctl -w net.core.rmem_max=2147483647
```

For more information, Refer to [ROS 2 documentation](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html). Reading user guide for chosen DDS is helpful for more understanding.

### Tuning DDS for multiple host computers (for advanced users)

```xml
<Interfaces>
  <NetworkInterface autodetermine="true" priority="default" multicast="default" />
</Interfaces>
```

When Autoware runs on multiple host computers, IP Fragmentation should be taken into account. As [ROS 2 documentation](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html#cross-vendor-tuning) recommends, parameters for IP Fragmentation should be set as shown in the following example.

```bash
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728     # (128 MB)
```
