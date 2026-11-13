<!-- cspell: ignore iburst minpoll maxpoll maxdelay -->

# Communicating across multiple computers with CycloneDDS

Some setups run Autoware on more than one computer. For example, the Autoware nodes run on one computer and a simulator such as AWSIM runs on a second computer. The ROS 2 nodes on all computers must find each other over the network, and the clocks of the computers must agree.

This page assumes that every computer uses CycloneDDS (`rmw_cyclonedds_cpp`), the default middleware of Autoware.

## Before you start

Do these steps on every computer:

1. Apply the [DDS settings](dds-settings.md): the kernel buffer sizes, the `~/cyclonedds.xml` file, and the `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI` environment variables.
2. Use the same `ROS_DOMAIN_ID` on every computer. If you do not set it, ROS 2 uses `0`. For details, see [About `ROS_DOMAIN_ID` environment variable](dds-settings.md#about-ros_domain_id-environment-variable).
3. Connect the computers to the same network and give them addresses in the same subnet. Point clouds and images need a high bandwidth. For example, the [multi PC AWSIM test](https://github.com/orgs/autowarefoundation/discussions/3813) connected the two computers directly with a 10 Gigabit Ethernet link.
4. If a firewall runs on a computer, allow UDP traffic from the other computers. DDS uses UDP for discovery and for data.

## Configuring CycloneDDS

The `~/cyclonedds.xml` file from the DDS settings uses only `lo`, the loopback interface. Nodes on other computers cannot reach it. To communicate with the other computers, change the `Interfaces` section of `~/cyclonedds.xml` in one of the two ways below.

### Automatically determine the network interface (convenient)

With this setting, CycloneDDS will automatically determine the most suitable network interface to use.

```xml
<Interfaces>
  <NetworkInterface autodetermine="true" priority="default" multicast="default" />
</Interfaces>
```

### Manually set the network interface (recommended)

With this setting, you can manually set the network interface to use.

```xml
<Interfaces>
  <NetworkInterface autodetermine="false" name="enp38s0" priority="default" multicast="default" />
</Interfaces>
```

!!! warning

    Replace `enp38s0` with the actual network interface name.

!!! note

    `ip -brief address` lists the network interfaces and their addresses. The `ifconfig` command of the `net-tools` package shows the same information.

The full file with a manually set interface looks like this. Only the `NetworkInterface` line differs from the file on the [DDS settings](dds-settings.md#cyclonedds-configuration) page:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="false" name="enp38s0" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>default</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>none</ParticipantIndex>
    </Discovery>
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
```

CycloneDDS reads the file only when a node starts. After you change the file, restart the nodes. Also run `ros2 daemon stop`, so that the `ros2` command line tools use the new file.

### Using the Autoware Docker images

The Autoware Docker images contain a `/home/aw/cyclonedds.xml` file that uses `lo` only. To reach other computers from a container, start the container with `--net host` and mount your own file over it:

```bash
docker run --rm -it \
  --net host \
  -v /path/to/your/cyclonedds.xml:/home/aw/cyclonedds.xml \
  ghcr.io/autowarefoundation/autoware:universe-cuda-jazzy
```

For the other `docker run` options, see [`docker/README.md` → Usage](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md#usage).

## Make sure that the computers find each other

1. Make sure that UDP multicast works between the computers. On the first computer, run:

   ```bash
   ros2 multicast receive
   ```

   On the second computer, run:

   ```bash
   ros2 multicast send
   ```

   The first computer prints a line such as `Received from 192.168.10.2:43210: 'Hello World!'`. The operating system selects the interface for this test, not `~/cyclonedds.xml`.

2. Make sure that ROS 2 messages pass through CycloneDDS. Run this command on every computer at the same time:

   ```bash
   ros2 doctor hello
   ```

   Each computer prints a `MULTIMACHINE COMMUNICATION SUMMARY` table every second. The `Subscribed from:` list must contain the host names of all the other computers. Press `Ctrl+C` to stop the command.

If the computers do not find each other, make sure that:

- The `~/cyclonedds.xml` file of each computer names the network interface of that computer. The interface names can be different on each computer.
- The `ROS_DOMAIN_ID` and the `RMW_IMPLEMENTATION` are the same on all computers.
- The firewall allows UDP.

## Time synchronization

Autoware nodes compare the time stamps of the messages that they receive. When the nodes run on different computers, the clocks of the computers must agree.

This section uses `chrony` to synchronize the clocks. In this setup, one computer is the time server and the other computers follow it. The example uses `192.168.10.1` for the server and `192.168.10.2` for a client. Replace them with your addresses.

1. Install `chrony` on every computer.

   ```bash
   sudo apt install chrony
   ```

2. On the server, add this line to `/etc/chrony/chrony.conf` to accept requests from the client. Add one `allow` line for each client.

   ```text
   allow 192.168.10.2
   ```

   If the server has no internet connection, also add `local stratum 10`. Then the server serves its own clock to the clients.

3. On each client, comment out the `pool` lines in `/etc/chrony/chrony.conf` and add these lines:

   ```text
   server 192.168.10.1 iburst minpoll 0 maxpoll 5 maxdelay .05
   makestep 0 1
   ```

   The `minpoll 0 maxpoll 5` options make the client ask the server every 1 to 32 seconds. The `makestep 0 1` line lets `chrony` correct a large offset in one step at the first update.

4. Restart `chrony` on every computer.

   ```bash
   sudo systemctl restart chrony
   ```

5. On each client, make sure that `chrony` follows the server.

   ```bash
   chronyc sources
   chronyc tracking
   ```

   In the output of `chronyc sources`, the line of the server starts with `^*`. In the output of `chronyc tracking`, the `System time` line shows the offset from the server.

In the [multi PC AWSIM test](https://github.com/orgs/autowarefoundation/discussions/3813), this setup reached a time difference of about 50 µs over a direct 10 Gigabit Ethernet link. The full `chrony.conf` files of that test are in the discussion. For all options, see the [chrony documentation](https://chrony-project.org/documentation.html).
