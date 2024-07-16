# Communicating across multiple computers within a network with CycloneDDS

## Configuring CycloneDDS

Within the `~/cyclonedds.xml` file, Interfaces section can be set in various ways to communicate across multiple computers within a network.

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

    You should replace `enp38s0` with the actual network interface name.

!!! note

    `ifconfig` command can be used to find the network interface name.

## Time synchronization

To ensure that the nodes on different computers are synchronized, you should synchronize the time between the computers.

You can use the `chrony` to synchronize the time between computers.

Please refer to this post for more information: [Multi PC AWSIM + Autoware Tests #3813](https://github.com/orgs/autowarefoundation/discussions/3813)

!!! warning

    Under Construction
