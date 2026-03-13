# Runtime Troubleshooting

This page describes runtime errors that are not covered by [Performance Troubleshooting](performance-troubleshooting.md) and how to resolve them.

## CycloneDDS: Failed to find a free participant index

### Symptoms

When running Autoware on **ROS 2 Jazzy** (Ubuntu 24.04) with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, some nodes fail to start with errors such as:

```text
[component_container_mt-16] Failed to find a free participant index for domain 0
[component_container_mt-16] [ERROR] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Error (check_create_domain() at ./src/rmw_node.cpp:1242)
[component_container_mt-16] terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
[component_container_mt-16]   what():  failed to initialize rcl node: error not set, at ./src/rcl/node.c:252
```

This often occurs when launching demos that use many nodes (e.g. [Planning Simulator](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/)).

### Cause

CycloneDDS itself defaults to `ParticipantIndex=none`, which does not limit the number of participants. However, on ROS 2 Jazzy, **rmw_cyclonedds_cpp** builds the domain configuration and explicitly sets `ParticipantIndex=auto` and `MaxAutoParticipantIndex=32`, overriding the CycloneDDS default. As a result, only about 32 DDS participants can exist per host. Autoware can use more DDS participants than this limit (e.g. 100 or more), so new nodes cannot obtain a free participant index and fail to create the domain.

This behavior comes from the RMW implementation; see [rmw_cyclonedds: check_create_domain()](https://github.com/ros2/rmw_cyclonedds/blob/7cd457de5825d4cb46ec7b081aa00a5392e388d0/rmw_cyclonedds_cpp/src/rmw_node.cpp#L1193-L1199) (lines 1193–1199).

### Solution

Configure CycloneDDS by adding a `<Discovery>` section to your `cyclonedds.xml` with `ParticipantIndex` set to `none`:

```xml
<Discovery>
  <ParticipantIndex>none</ParticipantIndex>
</Discovery>
```

A full example is given in [CycloneDDS Configuration](../../../installation/additional-settings-for-developers/network-configuration/dds-settings.md#cyclonedds-configuration).

### References

- [autowarefoundation/autoware#6759](https://github.com/autowarefoundation/autoware/issues/6759) — Issue and discussion for this error
- [rmw_cyclonedds: check_create_domain() (ParticipantIndex=auto, MaxAutoParticipantIndex=32)](https://github.com/ros2/rmw_cyclonedds/blob/7cd457de5825d4cb46ec7b081aa00a5392e388d0/rmw_cyclonedds_cpp/src/rmw_node.cpp#L1193-L1199) — Where the RMW overrides CycloneDDS defaults
- [Eclipse Cyclone DDS — Controlling port numbers](https://cyclonedds.io/docs/cyclonedds/0.9.1/config.html#controlling-port-numbers)
- [CycloneDDS config reference — MaxAutoParticipantIndex](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html#cyclonedds-domain-discovery-maxautoparticipantindex)
