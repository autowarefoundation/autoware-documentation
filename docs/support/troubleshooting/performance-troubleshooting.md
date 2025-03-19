# Performance Troubleshooting

Overall symptoms:

- Autoware is running slower than expected
- Messages show up late in RViz2
- Point clouds are lagging
- Camera images are lagging behind
- Point clouds or markers flicker on RViz2
- When multiple subscribers use the same publishers, the message rate drops

## Diagnostic Steps

### Check if multicast is enabled

#### Target symptoms

- When multiple subscribers use the same publishers, the message rate drops

#### Diagnosis

Make sure that the multicast is enabled for your interface.

For example when you run following:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

If you get the error message `selected interface "{your-interface-name}" is not multicast-capable: disabling multicast`, this should be fixed.

#### Solution

Follow [DDS settings for ROS 2 and Autoware](../../installation/additional-settings-for-developers/network-configuration/dds-settings.md)

Especially the [Enable `multicast` on `lo`](../../installation/additional-settings-for-developers/network-configuration/enable-multicast-for-lo.md) section.

### Check the compilation flags

#### Target symptoms

- Autoware is running slower than expected
- Point clouds are lagging
- When multiple subscribers use the same publishers, the message rate drops even further

#### Diagnosis

Check the `~/.bash_history` file to see if there are any `colcon build` directives without `-DCMAKE_BUILD_TYPE=Release` or `-DCMAKE_BUILD_TYPE=RelWithDebInfo` flags at all.

Even if a build starts with these flags but same workspace gets compiled without these flags, it will still be a slow build in the end.

In addition, the nodes will run slow in general, especially the `pointcloud_preprocessor` nodes.

Example issue: [issue2597](https://github.com/autowarefoundation/autoware_universe/issues/2597#issuecomment-1491789081)

#### Solution

- Remove the `build`, `install` and optionally `log` folders in the main `autoware` folder.
- Compile the Autoware with either `Release` or `RelWithDebInfo` tags:

  ```bash
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  # Or build with debug flags too (comparable performance but you can debug too)
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
  ```

### Check the DDS settings

#### Target symptoms

- Autoware is running slower than expected
- Messages show up late in RViz2
- Point clouds are lagging
- Camera images are lagging behind
- When multiple subscribers use the same publishers, the message rate drops

#### Check the RMW (ROS Middleware) implementation

##### Diagnosis

Run following to check the middleware used:

```bash
echo $RMW_IMPLEMENTATION
```

The return line should be `rmw_cyclonedds_cpp`. If not, apply the solution.

If you are using a different DDS middleware, we might not have official support for it just yet.

##### Solution

Add `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` as a separate line in you `~/.bashrc` file.

More details in: [CycloneDDS Configuration](../../installation/additional-settings-for-developers/network-configuration/dds-settings.md#cyclonedds-configuration)

#### Check if the CycloneDDS is configured correctly

##### Diagnosis

Run following to check the configuration `.xml` file of the `CycloneDDS`:

```bash
echo $CYCLONEDDS_URI
```

The return line should be a valid path pointing to an `.xml` file with `CycloneDDS` configuration.

Also check if the file is configured correctly:

```bash
cat ${CYCLONEDDS_URI#file://}
```

This should print the `.xml` file on the terminal.

##### Solution

Follow [CycloneDDS Configuration](../../installation/additional-settings-for-developers/network-configuration/dds-settings.md#cyclonedds-configuration) and make sure:

- you have `export CYCLONEDDS_URI=file:///absolute_path_to_your/cyclonedds.xml` as a line on your `~/.bashrc` file.
- you have the `cyclonedds.xml` with the configuration provided in the documentation.

#### Check the Linux kernel maximum buffer size

##### Diagnosis

[Validate the sysctl settings](../../installation/additional-settings-for-developers/network-configuration/dds-settings.md#validate-the-sysctl-settings)

##### Solution

[Tune system-wide network settings](../../installation/additional-settings-for-developers/network-configuration/dds-settings.md#tune-system-wide-network-settings)

### Check if localhost only communication for DDS is enabled

- If you are using multi computer setup, please skip this check.
- Enabling localhost only communication for DDS can help improve the performance of ROS by reducing network traffic and avoiding potential conflicts with other devices on the network.

#### Target symptoms

- You see topics that shouldn't exist
- You see point clouds that don't belong to your machine
  - They might be from another computer running ROS 2 on your network
- Point clouds or markers flicker on RViz2
  - Another publisher (on another machine) may be publishing on the same topic as your node does.
  - Causing the flickering.

#### Diagnosis

Run:

```bash
cat ${CYCLONEDDS_URI#file://}
```

And it should return [DDS settings for ROS 2 and Autoware: CycloneDDS Configuration](../../installation/additional-settings-for-developers/network-configuration/dds-settings.md#cyclonedds-configuration) this file.

#### Solution

Follow [DDS settings for ROS 2 and Autoware: Enable localhost-only communication](../../installation/additional-settings-for-developers/network-configuration/dds-settings.md#enable-localhost-only-communication).

Also make sure the following returns an empty line:

```bash
echo $ROS_LOCALHOST_ONLY
```
