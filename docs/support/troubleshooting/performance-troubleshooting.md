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

Run the following command to allow multicast:

```bash
sudo ip link set multicast on {your-interface-name}
```

This way DDS will function as intended and multiple subscribers can receive data from a single publisher without any significant degradation in performance.

This is a temporary solution. And will be reverted once the computer restarts.

To make it permanent either,

- Create a service to run this on startup (recommended)
- **OR** put following lines to the `~/.bashrc` file:

  ```bash
  if [ ! -e /tmp/multicast_is_set ]; then
  sudo ip link set lo multicast on
  touch /tmp/multicast_is_set
  fi
  ```

  - This will probably ask for password on the terminal every time you restart the computer.

### Check the compilation flags

#### Target symptoms

- Autoware is running slower than expected
- Point clouds are lagging
- When multiple subscribers use the same publishers, the message rate drops even further

#### Diagnosis

Check the `~/.bash_history` file to see if there are any `colcon build` directives without `-DCMAKE_BUILD_TYPE=Release` or `-DCMAKE_BUILD_TYPE=RelWithDebInfo` flags at all.

Even if a build starts with these flags but same workspace gets compiled without these flags, it will still be a slow build in the end.

In addition, the nodes will run slow in general, especially the `pointcloud_preprocessor` nodes.

Example issue: [issue2597](https://github.com/autowarefoundation/autoware.universe/issues/2597#issuecomment-1491789081)

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

#### Check if the CycloneDDS is configured correctly

##### Diagnosis

Run following to check the configuration `.xml` file of the `CycloneDDS`:

```bash
echo $CYCLONEDDS_URI
```

The return line should be a valid path pointing to an `.xml` file with `CycloneDDS` configuration.

Also check if the file is configured correctly:

```bash
cat !{echo $CYCLONEDDS_URI}
```

This should print the `.xml` file on the terminal.

##### Solution

Follow [DDS settings:Tuning DDS documentation](../../installation/additional-settings-for-developers/index.md#tuning-dds) and make sure:
- you have `export CYCLONEDDS_URI=/absolute_path_to_your/cyclonedds_config.xml` as a line on your `~/.bashrc` file.
- you have the `cyclonedds_config.xml` with the configuration provided in the documentation.

#### Check the Linux kernel maximum buffer size

##### Diagnosis

- Run: `sysctl net.core.rmem_max`, it should return at least `net.core.rmem_max = 2147483647`.
  - This parameter specifies the maximum size of the "receive buffer" for each network connection, which determines the maximum amount of data that can be held in memory at any given time. By increasing the maximum buffer size, the operating system can accommodate larger bursts of data, which can help prevent network congestion and reduce packet loss, resulting in faster and more reliable data transfers.
- Run: `sysctl net.ipv4.ipfrag_time`, it should return around: `net.ipv4.ipfrag_time = 3`
  - The "net.ipv4.ipfrag_time" parameter specifies the maximum time in seconds that the kernel should retain partially fragmented IP packets before discarding them. The default value for this parameter is usually set to 30 seconds, but it may vary depending on the specific operating system and configuration.
  - By setting this parameter to a lower value, such as 3 seconds, the kernel can free up memory resources more quickly by discarding partially fragmented packets that are no longer needed, which can help improve the overall performance and stability of the system.
- Run: `sysctl net.ipv4.ipfrag_high_thresh`, it should return at around: `net.ipv4.ipfrag_high_thresh = 134217728`
  - The "net.ipv4.ipfrag_high_thresh" parameter specifies the high watermark threshold for the number of partially fragmented packets allowed in the kernel IP packet reassembly queue. When the number of partially fragmented packets in the queue exceeds this threshold, the kernel will start to drop newly arrived packets until the number of partially fragmented packets drops below the threshold.
  - By setting this parameter to a higher value, such as 134217728 (128 MB), the kernel can accommodate a larger number of partially fragmented packets in the queue, which can help improve the performance of network applications that transfer large amounts of data, such as file transfer protocols and multimedia streaming applications.

More info on these values: [Cross-vendor tuning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html#cross-vendor-tuning)

##### Solution

Either:

- Create the following file: `sudo touch /etc/sysctl.d/10-cyclone-max.conf` (recommended)

  - Edit the file to contain (`sudo gedit /etc/sysctl.d/10-cyclone-max.conf`):

    ```bash
    net.core.rmem_max=2147483647
    net.ipv4.ipfrag_time=3
    net.ipv4.ipfrag_high_thresh=134217728 # (128 MB)
    ```

    - Either restart the computer or run following to enable the changes:

      ```bash
      sudo sysctl -w net.core.rmem_max=2147483647
      sudo sysctl -w net.ipv4.ipfrag_time=3
      sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
      ```

- **OR** put following lines to the `~/.bashrc` file:

  ```bash
  if [ ! -e /tmp/kernel_network_conf_is_set ]; then
  sudo sysctl -w net.core.rmem_max=2147483647
  sudo sysctl -w net.ipv4.ipfrag_time=3
  sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728 # (128 MB)
  fi
  ```

  - This will probably ask for password on the terminal every time you restart the computer.

### Check if ROS localhost only communication is enabled

- If you are using multi computer setup, please skip this check.
- Enabling ROS localhost only communication can help improve the performance of ROS by reducing network traffic and avoiding potential conflicts with other devices on the network.
- Also check [Enable localhost-only communication](../../installation/additional-settings-for-developers/index.md#enabling-localhost-only-communication)

#### Target symptoms

- You see topics that shouldn't exist
- You see point clouds that don't belong to your machine
  - They might be from another computer running ROS 2 on your network
- Point clouds or markers flicker on RViz2
  - Another publisher (on another machine) may be publishing on the same topic as your node does.
  - Causing the flickering.

#### Diagnosis

Run following to check it:

```bash
echo $ROS_LOCALHOST_ONLY
```

The return line should be `1`. If not, apply the solution.

#### Solution

- Add `export $ROS_LOCALHOST_ONLY=1` as a separate line in you `~/.bashrc` file.
  - This environment variable tells ROS to only use the `loopback` network interface (i.e., localhost) for communication, rather than using the network interface card (NIC) for Ethernet or Wi-Fi. This can reduce network traffic and potential conflicts with other devices on the network, resulting in better performance and stability.
