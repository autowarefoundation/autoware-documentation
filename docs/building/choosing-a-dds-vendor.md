Choosing a DDS Vendor {#choosing-a-dds-vendor}
==============================================

@tableofcontents

Choosing a DDS vendor is usually as simple as changing the `RMW_IMPLEMENTATION` environment variable. It can be set after building Autoware, as long as the respective `rmw_*` packages are installed before building, because the software is built for all the available typesupports installed. The rmw layer loads a given `rmw_*` implementation at runtime via the `RMW_IMPLEMENTATION` variable, unless there's only one, in which case it shortcuts to the installed RMW implementation. The change can also be made more permanent by changing it in the `ADE_DOCKER_RUN_ARGS` in the `.aderc` file.

For more information about why one would want to use a different DDS vendor and which ones are available, see [this ROS Index article](https://index.ros.org/doc/ros2/Concepts/About-Different-Middleware-Vendors/).
For more information about working with multiple middleware (DDS) implementations, see [this ROS Index article](https://index.ros.org/doc/ros2/Tutorials/Working-with-multiple-RMW-implementations/).

The supported versions of each DDS implementation for a particular version of ROS are detailed in [REP-2000](https://ros.org/reps/rep-2000.html).

# Eclipse Cyclone DDS
[Eclipse Cyclone DDS](https://projects.eclipse.org/projects/iot.cyclonedds) is the default DDS in ADE and required for integration with the @ref lgsvl :
```{bash}
ade$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

# FastDDS
[FastDDS](https://www.eprosima.com/index.php/products-all/eprosima-fast-dds), formerly known as FastRTPS, is the default in ROS Dashing:
```{bash}
ade$ export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

# RTI Connext DDS
[RTI Connext DDS](pthttps://www.rti.com/products) is not installed in ADE by default, so one more step is needed:
```{bash}
ade$ sudo apt update
ade$ sudo apt-get install rti-connext-dds-5.3.1 ros-${ROS_DISTRO}-rmw-connext-cpp
ade$ export RMW_IMPLEMENTATION=rmw_connext_cpp
```

# GurumDDS
GurumDDS is not installed in ADE by default, so one more step is needed:
```{bash}
ade$ sudo apt update
ade$ sudo apt-get install gurumdds-2.6 ros-${ROS_DISTRO}-rmw-gurumdds-cpp
ade$ export RMW_IMPLEMENTATION=rmw_gurumdds_cpp
```
