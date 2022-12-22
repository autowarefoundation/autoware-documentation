# Add a custom ROS message

## Overview

During the Autoware development, you will probably need to define your own messages. Read the following instructions before adding a custom message.

1. Message in [autoware_msgs](https://github.com/autowarefoundation/autoware_msgs) define interfaces of `Autoware Core`.

   - If a contributor wishes to make changes or add new messages to `autoware_msgs`, they should first create a new discussion post under the [Design category](https://github.com/orgs/autowarefoundation/discussions/categories/design).

2. Any other minor or proposal messages used for internal communication within a component(such as planning) should be defined in another repository.

   - [tier4_autoware_msgs](https://github.com/tier4/tier4_autoware_msgs) is an example of that.

The following is a simple tutorial of adding a message package to `autoware_msgs`. For the general ROS2 tutorial, see [Create custom msg and srv files](http://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).

## How to create custom message

Make sure you are in the Autoware workspace, and then run the following command to create a new package.
As an example, let's create a package to define sensor messages.

1. Create a package

   ```console
   cd ./src/core/autoware_msgs
   ros2 pkg create --build-type ament_cmake autoware_sensing_msgs
   ```

2. Create custom messages

   You should create `.msg` files and place them in the `msg` directory.

   **NOTE**: The initial letters of the `.msg` and `.srv` files must be capitalized.

   As an example, let's make `.msg` files `GnssInsOrientation.msg` and `GnssInsOrientationStamped.msg` to define GNSS/INS orientation messages:

   ```console
   mkdir msg
   cd msg
   touch GnssInsOrientation.msg
   touch GnssInsOrientationStamped.msg
   ```

   Edit `GnssInsOrientation.msg` with your editor to be the following content:

   ```c++
   geometry_msgs/Quaternion orientation
   float32 rmse_rotation_x
   float32 rmse_rotation_y
   float32 rmse_rotation_z
   ```

   In this case, the custom message uses a message from another message package `geometry_msgs/Quaternion`.

   Edit `GnssInsOrientationStamped.msg` with your editor to be the following content:

   ```c++
   std_msgs/Header header
   GnssInsOrientation orientation
   ```

   In this case, the custom message uses a message from another message package `std_msgs/Header`.

3. Edit CMakeLists.txt

   In order to use this custom message in `C++` or `Python` languages, we need to add the following lines to `CMakeList.txt`:

   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/GnssInsOrientation.msg"
     "msg/GnssInsOrientationStamped.msg"
     DEPENDENCIES
       geometry_msgs
       std_msgs
     ADD_LINTER_TESTS
   )
   ```

   :speech_balloon: The `ament_cmake_auto` tool is very useful and is more widely used in Autoware, so we recommend using `ament_cmake_auto` instead of `ament_cmake`.

   We need to replace

   ```cmake
   find_package(ament_cmake REQUIRED)

   ament_package()
   ```

   with

   ```cmake
   find_package(ament_cmake_auto REQUIRED)

   ament_auto_package()
   ```

4. Edit package.xml

   We need to declare relevant dependencies in `package.xml`. For the above example we need to add the following content:

   ```xml
   <buildtool_depend>rosidl_default_generators</buildtool_depend>

   <exec_depend>rosidl_default_runtime</exec_depend>

   <depend>geometry_msgs</depend>
   <depend>std_msgs</depend>

   <member_of_group>rosidl_interface_packages</member_of_group>
   ```

   We need to replace `<buildtool_depend>ament_cmake</buildtool_depend>` with `<buildtool_depend>ament_cmake_auto</buildtool_depend>` in the `package.xml` file.

5. Build the custom message package

   You can build the package in the root of your workspace, for example by running the following command:

   ```console
   colcon build --packages-select autoware_sensing_msgs
   ```

   Now the `GnssInsOrientationStamped` message will be discoverable by other packages in Autoware.

## How to use custom messages in Autoware

You can use the custom messages in Autoware by following these steps:

- Add dependency in `package.xml`.
  - For example, `<depend>autoware_sensing_msgs</depend>`.
- Include the `.hpp` file of the relevant message in the code.
  - For example, `#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>`.
