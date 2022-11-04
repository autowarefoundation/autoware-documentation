# Add a custom ROS message

## Description

During the autoware development, you will probably need to define your own messages, The page only list a simple example of add a custom message to autoware, for basic tutorial see [Create custom msg and srv files](http://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html). 

## How to create custom message

Make sure you are in the autoware workspace, and then run the following command to create a new pakage. 

For example we create a package to define sensor message.
1. Create a package

   ```console
   ros2 pkg create --build-type ament_cmake autoware_sensing_msgs
   ```

1. Create custom message

   You should create `.msg` file and placed it in `msg` derectory.

   **NOTE**:  The initial letters of the `.msg` and `.srv` files must be capitalized. 

   For example we make  msg files `GnssInsOrientation.msg` and `GnssInsOrientationStamped.msg` to define GNSS INS orientation message:


   ```console
   mkdir msg
   cd msg
   touch GnssInsOrientation.msg
   touch GnssInsOrientationStamped.msg
   ```

    The `GnssInsOrientation.msg` with the following content:

    ```c++
    geometry_msgs/Quaternion orientation
    float32 rmse_rotation_x
    float32 rmse_rotation_x
    float32 rmse_rotation_x
    ``` 

    In this case, the custom message uses a message from another message package `geometry_msgs/Quaternion`.

    The `GnssInsOrientationStamped.msg` with the following content:

    ```c++
    std_msgs/Header header
    GnssInsOrientation orientation
    ``` 

    In this case, the custom message uses a message from another message package `std_msgs/Header`.

1. CmakeList.txt

   In order to use this custom message in `C++` or `Python` languages, we need add the following lines to `CmakeList.txt`:

   ```cmake
   set(msg_files
   "msg/GnssInsOrientation.msg"
   "msg/GnssInsOrientationStamped.msg")
   set(msg_dependencies
   geometry_msgs
   std_msgs)
   rosidl_generate_interfaces(${PROJECT_NAME}
   ${msg_files}
   DEPENDENCIES ${msg_dependencies}
   ADD_LINTER_TESTS)
   ```                                
   
1. package.xml

   We need to declare a relevant dependencies in `package.xml`. In up example we need add following contents:

   ```xml
   <depend>geometry_msgs</depend>
   <depend>std_msgs</depend>
   <buildtool_depend>rosidl_default_generators</buildtool_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   <member_of_group>rosidl_interface_packages</member_of_group>
   ```

1. Build the custom message package

   You can build the package in the root of workspace, for example run the following command:

   ```console
   colcon build --packages-select autoware_sensing_msgs
   ```

   Now the GnssInsOrientationStamped message will be discoverable by other packages in autoware.
## How to use custom message in autoware

You can use the custom messages in autoware by following steps:
- add dependency in `package.xml`
- include the `.hpp` file of the relevant message in the code.

For up example:

We should add following contents in the `package.xml`:

```xml
<depend>autoware_sensing_msgs</depend>
```

we should add following contents in the code:

```c++
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
```
