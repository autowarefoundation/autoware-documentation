# Testing guidelines

## Unit Tests

- C++ code should be tested using `gtest`.
- Test code should be written in a subfolder `test` at the root of the package.
- The CMake command [`ament_add_ros_isolated_gtest`](https://github.com/ros2/ament_cmake_ros/blob/master/ament_cmake_ros/cmake/ament_add_ros_isolated_gtest.cmake) should be used to run tests in isolation mode and prevent any ROS communication to interfere with another test.