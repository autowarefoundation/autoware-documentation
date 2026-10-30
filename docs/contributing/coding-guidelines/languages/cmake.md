# CMake

This page describes how to write the `CMakeLists.txt` file of an Autoware package. Autoware packages use [ament_cmake_auto](https://github.com/ament/ament_cmake/tree/humble/ament_cmake_auto) and the macros of [autoware_cmake](https://github.com/autowarefoundation/autoware_cmake/tree/main/autoware_cmake). With these tools, a package declares its dependencies only once, in `package.xml`.

## References

Follow the CMake section of the ROS 2 code style when this page does not define a rule:

- <https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#cmake>

In short, the ROS 2 style asks for these rules:

- Use lowercase command names (`find_package`, not `FIND_PACKAGE`).
- Use `snake_case` for variables, functions, and macros.
- Use empty `else()` and `end...()` commands.
- Use two spaces of indentation. Do not use tabs.

## Use the autoware_package macro

To reduce duplications in `CMakeLists.txt`, there is the `autoware_package()` macro. Call it directly after `project()` and before you define build targets. See the [README](https://github.com/autowarefoundation/autoware_cmake/tree/main/autoware_cmake) for details.

`autoware_package()` does these steps ([source](https://github.com/autowarefoundation/autoware_cmake/blob/main/autoware_cmake/cmake/autoware_package.cmake)):

- It sets the C++ standard to C++17, if the package does not set `CMAKE_CXX_STANDARD`.
- It adds the `-Wall -Wextra -Wpedantic` compiler options and treats warnings as errors.
- It defines a macro for the ROS distribution, for example `ROS_DISTRO_HUMBLE` or `ROS_DISTRO_JAZZY`. Use it when code must differ between distributions.
- It calls `ament_auto_find_build_dependencies()`, which finds all build dependencies in `package.xml`.
- It adds the Eigen include directory as a system include directory.
- If `BUILD_TESTING` is on, it calls `ament_lint_auto_find_test_dependencies()`, which runs the linters of the test dependencies.

Because `autoware_package()` finds the dependencies, you do not have to call `find_package()` for a package that `package.xml` already lists. Call `find_package()` yourself when the CMake name of a dependency is different from its name in `package.xml`. For example, [`autoware_ekf_localizer`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/localization/autoware_ekf_localizer/CMakeLists.txt) calls `find_package(Eigen3 REQUIRED)` for the `eigen` dependency.

The package must declare `autoware_cmake` as a build tool dependency in `package.xml`:

```xml
<buildtool_depend>ament_cmake_auto</buildtool_depend>
<buildtool_depend>autoware_cmake</buildtool_depend>
```

## Use the autoware_ament_auto_package macro

End `CMakeLists.txt` with `autoware_ament_auto_package()` instead of `ament_auto_package()`. The macro installs the libraries, the executables, and the exported headers. It also exports each build dependency that is a run dependency too. It accepts the same `INSTALL_TO_PATH` and `INSTALL_TO_SHARE` arguments as `ament_auto_package()`.

Use `INSTALL_TO_SHARE` to install directories such as `launch` and `config` to `share/<package_name>/`. The `USE_SCOPED_HEADER_INSTALL_DIR` option installs the headers to `include/<package_name>/` instead of `include/`. Many packages of `autoware_core` use this option. For the background, see the [autoware_cmake README](https://github.com/autowarefoundation/autoware_cmake/tree/main/autoware_cmake#autoware_ament_auto_packagecmake).

## Example

This is the `CMakeLists.txt` of [`autoware_vehicle_velocity_converter`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/sensing/autoware_vehicle_velocity_converter/CMakeLists.txt) in `autoware_core` 1.10.0. It builds a library with the algorithm, a library with the ROS 2 node, a composable node executable, and two tests.

```cmake
cmake_minimum_required(VERSION 3.14)
project(autoware_vehicle_velocity_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/vehicle_velocity_converter.cpp
)

ament_auto_add_library(${PROJECT_NAME}_ros SHARED
  src/vehicle_velocity_converter_node.cpp
)

target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}_ros
  PLUGIN "autoware::vehicle_velocity_converter::VehicleVelocityConverterNode"
  EXECUTABLE ${PROJECT_NAME}_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_vehicle_velocity_converter.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  target_include_directories(test_${PROJECT_NAME} PRIVATE src)

  if(NOT (DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1"))
    ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_node
      test/test_vehicle_velocity_converter_node.cpp
    )
    target_link_libraries(test_${PROJECT_NAME}_node ${PROJECT_NAME} ${PROJECT_NAME}_ros)
    target_include_directories(test_${PROJECT_NAME}_node PRIVATE src)
  endif()
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
```

The example shows these conventions:

- Use `cmake_minimum_required(VERSION 3.14)`. The ROS 2 code style gives 3.14 as the minimum CMake version that a package must support.
- Give `project()` the name of the package.
- Add libraries and executables with `ament_auto_add_library()` and `ament_auto_add_executable()`. These commands link the dependencies from `package.xml` automatically.
- Build a node as a shared library and register it as a composable node component. Most packages use `rclcpp_components_register_node()`. Packages that support the zero-copy middleware Agnocast use `autoware_agnocast_wrapper_register_node()` from [`autoware_agnocast_wrapper`](https://github.com/autowarefoundation/autoware_core/tree/1.10.0/common/autoware_agnocast_wrapper). For the naming rules of the executables, see [Directory structure](../ros-nodes/directory-structure.md#cmakeliststxt).
- Put the tests in an `if(BUILD_TESTING)` block. Use `ament_auto_add_gtest()` for a test that does not use ROS 2 communication. Use `ament_add_ros_isolated_gtest()` for a test that starts ROS 2 nodes, so that parallel tests do not see the topics of other tests. For more information, see [Unit testing](../../testing-guidelines/unit-testing.md).

## Linting

A package that has `autoware_lint_common` as a test dependency runs `ament_cmake_lint_cmake` during `colcon test`. This linter checks the style of `CMakeLists.txt` and of the `.cmake` files of the package.
