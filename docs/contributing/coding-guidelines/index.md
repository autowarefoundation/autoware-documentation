# Coding guidelines

!!! warning

    Under Construction

## Common guidelines

Refer to the following links for now:

- <https://docs.ros.org/en/humble/Contributing/Developer-Guide.html>

Also, keep in mind the following concepts.

- Keep things consistent.
- Automate where possible, using simple checks for formatting, syntax, etc.
- Write comments and documentation in English.
- Functions that are too complex (low cohesion) should be appropriately split into smaller functions.
- Try to minimize the use of member variables or global variables that have a large scope.
- Whenever possible, break large pull requests into smaller, manageable PRs.
- When it comes to code reviews, don't spend too much time on trivial disagreements. For details see:
  - <https://en.wikipedia.org/wiki/Law_of_triviality>
  - <https://steemit.com/programming/@emrebeyler/code-reviews-and-parkinson-s-law-of-triviality>

## C++ Style Guide

As a basic rule, follow [the Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) and [the C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md). The following are common areas requiring special attention:

- Use fixed-width integer types like `uint32_t` instead of `int`.
  - cf. [How to C in 2016](https://matt.sh/howto-c)
- Use `const` for variables whose values will not change after initialization, and for functions or arguments that guarantee they will not modify the object's state. This improves code safety and clarity by preventing unintended side effects.
- When not using an index, prefer `for (const auto & e : v)`.
- For index access, prefer `.at(idx)` over `[idx]` whenever possible for safety.
- Avoid using auto with `Eigen::Matrix` or `Eigen::Vector` variables, as it can lead to bugs.
- Safe memory management is crucial for accident-free autonomous vehicles, so `new`, `delete`, `free`, and raw pointers are generally not acceptable.
- When using `mutex`, use `std::lock_guard` instead of `std::lock` and `std::unlock`.
- Use assert to validate that function arguments and return values meet specific conditions or expectations.
- Be cautious when using `throw`. C++ exceptions should only be used when it's acceptable for the process to terminate if uncaught. This applies to scenarios where Autoware has no other choice but to shut down.

## ROS 2 Style Guide

As a basic rule, follow [the ROS 2 developer guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html) and [the ROS nodes guidelines](./ros-nodes/class-design.md). The following are commonly pointed out areas that require attention:

- Use `RCLCPP` instead of `printf` or `std::cout`.
  - Reasons include the following:
    - It allows for consistent log level management. For instance, with `RCLCPP`, you can simply set `--log_level` to adjust the log level uniformly across the application.
    - You can standardize the format using `RCUTILS_CONSOLE_OUTPUT_FORMAT`.
    - With `RCLCPP`, logs are automatically recorded to `/rosout`. These logs can be saved to a rosbag, which can then be replayed to review the log data.
- Follow [the directory structure guideline](./ros-nodes/directory-structure.md).
- If `RCLCPP_INFO` generates a large amount of logs, use `RCLCPP_INFO_THROTTLE`.
- Avoid setting default values in `declare_parameter`. Instead, specify them in a config file or launch file.

## Autoware Style Guide

For Autoware-specific styles, refer to the following:

- Use the `autoware_` prefix for package names.
- Add implementations within the `autoware` namespace.
- In `CMakeLists.txt`, use `autoware_package()`.
  - cf. [autoware_cmake README](https://github.com/autowarefoundation/autoware_cmake/tree/main/autoware_cmake)
