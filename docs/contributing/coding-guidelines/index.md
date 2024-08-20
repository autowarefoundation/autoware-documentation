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

As a basic rule, follow [the Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), [the C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md) and [Autoware C++ Coding Guidelines](./languages/cpp.md). The following are common areas requiring special attention:

- Use fixed-width integer types like `uint32_t` instead of `int`.
  - cf. [How to C in 2016](https://matt.sh/howto-c)
- Use `const` for variables whose values will not change after initialization, and for functions or arguments that guarantee they will not modify the object's state. This improves code safety and clarity by preventing unintended side effects.
- When not using an index, prefer `for (const auto & element : values)`.
  - cf. [C++ Core Guidelines, ES.71: Prefer a range-for-statement to a for-statement when there is a choice](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#es71-prefer-a-range-for-statement-to-a-for-statement-when-there-is-a-choice)
- For index access, prefer `.at(idx)` over `[idx]` whenever possible for safety.
  - cf. [C++ Core Guidelines, Avoid bounds errors](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#slcon3-avoid-bounds-errors)
- Avoid using auto with `Eigen::Matrix` or `Eigen::Vector` variables, as it can lead to bugs.
  - cf. [Eigen, C++11 and the auto keyword](https://eigen.tuxfamily.org/dox/TopicPitfalls.html)
- Safe memory management is crucial for accident-free autonomous vehicles, so `new`, `delete`, `malloc`, `free`, and raw pointers are generally not acceptable.
  - cf. [C++ Core Guidelines, C.149: Use unique_ptr or shared_ptr to avoid forgetting to delete objects created using new](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c149-use-unique_ptr-or-shared_ptr-to-avoid-forgetting-to-delete-objects-created-using-new), [C++ Core Guidelines, R.10: Avoid malloc() and free()](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#r10-avoid-malloc-and-free)
- When using `mutex`, use `std::lock_guard` instead of `std::lock` and `std::unlock`.
  - cf. [C++ Core Guidelines, RAII](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#Rr-raii)
- Use assert to validate that function arguments and return values meet specific conditions or expectations.
- Be cautious when using `throw`. C++ exceptions should only be used when it is acceptable for the process to terminate if uncaught. This applies to scenarios where Autoware has no other choice but to shut down.
  - cf. [Google C++ Style Guide, Exceptions](https://google.github.io/styleguide/cppguide.html#Exceptions)

## ROS 2 Style Guide

As a basic rule, follow [the ROS 2 developer guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html) and [the ROS nodes guidelines](./ros-nodes/class-design.md). The following are commonly pointed out areas that require attention:

- Use `RCLCPP_*` (e.g. `RCLCPP_INFO`) macros instead of `printf` or `std::cout` for logging.
  - Reasons include the following:
    - It allows for consistent log level management. For instance, with `RCLCPP_*` macros, you can simply set `--log_level` to adjust the log level uniformly across the application.
    - You can standardize the format using `RCUTILS_CONSOLE_OUTPUT_FORMAT`.
    - With `RCLCPP_*` macros, logs are automatically recorded to `/rosout`. These logs can be saved to a rosbag, which can then be replayed to review the log data.
- Follow [the directory structure guideline](./ros-nodes/directory-structure.md).
- If `RCLCPP_INFO` generates a large amount of logs, use `RCLCPP_INFO_THROTTLE`.
  - cf. [Coding guidelines, ros-nodes, Use throttled logging when the log is unnecessarily shown repeatedly](./ros-nodes/console-logging.md#use-throttled-logging-when-the-log-is-unnecessarily-shown-repeatedly-required-non-automated)
- Avoid setting default values in `declare_parameter`. Instead, specify them in a config file or launch file.

## Autoware Style Guide

For Autoware-specific styles, refer to the following:

- Use the `autoware_` prefix for package names.
  - cf. [Prefix packages with autoware\_](https://github.com/orgs/autowarefoundation/discussions/4097)
- Add implementations within the `autoware` namespace.
  - cf. [Prefix packages with autoware\_, Option 3:](https://github.com/orgs/autowarefoundation/discussions/4097#discussioncomment-8384169)
- The header files to be exported must be placed in the `PACKAGE_NAME/include/autoware/` directory.
  - cf. [Directory structure guideline, Exporting headers](./ros-nodes/directory-structure.md#exporting-headers)
- In `CMakeLists.txt`, use `autoware_package()`.
  - cf. [autoware_cmake README](https://github.com/autowarefoundation/autoware_cmake/tree/main/autoware_cmake)
