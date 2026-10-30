# C++

This page describes the rules for C++ code in Autoware. Tools check many of the rules. The [tools](#tools) section lists them, and each rule says if a tool checks it.

## References

Follow the guidelines below if a rule is not defined on this page.

1. <https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html>
2. <https://www.autosar.org/fileadmin/standards/R22-11/AP/AUTOSAR_RS_CPP14Guidelines.pdf>
3. <https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines>

Also, it is encouraged to apply Clang-Tidy to each file.
For the usage, see [Applying Clang-Tidy to ROS packages](../../../tutorials/others/applying-clang-tidy-to-ros-packages.md).

Note that not all rules are covered by Clang-Tidy.

## Tools

### Language standard and compiler warnings

Autoware code uses C++17. The [`autoware_package()`](cmake.md#use-the-autoware_package-macro) macro sets `CMAKE_CXX_STANDARD` to 17 when the package does not set it. The macro also turns on `-Wall -Wextra -Wpedantic` and treats compiler warnings as errors. Fix each warning in your code. Do not turn the warnings off for the whole package.

### ClangFormat

pre-commit formats all C, C++, and CUDA files with [ClangFormat](https://clang.llvm.org/docs/ClangFormat.html). The configuration is the [`.clang-format`](https://github.com/autowarefoundation/sync-file-templates/blob/main/sources/.clang-format) file at the root of each repository. It is a modified copy of the ROS 2 configuration in [`ament_clang_format`](https://github.com/ament/ament_lint/blob/humble/ament_clang_format/ament_clang_format/configuration/.clang-format), which is based on the Google style. The main settings are:

- The column limit is 100 characters.
- The opening brace of a class, function, namespace, and struct goes on its own line.
- Access modifiers (`public:`, `private:`) have an indent of `-2`.
- Pointers and references are aligned in the middle: `int * ptr` and `const Foo & foo`.
- Include directives are sorted in the [order that this page defines](#include-header-files-in-the-defined-order-required-partially-automated).

The ClangFormat version is pinned in `.pre-commit-config.yaml`. To use the same version in your editor, see [Matching your local clang-format to pre-commit](../../pull-request-guidelines/ci-checks.md#matching-your-local-clang-format-to-pre-commit).

### cpplint

pre-commit runs [cpplint](https://github.com/cpplint/cpplint) with the [`CPPLINT.cfg`](https://github.com/autowarefoundation/sync-file-templates/blob/main/sources/CPPLINT.cfg) file at the root of each repository. The line length is 100. The configuration turns off the checks that conflict with ClangFormat and with the rules on this page, for example `whitespace/braces`, `build/include_order`, and `runtime/references`.

If cpplint reports a false positive, suppress it on that line with a `// NOLINT` comment that names the category and the reason. For example, [`status.hpp`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/common/autoware_component_interface_utils/include/autoware/component_interface_utils/status.hpp) of `autoware_component_interface_utils` has:

```cpp
void copy(const T1 & src, T2 & dst)  // NOLINT(build/include_what_you_use): cpplint false positive
```

### Include guards

Use include guards in header files. The `ros-include-guard` hook of [tier4/pre-commit-hooks-ros](https://github.com/tier4/pre-commit-hooks-ros) makes the macro name from the path of the file after the `include`, `src`, or `test` directory. It joins the path parts with `__`, replaces `.` with `_`, changes all letters to uppercase, and adds `_` at the end. The hook fixes a wrong name automatically. It accepts a header that uses `#pragma once` instead of an include guard. For example, the guard of `include/autoware/interpolation/linear_interpolation.hpp` in [`autoware_interpolation`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/common/autoware_interpolation/include/autoware/interpolation/linear_interpolation.hpp) is:

```cpp
#ifndef AUTOWARE__INTERPOLATION__LINEAR_INTERPOLATION_HPP_
#define AUTOWARE__INTERPOLATION__LINEAR_INTERPOLATION_HPP_

// ...

#endif  // AUTOWARE__INTERPOLATION__LINEAR_INTERPOLATION_HPP_
```

### Static analysis

These tools run in the CI of `autoware_core` and `autoware_universe`, not in pre-commit:

- Clang-Tidy checks the changed files of a pull request with the `.clang-tidy-ci` file of the repository. The configuration treats all enabled warnings as errors.
- Cppcheck checks the changed files with the `cppcheck-differential` workflow.
- `ament_cmake_cppcheck` runs during `colcon test` for each package that depends on [`autoware_lint_common`](https://github.com/autowarefoundation/autoware_cmake/tree/main/autoware_lint_common).

## Style rules

### Include header files in the defined order (required, partially automated)

#### Rationale

- Due to indirect dependencies, the include system of C++ makes different behaviors if the header order is different.
- To reduce unintended bugs, local header files should come first.

#### Reference

- <https://llvm.org/docs/CodingStandards.html#include-style>

#### Example

Include the headers in the following order:

- Main module header
- Local package headers
- Other package headers
- Message headers
- Boost headers
- C system headers
- C++ system headers

```cpp
// Compliant
#include "my_header.hpp"

#include "my_package/foo.hpp"

#include <package1/foo.hpp>
#include <package2/bar.hpp>

#include <std_msgs/msg/header.hpp>

#include <iostream>
#include <vector>
```

If you use `""` and `<>` properly, `ClangFormat` in `pre-commit` sorts headers automatically.

Do not define macros between `#include` lines because it prevents automatic sorting.

```cpp
// Non-compliant
#include <package1/foo.hpp>
#include <package2/bar.hpp>

#define EIGEN_MPL2_ONLY
#include "my_header.hpp"
#include "my_package/foo.hpp"

#include <Eigen/Core>

#include <std_msgs/msg/header.hpp>

#include <iostream>
#include <vector>
```

Instead, define macros before `#include` lines.

```cpp
// Compliant
#define EIGEN_MPL2_ONLY

#include "my_header.hpp"

#include "my_package/foo.hpp"

#include <Eigen/Core>
#include <package1/foo.hpp>
#include <package2/bar.hpp>

#include <std_msgs/msg/header.hpp>

#include <iostream>
#include <vector>
```

If there are any reasons for defining macros at a specific position, write a comment before the macro.

```cpp
// Compliant
#include "my_header.hpp"

#include "my_package/foo.hpp"

#include <package1/foo.hpp>
#include <package2/bar.hpp>

#include <std_msgs/msg/header.hpp>

#include <iostream>
#include <vector>

// For the foo bar reason, the FOO_MACRO must be defined here.
#define FOO_MACRO
#include <foo/bar.hpp>
```

### Use lower snake case for function names (required, partially automated)

#### Rationale

- It is consistent with the C++ standard library.
- It is consistent with other programming languages such as Python and Rust.

#### Exception

- For member functions of classes inherited from external project classes such as Qt, follow that naming convention.

#### Reference

- <https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#function-and-method-naming>

#### Example

```cpp
void function_name()
{
}
```

### Use upper camel case for enum names (required, partially automated)

#### Rationale

- It is consistent with ROS 2 core packages.

#### Exception

- Enums defined in the `rosidl` file can use other naming conventions.

#### Reference

- <http://wiki.ros.org/CppStyleGuide> (Refer to "15. Enumerations")

#### Example

```cpp
enum class Color
{
  Red, Green, Blue
};
```

### Use lower snake case for constant names (required, partially automated)

#### Rationale

- It is consistent with ROS 2 core packages.
- It is consistent with `std::numbers`.

#### Exception

- Constants defined in the `rosidl` file can use other naming conventions.

#### Reference

- <https://en.cppreference.com/w/cpp/numeric/constants>

#### Example

```cpp
constexpr double gravity = 9.80665;
```

### Count acronyms and contractions of compound words as one word (required, partially automated)

#### Rationale

- To clarify the boundaries of words when acronyms are consecutive.

#### Reference

- <https://rust-lang.github.io/api-guidelines/naming.html#casing-conforms-to-rfc-430-c-case>

#### Example

```cpp
class RosApi;
RosApi ros_api;
```

### Do not use the auto keywords with Eigen's expressions (required)

#### Rationale

- Avoid using auto with `Eigen::Matrix` or `Eigen::Vector` variables, as it can lead to bugs.

#### Reference

- [Eigen, C++11 and the auto keyword](https://libeigen.gitlab.io/eigen/docs-nightly/TopicPitfalls.html)

### Use RCLCPP\_\* (e.g. RCLCPP_INFO) macros instead of printf or std::cout for logging (required)

#### Rationale

- Reasons include the following:
  - It allows for consistent log level management. For instance, with RCLCPP\_\* macros, you can simply set `--ros-args --log-level` to adjust the log level uniformly across the application.
  - You can standardize the format using RCUTILS_CONSOLE_OUTPUT_FORMAT.
  - With RCLCPP\_\* macros, logs are automatically recorded to /rosout. These logs can be saved to a rosbag, which can then be replayed to review the log data.

#### Reference

- [Autoware Documentation for Console logging in ROS Node](../ros-nodes/console-logging.md)
