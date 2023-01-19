# C++

!!! warning

    Under Construction

## References

Follow the guidelines below if a rule is not defined on this page.

1. <https://docs.ros.org/en/humble/Contributing/Code-Style-Language-Versions.html>
2. <https://www.autosar.org/fileadmin/standards/adaptive/22-11/AUTOSAR_RS_CPP14Guidelines.pdf>
3. <https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines>

Also, it is encouraged to apply Clang-Tidy to each file.
For the usage, see [Applying Clang-Tidy to ROS packages](../../../how-to-guides/applying-clang-tidy-to-ros-packages.md).

Note that not all rules are covered by Clang-Tidy.

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
}
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
