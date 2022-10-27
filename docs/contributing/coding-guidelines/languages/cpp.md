# C++

!!! warning

    Under Construction

## References

- <https://docs.ros.org/en/humble/Contributing/Code-Style-Language-Versions.html#id1>
- <https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines>
- <https://www.autosar.org/fileadmin/user_upload/standards/adaptive/21-11/AUTOSAR_RS_CPP14Guidelines.pdf>

## Rules

### Include header files in the defined order (required, partially automated)

Include the headers in the following order:

- Local package headers
- Other package headers
- Message headers
- Boost headers
- C system headers
- C++ system headers

#### Rationale

- Detecting indirect dependencies.

#### Reference

- <https://llvm.org/docs/CodingStandards.html#include-style>

#### Example

```cpp
#include "my_header.hpp"

#include <package1/foo.hpp>
#include <package2/bar.hpp>

#include <std_msgs/msg/header.hpp>

#include <iostream>
#include <vector>
```

### Use lower snake case for function names (required, partially automated)

#### Rationale

- It is consistent with C++ standard library and Python.

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

- It is consistent with ROS2 libraries.

#### Exception

- None

#### Reference

- None

#### Example

```cpp
enum class Color
{
  Red, Green, Blue
}
```

### Use lower snake case for constant names (required, partially automated)

#### Rationale

- It is consistent across Autoware.

#### Exception

- Constants defined in the rosidl file, such as `.msg` and `.srv`.

#### Reference

- None

#### Example

```cpp
constexpr double gravity = 9.80665;
```

### Treat acronyms like normal words (required, partially automated)

#### Rationale

- To clarify the boundaries of words when acronyms are consecutive.

#### Exception

- None

#### Reference

- None

#### Example

```cpp
class RosApi;
RosApi ros_api;
```
