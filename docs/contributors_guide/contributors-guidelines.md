Guidelines and Best Practices {#contributor-guidelines}
===========

@tableofcontents

# Contribution Workflow

1. [Create an issue](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/new?issue%5Bassignee_id%5D=&issue%5Bmilestone_id%5D=) defining your intended contribution
  1. Use one of the provided templates [by selecting one from the drop-down list](https://docs.gitlab.com/ee/user/project/description_templates.html#using-the-templates).
  2. Select yourself in the `Assignee` field.
  3. If you have permissions to do so, assign to an appropriate milestone. If you do not have permissions, [mention](https://about.gitlab.com/blog/2016/03/08/gitlab-tutorial-its-all-connected/#mention-others-and-assign) a [maintainer](https://gitlab.com/groups/autowarefoundation/autoware.auto/committers/-/group_members) in the issue for milestone assignment.
  4. The issue template you choose will assign one appropriate label for the issue type (bug, discussion, feature, or improvement).
     Assign any additional labels from the available list that you feel are appropriate for the issue's status or other attributes.

2. Create a fork
  1. For more information about the fork-and-pull model, see the [Develop in a Fork](@ref develop-in-a-fork) page.

3. Write code

4. Create a merge request
  1. For more information about the fork-and-pull model, see the [Develop in a Fork](@ref develop-in-a-fork) page.

5. Finish a merge request

  1. The review process for the MR is as follows:
    1. Submitter adds Reviewer(s) and assigns herself.
    2. Reviewer reviews and submits comments.
    3. Assignee addresses the reviews making necessary changes and notifies Reviewer(s) to have another look.
    4. Repeat 2 and 3 as necessary.
    5. When Reviewer approves, the MR should be assigned to the `@merge-bot` account which will automatically rebase and merge it.
  2. In order for a merge request to be merged to Autoware.Auto, it must meet the following criteria:
    - All discussions on the merge request must be resolved.
    - All items of the merge-request checklist are checked off.
    - CI jobs for the merge request must have passed successfully.
  3. Once your merge request has been accepted by the reviewer, assign the merge request to the `@merge-bot` account.


# Guidelines for General Code Development

Only C++14 and below is allowed for functional code.
Python 3.7+ and Bash are allowed for tooling.
CMake is the preferred build system, it should integrate with Colcon.
Deviations need to be approved by the maintainers.

The requirements for C++14 and Python 3.7+ align with compiler and tooling support found in ROS Foxy.
This may change in the future as new OS or ROS environments are targeted; see @ref target-environments for details.

## License and Copyright
Every file should have a license header; see the *License* section in [CONTRIBUTING.md](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/CONTRIBUTING.md).

## Building
See @ref building.

## Committing {#contributors-guidelines-committing}

Developers should commit and push regularly to GitLab to avoid data loss. Commit messages should follow
common standards as laid out in this [post](https://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html). In
summary,

1. Write your commit message in the imperative
1. In the mandatory first line, summarize *what* the functional change is, not *why* it is introduced
1. Optionally add more details separated by a blank line

As a general recommendation, add a reference to the issue in the commit message so it is easier for others in the future
to get more context about the changes in the commit. If the commit doesn't refer to a particular issue but only touches
a particular package or aspect, add a reference to that.

Example summary line referring to issue #716:

    [#716] Expand contributor guidelines

Example summary line without an issue:

    [CI] Disable flaky tests in foo and bar packages

There is assistance via `git` hooks to help with commit messages. Navigate to `.git/hooks` in the checkout of
Autoware.Auto, then:

```{bash}
ln -s ../../.git-hooks/prepare-commit-msg  # prepend issue number
ln -s ../../.git-hooks/commit-msg          # check formatting of commit message
```
## Cross-platform Compatibility

It is preferred to use cross-platform solutions for system-level function calls whenever possible. While the C++
standard library should be used for as many tasks as possible, some functions (such as `std::filesystem`) are not
available in C++14 in cross-platform implementations. This usually means utilizing libraries like
[`asio`](https://think-async.com/Asio/index.html) for networking tasks and [`a std::filesystem
shim`](https://github.com/gulrak/filesystem) for filesystem navigation is preferred to creating platform-specific
implementations.

## Documentation

To check that the code is properly documented and all documents are correctly linked, you can run `AutowareAuto/docs/.doxygen/build.py`.
The generated documentation can be found in `AutowareAuto/docs/_build/html/index.html`.
For more details see the [documentation guide](@ref writing-documentation).

## Formatting {#contributors-guidelines-formatting}

Autoware.Auto follows ROS recommendations for code style and formatting. See the [Coding Style and Language Versions
entry for C++](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/#id3) or the [Coding Style and
Language Versions entry for Python](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/#python)
for more information. We enforce these guidelines using linters provided with `ament` as far as possible. All
packages should have the following in their `package.xml` files:

```{xml}
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
```

In addition, the following should be in the package's `CMakeLists.txt` (extended with other tests):

```{cmake}
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
```

In CI, merge requests fail if they introduce improperly formatted code. To avoid that, format the
C++ code locally with

```{bash}
ament_uncrustify --reformat file.cpp         # update single file in place
ament_uncrustify --reformat path/to/pkg_foo  # update all C++ source files in package
```

With the above CMake setup, run all linters together with all other tests of a package as described in the [Running
Tests](#contributors-guidelines-run-tests) section or run a specific linter; e.g.,

```{bash}
ament_cpplint path/to/pkg_foo
```

Tools such as CLion can parse the output of the previous command and provide fast navigation to
offending lines in the code.

### Using ament_clang_format

`ament_uncrustify --reformat` is able to format the code to a degree but its results
are generally not enough to pass `ament_cpplint`. To automate the process,
`ament_clang_format` can be used like:
```{bash}
AutowareAuto $ ament_clang_format --config .clang-format --reformat file.cpp
```

The configuration is stored in the text file `.clang-format` in the base directory of a source
checkout of Autoware.Auto.

`ament_clang_format` is available in ADE by default. When working outside of ADE, install it e.g.
with

```{bash}
sudo apt install ros-${ROS_DISTRO}-ament-clang-format
```

A way to use all these three tools is as follows:
-# `ament_clang_format --config AutowareAuto/.clang-format --reformat file.cpp`
-# `ament_uncrustify --reformat file.cpp`
-# `ament_cpplint file.cpp`
-# Fix all reported errors
-# Repeat the previous steps until `ament_uncrustify` and `ament_cpplint` give no more errors.

#### Order of header includes

The recommended order of `#include` directives is based on the [google styleguide](https://google.github.io/styleguide/cppguide.html#Names_and_Order_of_Includes)

-# corresponding file: from `bar.cpp` in package `foo`, that's `#include "foo/bar.hpp"`
-# C system headers; e.g. `#include <stdint.h>`
-# C++ system headers; e.g. `#include <vector>`
-# headers from this or other packages; e.g. `#include "pkg/baz.hpp"`
-# message headers; e.g. `#include "foo/msg/bar.hpp"`

with headers in each group sorted alphabetically and groups separated by a single blank line.

In order for automatic grouping and sorting to work:

-# Use `#include "foo/bar.hpp"` for headers from the same and other packages
-# Use `#include <vector>` with **angle brackets only for C and C++ system headers**
-# Invoke `ament_clang_format` like explained above.

The resulting order should satisfy `ament_cpplint`.

@note If cpplint complains about the order of headers, ensure that the delimiters match the above
rules. Example error:

```console
Found C system header after C++ system header.
```

### Utilizing git pre-commit

To lint the code automatically before each commit, activate the `pre-commit`
[hook](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/.git-hooks/pre-commit). From the
repository base directory, do:

```{bash}
cd .git/hooks
ln -s ../../.git-hooks/pre-commit
```

## Code Coverage {#contributors-guidelines-coverage}

For [code coverage](https://en.wikipedia.org/wiki/Code_coverage), we use the popular [GNU tool
`lcov`](http://ltp.sourceforge.net/coverage/lcov.php) for generating statistics about line coverage. For every merge
request, we run our test suite and report the percentage of lines covered. We aim for a 100% line coverage and
continuously improve our test suite to achieve that number. In particular, we do not accept changes that reduce the
coverage value. If a merge request has a lower line coverage than `master`, we will request the contributor to add more
tests.

The coverage report for the latest successful CI run on the `master` branch is available
[here](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/coverage/index.html).

Check @ref unit-testing-coverage for details to create that report manually or @ref testing-overview for more information.

## C++ Resources

- [cppreference.com](https://en.cppreference.com/w/)
- [C++ Core Guidelines](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)

# Guidelines for ROS Development

In general, Autoware.Auto follows the [ROS 2 Developer
Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/) for contributions, except where noted. Some special
items of note that are not described in the ROS 2 Developer Guide are listed below.

## Creating a New Package

Basic instructions for creating a new ROS 2 package can be found [in this
tutorial](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/#create-your-own-package). In Autoware.Auto, much of
the boilerplate code can be automatically generated by utilizing the `autoware_auto_create_pkg` tool.

For more information on using the tool, see \ref autoware_auto_create_pkg-package-design.

## 2-Tier Development Pattern

In all but the most trivial utilities, it is best to implement a code pattern with *at least* two tiers of abstraction
which would look something like:

1. A "core," pure C++ class which performs all basic algorithmic and utility functions which are not ROS-related.

   This class may use ROS utilities such as logging or message structures, but such use must be justified in terms of
   why it cannot be done via the class's external interface (e.g. the enclosing node uses information obtained via the
   class's external interface to populate log messages).
2. A "ROS Node" or "ROS Component" class which inherits from `rclcpp::Node` or a subclass, handles all ROS-specific
   functions.

   This class should instantiate the class defined in 1. and register the node as a
   [component](#contributors-guidelines-components), so it can be created with launch files.

In the rare case that fine-grained control over execution is desired, create a main function in a separate file with a
[ROS Executor](http://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Executor.html#details) to control provision of
execution time of the node in some way (e.g. through calling `spin()`).

This design pattern helps to promote separation of concerns and code re-use. The core and the ROS node(s) can be
implemented in separate packages; e.g. `foo` and `foo_nodes`. There are some trivial cases where a simple ROS Node that
does not require a "core" are acceptable but these should be the exception, not the rule.

## Naming in Autoware.Auto

The [Naming Guidelines](@ref autoware-common-naming-guidelines) provide for standard, reliable naming and namespacing
conventions which should be used in all Autoware.Auto packages.

## On Topics and Parameters

In most cases, topics should receive a default name in code and be remapped if needed. Providing topic names as ROS
parameters is an anti-pattern, with few exceptions.

Required parameters should not have default values but fail during construction if no value is provided.

### Parameter File Syntax

To avoid the need to change parameter files based on the namespacing or node name of a node, use the "double-star"
syntax. e.g.:

```{yaml}
/**:
  ros__parameters:
    param1: value
```

The above parameter file can be passed to any node regardless of namespace or name and the parameters will populate
those of the node if the declared parameters match those in the file.

## ROS Components {#contributors-guidelines-components}

As of ROS Dashing, the recommended way to write Nodes in ROS 2 is using Components.
For more information about components and their use, see [the ROS Composition Guide](https://index.ros.org/doc/ros2/Tutorials/Composition/).
To implement your node as a Component, it must conform to the items below (using `ListenerNode` as an example):

- Must inherit from `rclcpp::Node` or a subclass (such as `rclcpp::LifecycleNode`)
- Must use a single-argument constructor in the form of:

```{cpp}
namespace composition_example
{
class ListenerNode: public rclcpp::Node {
  ListenerNode(const rclcpp::NodeOptions & options)
  : Node("listener", options)
  {
    ...
  }
}
}  // namespace composition_example
```

- Must contain a registration macro and header in a single translation unit. For example, the following at the bottom of your `cpp` file would suffice:

```{cpp}
// Insert at bottom of translation unit, e.g. listener_node.cpp
#include <rclcpp_components/register_node_macro.hpp>
// Use fully-qualified name in registration
RCLCPP_COMPONENTS_REGISTER_NODE(composition_example::ListenerNode)
```

- Must compile the components as a shared library and register them in your `CMakeLists.txt` file.
- Must depend on the `rclcpp_components` package.

### Minimal CMake Example {#contributors-guidelines-minimal-cmake-example}

The following is a minimal `CMakeLists.txt` file which uses the recommended `ament_cmake_auto` macros, registers a
single component, builds a stand-alone node which uses the component, and exports it as a dependency for downstream
packages. It can be conveniently created by
[`autoware_auto_create_pkg`](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/autoware_auto_create_pkg-package-design.html):

```{cmake}
cmake_minimum_required(VERSION 3.5)
project(composition_example)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(listener_node SHARED src/listener_node.cpp)
autoware_set_compile_options(listener_node)
rclcpp_components_register_nodes(listener_node "composition_example::ListenerNode")

ament_auto_add_executable(listener_node_exe src/listener_main.cpp)
autoware_set_compile_options(listener_node_exe)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
```

#### Compiler settings
The C++ standard is set in `autoware_auto_cmake.cmake` and becomes available to a package by
depending on the `autoware_auto_cmake` package in `package.xml` as shown below. Compiler options and
warning flags are set per target by calling the function `autoware_set_compile_options(${target})`
defined in `autoware_auto_cmake.cmake` as well. It should be applied to every C++ target and in
general Autoware.Auto code shall not generate warnings.

In case the warning flags are too strict for example when including external code, they can be selectively deactivated in special cases as follows:

```{cmake}
autoware_set_compile_options(${target})
target_compile_options(${target} PRIVATE -Wno-double-promotion)
```

### Minimal Package.xml Example {#contributors-guidelines-minimal-package-xml-example}

The following is a minimal `package.xml` file to go with the above `CMakeLists.txt` example:

```{xml}
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>composition_example</name>
  <version>0.0.1</version>
  <description>Example of node composition</description>
  <maintainer email="my.email@example.com">The Autoware Foundation</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake_auto</buildtool_depend>
  <buildtool_depend>autoware_auto_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```


## Resources

- [rclcpp_components in Dashing Diademata Release Notes](https://index.ros.org/doc/ros2/Releases/Release-Dashing-Diademata/#rclcpp-components)
