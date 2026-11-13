# package.xml

This page describes how to write the `package.xml` file of an Autoware package. The file is the package manifest. It gives the name, version, maintainers, license, and dependencies of the package. colcon, rosdep, and the build scripts read the dependencies from this file.

## References

- [REP 149: Package Manifest Format Three Specification](https://ros.org/reps/rep-0149.html)
- [tier4/pre-commit-hooks-ros](https://github.com/tier4/pre-commit-hooks-ros)

## Use format 3

Write the manifest in format 3, and add the `xml-model` line that points to the schema. The validation tools use this line to find the schema.

This is the `package.xml` of [`autoware_interpolation`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/common/autoware_interpolation/package.xml) in `autoware_core` 1.10.0:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>autoware_interpolation</name>
  <version>1.10.0</version>
  <description>The spline interpolation package</description>
  <maintainer email="fumiya.watanabe@tier4.jp">Fumiya Watanabe</maintainer>
  <maintainer email="takayuki.murooka@tier4.jp">Takayuki Murooka</maintainer>
  <maintainer email="junya.sasaki@tier4.jp">Junya Sasaki</maintainer>
  <maintainer email="taiki.yamada@tier4.jp">Taiki Yamada</maintainer>
  <maintainer email="arjun.ram@tier4.jp">Arjun Ram</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake_auto</buildtool_depend>
  <buildtool_depend>autoware_cmake</buildtool_depend>

  <depend>autoware_internal_planning_msgs</depend>
  <depend>autoware_planning_msgs</depend>
  <depend>autoware_utils_geometry</depend>
  <depend>eigen</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_geometry_msgs</depend>

  <test_depend>ament_cmake_ros</test_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>autoware_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Fill in the package information

- `<name>`: the package name. It starts with `autoware_` and is the same as the name of the package directory and the `project()` name in `CMakeLists.txt`. For details, see [Directory structure](../ros-nodes/directory-structure.md#package-name).
- `<version>`: the version of the package. In repositories such as `autoware_core` and `autoware_universe`, all packages have the same version, and the `bump-new-version` workflow updates it for a release. Do not change it in a feature pull request. Give a new package the version that the other packages of the repository have.
- `<description>`: one sentence that says what the package does.
- `<maintainer>`: one element for each maintainer, with the email address. The `update-codeowners-from-packages` workflow makes the `CODEOWNERS` file of the repository from these elements, so the maintainers are asked to review the pull requests that change the package.
- `<license>`: the license of the package. Autoware packages use `Apache License 2.0`.

## Declare all dependencies

List each package that your package uses. colcon uses the list to build the packages in the correct order, and rosdep uses it to install the system dependencies. A missing dependency can build on your computer and fail in CI or in the Docker images, which install only the declared dependencies.

Use the tag that matches how the package uses the dependency:

| Tag                     | Use it for                                                                                     |
| ----------------------- | ---------------------------------------------------------------------------------------------- |
| `<buildtool_depend>`    | Build tools, for example `ament_cmake_auto` and `autoware_cmake`.                              |
| `<depend>`              | Packages that the code uses when it builds and when it runs. Most dependencies use this tag.   |
| `<build_depend>`        | Packages that only the build uses, for example header-only libraries that are not exported.    |
| `<build_export_depend>` | Packages that the exported headers of your package need, but the package itself does not link. |
| `<exec_depend>`         | Packages that only run time uses, for example nodes that a launch file starts.                 |
| `<test_depend>`         | Packages that only the tests use, for example `ament_lint_auto` and `autoware_lint_common`.    |

Use the rosdep key of a system dependency, for example `eigen` or `python3-numpy`. To find a key, see the [rosdep database](https://github.com/ros/rosdistro/tree/master/rosdep).

The `check-package-depends` hook of [autoware-guideline-check](https://github.com/autowarefoundation/autoware-guideline-check) finds one type of missing dependency. It searches the `*.launch.xml` files of the package for `$(find-pkg-share <name>)`. If `package.xml` does not list `<name>`, the hook adds `<exec_depend><name></exec_depend>` to the file.

## Formatting

pre-commit formats `package.xml` with two hooks of [tier4/pre-commit-hooks-ros](https://github.com/tier4/pre-commit-hooks-ros):

- `sort-package-xml` puts all elements of one dependency tag next to each other and sorts them in alphabetical order. To keep one line in its place, start the line with `<!-- no format -->`.
- `prettier-package-xml` formats the file with Prettier and its XML plugin.

Put a blank line between the groups of different dependency tags, as in the example.

## Validation

A package that has `autoware_lint_common` as a test dependency runs `ament_cmake_xmllint` during `colcon test`. The linter validates `package.xml` against the schema in its `xml-model` line. A file that does not match the format 3 schema fails the test. For example, the schema requires the order `<name>`, `<version>`, `<description>`, `<maintainer>`, `<license>`, then the optional `<url>` and `<author>`, then the dependencies, and `<export>` at the end.

`ament_xmllint` downloads the schema from `download.ros.org` by default. The Autoware Docker images keep local copies of the schemas, so that the test does not need the network. [`docker/base.Dockerfile`](https://github.com/autowarefoundation/autoware/blob/2.0.0/docker/base.Dockerfile) downloads the format 1, 2, and 3 schemas to `/usr/share/xml/ros/` and sets `XML_CATALOG_FILES` to an XML catalog that points `xmllint` to them.
