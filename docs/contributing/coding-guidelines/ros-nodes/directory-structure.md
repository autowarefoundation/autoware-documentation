# Directory structure

This document describes the directory structure of ROS nodes within Autoware.

We'll use the package `autoware_gnss_poser` as an example.

**Note that this example does not reflect the actual `autoware_gnss_poser` and includes extra files and directories to demonstrate all possible package structures.**

## C++ package

### Entire structure

- This is a reference on how the entire package might be structured.
- A package may not have all the directories shown here.

```txt
autoware_gnss_poser
├─ package.xml
├─ CMakeLists.txt
├─ README.md
│
├─ config
│   ├─ gnss_poser.param.yaml
│   └─ another_non_ros_config.yaml
│
├─ schema
│   └─ gnss_poser.schema.json
│
├─ design
│   └─ GnssPoser.node.yaml
│
├─ doc
│   ├─ foo_document.md
│   └─ foo_diagram.svg
│
├─ include  # for exporting headers
│   └─ autoware
│       └─ gnss_poser
│           └─ exported_header.hpp
│
├─ src
│   ├─ include
│   │   ├─ gnss_poser_node.hpp
│   │   └─ foo.hpp
│   ├─ gnss_poser_node.cpp
│   └─ bar.cpp
│
├─ launch
│   ├─ gnss_poser.launch.xml
│   └─ gnss_poser.launch.py
│
└─ test
    ├─ test_foo.hpp  # or place under an `include` folder here
    └─ test_foo.cpp
```

### Package name

- All the packages in Autoware should be prefixed with `autoware_`.
- Even if the package exports a node, the package name **should NOT** have the `_node` suffix.
- The package name should be in `snake_case`.

| Package Name                      | OK  | Alternative                  |
| --------------------------------- | --- | ---------------------------- |
| path_smoother                     | ❌  | autoware_path_smoother       |
| autoware_trajectory_follower_node | ❌  | autoware_trajectory_follower |
| autoware_geography_utils          | ✅  | -                            |

### Package folder

```txt
autoware_gnss_poser
├─ package.xml
├─ CMakeLists.txt
└─ README.md
```

The package folder name should be the same as the package name.

#### `package.xml`

- The package name should be entered within the `<name>` tag.
  - `<name>autoware_gnss_poser</name>`

#### `CMakeLists.txt`

- The [`project()`](https://cmake.org/cmake/help/latest/command/project.html) command should call the package name.
  - **Example:** `project(autoware_gnss_poser)`
- Call `autoware_package()` from [`autoware_cmake`](https://github.com/autowarefoundation/autoware_cmake/tree/1.4.0/autoware_cmake) before you define the build targets. It sets the common options of Autoware.
- End the file with `autoware_ament_auto_package()` instead of `ament_auto_package()`. It keeps the `autoware/<module>/` include structure on ROS 2 Humble, Jazzy, and Kilted. For the other build rules, see the [CMake guideline](../languages/cmake.md#use-the-autoware_ament_auto_package-macro).

##### Exporting a composable node component executables

For best practices and system efficiency, it is recommended to primarily use composable node components.

This method facilitates easier deployment and maintenance within ROS environments.

```cmake
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/gnss_poser_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::gnss_poser::GNSSPoser"
  EXECUTABLE ${PROJECT_NAME}_node
)
```

- If you are building:
  - **only a single composable node component,** the executable name should start with `${PROJECT_NAME}`
  - **multiple composable node components,** the executable name is up to the developer.
- All composable node component executables should have the `_node` suffix.

###### Nodes that support Agnocast

A node that supports Agnocast (see [Class design](./class-design.md#nodes-that-support-agnocast)) uses `autoware_agnocast_wrapper_register_node()` from [`autoware_agnocast_wrapper`](https://github.com/autowarefoundation/autoware_core/tree/1.10.0/common/autoware_agnocast_wrapper) instead of `rclcpp_components_register_node()`:

```cmake
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/gnss_poser_node.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::gnss_poser::GNSSPoser"
  EXECUTABLE ${PROJECT_NAME}_node
)
```

- The macro registers the component, like `rclcpp_components_register_node()`. If you build with `ENABLE_AGNOCAST=1`, it also creates an executable that selects `rclcpp` or Agnocast at runtime from the `ENABLE_AGNOCAST` environment variable. Without Agnocast, it behaves like `rclcpp_components_register_node()`.
- The naming rules for the executable stay the same.
- The optional `ROS2_EXECUTOR` and `AGNOCAST_EXECUTOR` arguments select the executor of the executable. See [Task scheduling](./task-scheduling.md#select-the-executor).

##### Exporting a standalone node executable without composition _(discouraged for most cases)_

Use of standalone executables **should be limited** to cases where specific needs such as debugging or tooling are required.

[Exporting a composable node component executables](#exporting-a-composable-node-component-executables) is generally preferred for standard operational use due its flexibility and scalability within the ROS ecosystem.

Assuming:

- `src/gnss_poser.cpp` has the `GNSSPoser` class.
- `src/gnss_poser_node.cpp` has the `main` function.
- There is no composable node component registration.

```cmake
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/gnss_poser.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_node src/gnss_poser_node.cpp)
```

- The node executable:
  - should have `_node` suffix.
  - should start with `${PROJECT_NAME}`

### `config` and `schema`

```txt
autoware_gnss_poser
│─ config
│   ├─ gnss_poser.param.yaml
│   └─ another_non_ros_config.yaml
└─ schema
    └─ gnss_poser.schema.json
```

#### `config`

- ROS parameters uses the extension `.param.yaml`.
- Non-ROS parameters use the extension `.yaml`.

**Rationale:** Different linting rules are used for ROS parameters and non-ROS parameters.

#### `schema`

Place parameter definition files. See [Parameters](./parameters.md) for details.

### `design`

```txt
autoware_gnss_poser
└─ design
    └─ GnssPoser.node.yaml
```

Place the node description files for [Autoware System Designer](https://github.com/autowarefoundation/autoware_system_designer) here. A `*.node.yaml` file describes one node: its package, plugin, and executable, its input and output topics with their message types and QoS, its parameter files, and its processes. Autoware System Designer reads these files to generate launch files and system diagrams. Most node packages of Autoware Core and Autoware Universe have one, for example [`design/GnssPoser.node.yaml` of `autoware_gnss_poser`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/sensing/autoware_gnss_poser/design/GnssPoser.node.yaml).

### `doc`

```txt
autoware_gnss_poser
└─ doc
    ├─ foo_document.md
    └─ foo_diagram.svg
```

Place documentation files and link them from the README file.

### `include` and `src`

- Unless you specifically need to export headers, you shouldn't have a `include` directory under the package directory.
- For most cases, follow [Not exporting headers](#not-exporting-headers).
- Library packages that export headers may follow [Exporting headers](#exporting-headers).

#### Not exporting headers

```txt
autoware_gnss_poser
└─ src
    ├─ include
    │   ├─ gnss_poser_node.hpp
    │   └─ foo.hpp
    │─ gnss_poser_node.cpp
    └─ bar.cpp

OR

autoware_gnss_poser
└─ src
    ├─ gnss_poser_node.hpp
    ├─ gnss_poser_node.cpp
    ├─ foo.hpp
    └─ bar.cpp
```

- The source file exporting the node should:
  - have `_node` suffix.
    - **Rationale:** To distinguish from other source files.
  - **NOT** have `autoware_` prefix.
    - **Rationale:** To avoid verbosity.
- See [Class design](./class-design.md) for more details on how to construct `gnss_poser_node.hpp` and `gnss_poser_node.cpp` files.
- It is up to developer how to organize the source files under `src`.
  - **Note:** The `include` folder under `src` is optional.

#### Exporting headers

```txt
autoware_gnss_poser
└─ include
    └─ autoware
        └─ gnss_poser
            └─ exported_header.hpp
```

- `autoware_gnss_poser/include` folder should contain **ONLY** the `autoware` folder.
  - **Rationale:** When installing ROS debian packages, the headers are copied to the `/opt/ros/$ROS_DISTRO/include/` directory. This structure is used to avoid conflicts with non-Autoware packages.
- `autoware_gnss_poser/include/autoware` folder should contain **ONLY** the `gnss_poser` folder.
  - **Rationale:** Similarly, this structure is used to avoid conflicts with other packages.
- `autoware_gnss_poser/include/autoware/gnss_poser` folder should contain the header files to be exported.

**Note:** If `ament_auto_package()` command is used in the `CMakeLists.txt` file and `autoware_gnss_poser/include` folder exists,
this `include` folder will be exported to the `install` folder as part of [ament_auto_package.cmake](https://github.com/ament/ament_cmake/blob/79cc237f8eb819edf4c1c624b56451e0a05a45f8/ament_cmake_auto/cmake/ament_auto_package.cmake#L62-L66)

`autoware_ament_auto_package()` exports the `include` folder in the same way. With the `USE_SCOPED_HEADER_INSTALL_DIR` option, it installs the headers in `include/${PROJECT_NAME}/` instead of `include/`. The source code still includes them as `<autoware/gnss_poser/exported_header.hpp>`.

**Reference:** <https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html#adding-targets>

### `launch`

```txt
autoware_gnss_poser
└─ launch
    ├─ gnss_poser.launch.xml
    └─ gnss_poser.launch.py
```

- You may have multiple launch files here.
- Unless you have a specific reason, use the `.launch.xml` extension.
  - **Rationale:** While the `.launch.py` extension is more flexible, it comes with a readability cost.
- Avoid `autoware_` prefix in the launch file names.
  - **Rationale:** To avoid verbosity.
- If the node supports Agnocast, include `agnocast_env.launch.xml` of `autoware_agnocast_wrapper` and give the node the `LD_PRELOAD` value that it sets. If Agnocast is not enabled, this value is the `LD_PRELOAD` of the environment, unchanged:

  ```xml
  <include file="$(find-pkg-share autoware_agnocast_wrapper)/launch/agnocast_env.launch.xml"/>

  <node pkg="autoware_gnss_poser" exec="gnss_poser" name="gnss_poser" output="both">
    ...
    <env name="LD_PRELOAD" value="$(var ld_preload_value)"/>
  </node>
  ```

### `test`

```txt
autoware_gnss_poser
└─ test
    ├─ test_foo.hpp  # or place under an `include` folder here
    └─ test_foo.cpp
```

Place source files for testing. See [unit testing](../../testing-guidelines/unit-testing.md) for details.

## Python package

Most Autoware packages are C++ packages. A few packages are pure Python packages with the `ament_python` build type. We'll use the [`autoware_adapi_visualizers`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/system/autoware_default_adapi_helpers/autoware_adapi_visualizers) package of Autoware Universe as an example.

### Entire structure

```txt
autoware_adapi_visualizers
├─ package.xml
├─ setup.py
├─ setup.cfg
├─ README.md
│
├─ resource
│   └─ autoware_adapi_visualizers
│
├─ autoware_adapi_visualizers
│   ├─ __init__.py
│   └─ planning_factors.py
│
├─ config
│   └─ planning_factors.param.yaml
│
├─ launch
│   └─ autoware_adapi_visualizers.launch.xml
│
└─ test
    ├─ test_copyright.py
    └─ test_pep257.py
```

- The `README.md` file and the `config` folder are not in the real package. They show where these files go.
- The [package name](#package-name) rules are the same as for a C++ package.
- The `config` and `launch` folders follow the same rules as for a C++ package. See [`config` and `schema`](#config-and-schema) and [`launch`](#launch).

### `package.xml`

- Set the build type to `ament_python` in the `<export>` tag:

  ```xml
  <export>
    <build_type>ament_python</build_type>
  </export>
  ```

- Add the dependencies that the Python code imports as `<exec_depend>`, for example `rclpy`.
- Add the test tools as `<test_depend>`, for example `ament_copyright`, `ament_pep257`, and `python3-pytest`.

### Python module folder

- Put the Python code in a folder with the same name as the package, with an `__init__.py` file.
- Give each executable a `main()` function.

### `setup.py`

`setup.py` installs the package. Use the package name as `name`, and list the executables as `console_scripts` entry points:

```python
package_name = "autoware_adapi_visualizers"

setup(
    name=package_name,
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [f"launch/{package_name}.launch.xml"]),
    ],
    entry_points={
        "console_scripts": ["planning_factors = autoware_adapi_visualizers.planning_factors:main"],
    },
    ...
)
```

- `data_files` installs the files that are not Python code. List the `package.xml` file, the files in `resource`, and each launch file and parameter file. A file that is not in this list is not installed.
- Each entry of `console_scripts` makes one executable: `<executable name> = <module>:<function>`.

### `setup.cfg`

`setup.cfg` installs the executables in `lib/<package name>`. `ros2 run` and the `exec` attribute of a launch file look for them there. This example is the `setup.cfg` file of [`autoware_debug_tools`](https://github.com/autowarefoundation/autoware_tools/tree/0.8.0/common/autoware_debug_tools) in Autoware Tools:

```ini
[develop]
script_dir=$base/lib/autoware_debug_tools
[install]
install_scripts=$base/lib/autoware_debug_tools
```

### `resource`

The `resource` folder contains one empty file with the same name as the package. `setup.py` installs it into the ament resource index, so that ROS 2 tools can find the package.

### `test`

Place the test files here. `colcon test` runs them with `pytest`. The `test_copyright.py` and `test_pep257.py` files run the `ament_copyright` and `ament_pep257` linters.

### Python scripts in a C++ package

A C++ package can also install Python scripts, for example tools for debugging. Place them in a `scripts` folder and install them from the `CMakeLists.txt` file:

```cmake
install(PROGRAMS
  scripts/min_turning_radius_calculator.py
  DESTINATION lib/${PROJECT_NAME}
)
```

This example comes from [`autoware_vehicle_info_utils`](https://github.com/autowarefoundation/autoware_core/tree/1.10.0/common/autoware_vehicle_info_utils) of Autoware Core. Start each script with `#!/usr/bin/env python3`. `install(PROGRAMS ...)` installs the script with the permission to execute it.
