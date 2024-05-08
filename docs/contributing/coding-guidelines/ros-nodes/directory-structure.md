# Directory structure

This document describes the directory structure of ROS nodes within Autoware.

We'll use the package `autoware_gnss_poser` as an example.

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
- Even if the package is exports a node, the package name **should NOT** have the `_node` suffix.
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

##### Exporting a component

```cmake
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/gnss_poser_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::gnss_poser::GNSSPoserNode"
  EXECUTABLE ${PROJECT_NAME}_node
)
```

- The component executable should have `_node` suffix.
- The component executable name should start with `${PROJECT_NAME}`

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
```

- Put the header files in the `include` directory under the `src` directory.
- The source file exporting the node should:
  - have `_node` suffix.
    - **Rationale:** To distinguish from other source files.
  - **NOT** have `_autoware` prefix.
    - **Rationale:** To avoid verbosity.
- See [Classes](../../class-design.md) for more details on how to construct `gnss_poser_node.hpp` and `gnss_poser_node.cpp` files.

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

### `test`

```txt
autoware_gnss_poser
└─ test
    ├─ test_foo.hpp  # or place under an `include` folder here
    └─ test_foo.cpp
```

Place source files for testing. See [unit testing](../../testing-guidelines/unit-testing.md) for details.

## Python package

!!! warning

    Under Construction
