# Directory structure

!!! warning

    Under Construction

## C++ package

```txt
<package_name>
├─ config
│   ├─ foo_ros.param.yaml
│   └─ foo_non_ros.yaml
├─ doc
│   ├─ foo_document.md
│   └─ foo_diagram.svg
├─ include
│   └─ <package_name>
│       └─ foo_public.hpp
├─ launch
│   ├─ foo.launch.xml
│   └─ foo.launch.py
├─ schema
│   └─ foo_node.schema.json
├─ src
│   ├─ foo_node.cpp
│   ├─ foo_node.hpp
│   └─ foo_private.hpp
├─ test
│   └─ test_foo.cpp
├─ package.xml
├─ CMakeLists.txt
└─ README.md
```

### Config directory

Place configuration files such as node parameters.
For ROS parameters, use the extension `.param.yaml`.
For non-ROS parameters, use the extension `.yaml`.

Rationale: Since ROS parameters files are type-sensitive, they should not be the target of some code formatters and linters. In order to distinguish the file type, we use different file extensions.

### Doc directory

Place document files and link from README.

### Include directory

Place header files exposed to other packages. Do not place files directly under the `include` directory, but place files under the directory with the package name.
This directory is used for mostly library headers. Note that many headers do not need to be placed here. It is enough to place the headers under the `src` directory.

Reference: <https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html#adding-files-and-headers>

### Launch directory

Place launch files (`.launch.xml` and `.launch.py`).

### Schema directory

Place parameter definition files. See [parameters](./parameters.md) for details.

### Src directory

Place source files and private header files.

### Test directory

Place source files for testing.

## Python package

T.B.D.
