# Directory structure

!!! warning

    Under Construction

## C++ package

```txt
<package-name>
├─ config
├─ include
│   └─ <package-name>
├─ launch
├─ src
├─ test
├─ package.xml
└─ CMakeLists.txt
```

### config directory

Place configuration files such as node parameters.

### include directory

Place header files exposed to other packages under a directory with the package name. Do not place files directly under include directory.
This is mostly library headers and such. Note that many headers do not need to be placed here.

Reference: <https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html#adding-files-and-headers>

### launch directory

Place launch files.

### src directory

Place source and private header files.

### test directory

Place source files for testing.

## Python package

T.B.D.
