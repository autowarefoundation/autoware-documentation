# Applying Clang-Tidy to ROS packages

[Clang-Tidy](https://clang.llvm.org/extra/clang-tidy/) is a powerful C++ linter.

## Preparation

You need to generate `build/compile_commands.json` before using Clang-Tidy.

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## Usage

```bash
clang-tidy -p build/ path/to/file1 path/to/file2 ...
```

If you want to apply Clang-Tidy to all files in a package, using the [fd](https://github.com/sharkdp/fd) command is useful.
To install `fd`, see the [installation manual](https://github.com/sharkdp/fd#on-ubuntu).

```bash
clang-tidy -p build/ $(fd -e cpp -e hpp --full-path "/autoware_utils/")
```

## IDE integration

### CLion

Refer to the [CLion Documentation](https://www.jetbrains.com/help/clion/clang-tidy-checks-support.html).

### Visual Studio Code

Use either one of the following extensions:

- [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
- [clangd](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd)

## Troubleshooting

If you encounter `clang-diagnostic-error`, try installing `libomp-dev`:

Related: <https://github.com/autowarefoundation/autoware-github-actions/pull/172>
