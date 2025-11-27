# Applying Clang-Tidy to ROS packages

[Clang-Tidy](https://clang.llvm.org/extra/clang-tidy/) is a powerful C++ linter.

## Prerequisites

Install `fd-find` to make it easier to feed file lists to `run-clang-tidy`.
More info can be found in the [fd documentation](https://github.com/sharkdp/fd?tab=readme-ov-file#on-ubuntu).

```bash
sudo apt-get install fd-find
```

Install `clang-tidy` and `clang-apply-replacements`.

```bash
sudo apt-get install clang-tidy clang-tools
```

If you encounter `clang-diagnostic-error`, try installing `libomp-dev`.
Related: <https://github.com/autowarefoundation/autoware-github-actions/pull/172>

```bash
sudo apt-get install libomp-dev
```

## Preparation

You need to generate `build/compile_commands.json` before using Clang-Tidy.

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## Usage

If you are on Ubuntu 22.04, run the following command **only once** to help `run-clang-tidy` find the correct include paths.
**Reference:** [autoware/pull/5543](https://github.com/autowarefoundation/autoware/pull/5543#issuecomment-2533325979).

```bash
sed -i '/- -Wno-c11-extensions/a\  - -I/usr/include/c++/11\n  - -I/usr/include/x86_64-linux-gnu/c++/11' .clang-tidy-ci
```

To run Clang-Tidy with all files in a package and export fixes to a YAML file, run the following command from the workspace root.
Here we use `autoware_utils` as an example package name.

```bash
cd autoware/
run-clang-tidy -p build/ -config="$(cat .clang-tidy-ci)" -export-fixes clang-tidy-fixes.yaml -j $(nproc) $(fdfind -e cpp -e hpp --full-path "/autoware_utils/") > clang-tidy-report.log
```

To apply fixes directly, run the following command.

```bash
run-clang-tidy -p build/ -config="$(cat .clang-tidy-ci)" -j $(nproc) -fix $(fdfind -e cpp -e hpp --full-path "/autoware_utils/")
```

`clang-apply-replacements` command can also be used to apply fixes from a YAML file.

## IDE integration

### CLion

Refer to the [CLion Documentation](https://www.jetbrains.com/help/clion/clang-tidy-checks-support.html).

### Visual Studio Code

Use either one of the following extensions:

- [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
- [clangd](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd)
