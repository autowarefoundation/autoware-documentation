# Coding guidelines

This section describes how to write code for Autoware. The [language-specific guidelines](#language-specific-guidelines) describe the formatters, linters, and conventions for each file type. The [ROS nodes guidelines](./ros-nodes/directory-structure.md) describe how to structure a ROS 2 package and node.

## Common guidelines

Autoware follows the ROS 2 developer guide when this section does not define a rule:

- <https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html>

Also, keep in mind the following concepts.

- Keep things consistent.
- Automate where possible, using simple checks for formatting, syntax, etc.
- Write comments and documentation in English.
- Functions that are too complex (low cohesion) should be appropriately split into smaller functions (e.g. less than 40 lines in one function is recommended in the [google style guideline](https://google.github.io/styleguide/cppguide.html#Write_Short_Functions)).
- Try to minimize the use of member variables or global variables that have a large scope.
- Whenever possible, break large pull requests into smaller, manageable PRs (less than 200 lines of change is recommended in some research e.g. [here](https://opensource.com/article/18/6/anatomy-perfect-pull-request)).
- When it comes to code reviews, don't spend too much time on trivial disagreements. For details see:
  - <https://en.wikipedia.org/wiki/Law_of_triviality>
  - <https://steemit.com/programming/@emrebeyler/code-reviews-and-parkinson-s-law-of-triviality>
- Please follow the guidelines for each language.

## Language-specific guidelines

| Language or file type                           | Main automated checks                                                                      |
| ----------------------------------------------- | ------------------------------------------------------------------------------------------ |
| [C++](./languages/cpp.md)                       | ClangFormat, cpplint, `ros-include-guard`, Clang-Tidy, Cppcheck                            |
| [Python](./languages/python.md)                 | Black, isort, flake8 (`flake8-ros`)                                                        |
| [Shell scripts](./languages/shell-scripts.md)   | ShellCheck, shfmt                                                                          |
| [Markdown](./languages/markdown.md)             | markdownlint, Prettier, CSpell                                                             |
| [CMake](./languages/cmake.md)                   | `ament_cmake_lint_cmake`, compiler warnings set by `autoware_package()`                    |
| [package.xml](./languages/package-xml.md)       | `sort-package-xml`, `prettier-package-xml`, `check-package-depends`, `ament_cmake_xmllint` |
| [Docker](./languages/docker.md)                 | hadolint                                                                                   |
| [GitHub Actions](./languages/github-actions.md) | yamllint, Prettier                                                                         |

## Automated checks

Most of these rules are checked by tools, so you do not have to remember them. Autoware runs the tools in two places.

[pre-commit](https://pre-commit.com/) runs the formatters and linters on the files of a pull request. Most Autoware repositories get their `.pre-commit-config.yaml` through the `sync-files` workflow, which copies it from [autowarefoundation/sync-file-templates](https://github.com/autowarefoundation/sync-file-templates/blob/main/sources/.pre-commit-config.yaml). The configuration files of the tools come from the same repository: `.clang-format`, `CPPLINT.cfg`, `setup.cfg`, `.markdownlint.yaml`, `.prettierrc.yaml`, and `.yamllint.yaml`. Do not change a synced file in your repository. Change it in `sync-file-templates`, and the next sync copies it to all repositories.

[pre-commit.ci](https://pre-commit.ci/) runs pre-commit on every pull request of a public repository and pushes a `style(pre-commit): autofix` commit with the fixes. To get the same result before you push, run pre-commit on your computer:

```bash
pre-commit run -a
```

`colcon test` runs the linters that need a built package. A package that adds `ament_lint_auto` and [`autoware_lint_common`](https://github.com/autowarefoundation/autoware_cmake/tree/main/autoware_lint_common) as test dependencies runs `ament_cmake_copyright`, `ament_cmake_cppcheck`, `ament_cmake_lint_cmake`, and `ament_cmake_xmllint`. `autoware_lint_common` does not include `cpplint`, `flake8`, or `uncrustify`, because pre-commit runs those tools faster and with any version.

For the CI workflows that run these checks on a pull request, see [CI checks](../pull-request-guidelines/ci-checks.md).

## Autoware style guide

For Autoware-specific styles, refer to the following:

- Use the `autoware_` prefix for package names.
  - cf. [Directory structure guideline, Package name](./ros-nodes/directory-structure.md#package-name)
  - cf. [Prefix packages with autoware\_](https://github.com/orgs/autowarefoundation/discussions/4097)
- Add implementations within the `autoware` namespace.
  - cf. [Class design guideline, Namespaces](./ros-nodes/class-design.md#namespaces)
  - cf. [Prefix packages with autoware\_, Option 3:](https://github.com/orgs/autowarefoundation/discussions/4097#discussioncomment-8384169)
- The header files to be exported must be placed in the `PACKAGE_NAME/include/autoware/` directory.
  - cf. [Directory structure guideline, Exporting headers](./ros-nodes/directory-structure.md#exporting-headers)
- In `CMakeLists.txt`, use `autoware_package()`.
  - cf. [CMake guideline](./languages/cmake.md)
  - cf. [autoware_cmake README](https://github.com/autowarefoundation/autoware_cmake/tree/main/autoware_cmake)
