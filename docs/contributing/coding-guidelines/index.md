# Coding guidelines

!!! warning

    Under Construction

## Common guidelines

Refer to the following links for now:

- <https://docs.ros.org/en/humble/Contributing/Developer-Guide.html>

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
  - [C++](./languages/cpp.md)
  - [Python](./languages/python.md)
  - [Shell script](./languages/shell-scripts.md)

## Autoware Style Guide

For Autoware-specific styles, refer to the following:

- Use the `autoware_` prefix for package names.
  - cf. [Prefix packages with autoware\_](https://github.com/orgs/autowarefoundation/discussions/4097)
- Add implementations within the `autoware` namespace.
  - cf. [Prefix packages with autoware\_, Option 3:](https://github.com/orgs/autowarefoundation/discussions/4097#discussioncomment-8384169)
- The header files to be exported must be placed in the `PACKAGE_NAME/include/autoware/` directory.
  - cf. [Directory structure guideline, Exporting headers](./ros-nodes/directory-structure.md#exporting-headers)
- In `CMakeLists.txt`, use `autoware_package()`.
  - cf. [autoware_cmake README](https://github.com/autowarefoundation/autoware_cmake/tree/main/autoware_cmake)
