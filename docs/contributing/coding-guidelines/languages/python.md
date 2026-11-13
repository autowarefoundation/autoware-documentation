# Python

This page describes the rules for Python code in Autoware. Autoware formats Python code with Black and isort, and checks it with flake8. pre-commit runs all three tools on each pull request, so most rules on this page are automated.

## References

Follow the Python section of the ROS 2 code style when this page does not define a rule:

- <https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#python>
- [PEP 8](https://peps.python.org/pep-0008/)

Autoware is different from the ROS 2 style in one point: Black formats strings with double quotes, and Autoware keeps that format. The ROS 2 style prefers single quotes.

## Python version

Autoware supports ROS 2 Humble on Ubuntu 22.04 and ROS 2 Jazzy on Ubuntu 24.04. The system Python of Ubuntu 22.04 is Python 3.10, and the system Python of Ubuntu 24.04 is Python 3.12. Write code that runs on Python 3.10, so that it runs on both distributions.

## Formatting

### Black

pre-commit formats all Python files with [Black](https://github.com/psf/black). The line length is 100 characters:

```yaml
- repo: https://github.com/psf/black-pre-commit-mirror
  hooks:
    - id: black
      args: [--line-length=100]
```

Do not format the code by hand against Black. If a block must keep a manual layout, for example a matrix, put `# fmt: off` and `# fmt: on` around it.

### isort

pre-commit sorts the imports with [isort](https://github.com/PyCQA/isort). The hook uses the Black profile and a line length of 100. The `[isort]` section of [`setup.cfg`](https://github.com/autowarefoundation/sync-file-templates/blob/main/sources/setup.cfg) adds these options:

- `force_single_line=true`: isort writes one import on each line.
- `force_sort_within_sections=true`: isort sorts `import x` and `from x import y` together by module name.
- `known_third_party=launch`: isort puts the `launch` package in the third-party section.

For example, the `setup.py` of [`autoware_adapi_visualizers`](https://github.com/autowarefoundation/autoware_universe/blob/0.53.1/system/autoware_default_adapi_helpers/autoware_adapi_visualizers/setup.py) starts with:

```python
from warnings import simplefilter

from pkg_resources import PkgResourcesDeprecationWarning
from setuptools import SetuptoolsDeprecationWarning
from setuptools import setup
```

## Linting

pre-commit checks all Python files with the `flake8-ros` hook of [tier4/pre-commit-hooks-ros](https://github.com/tier4/pre-commit-hooks-ros). The hook runs [flake8](https://github.com/PyCQA/flake8) with the plugins that the ROS 2 developer tools use:

- `flake8-blind-except`: finds a bare `except:` that catches all exceptions.
- `flake8-builtins`: finds variables and arguments that hide a Python built-in, such as `id` or `list`.
- `flake8-comprehensions`: finds `list()`, `dict()`, and `set()` calls that a comprehension can replace.
- `flake8-deprecated`: finds calls to deprecated methods.
- `flake8-docstrings`: checks the format of docstrings (PEP 257).
- `flake8-import-order`: checks the order of the imports.
- `flake8-quotes`: checks the quotes of docstrings and multiline strings.

The `[flake8]` section of [`setup.cfg`](https://github.com/autowarefoundation/sync-file-templates/blob/main/sources/setup.cfg) sets the maximum line length to 100 and turns off the checks that conflict with Black, for example `E203`, `E501`, and the quote check `Q000`. It also turns off some checks of the plugins, for example `B902` (`except Exception:`) and `CNL100` (the only check of `flake8-class-newline`, which the hook also installs). The checks for missing docstrings (`D100` to `D107`) are also off. A docstring is not required, but when you write one, flake8 checks its format.

If flake8 reports a false positive, add a `# noqa` comment with the code of the check on that line:

```python
from foo import *  # noqa: F403
```

## Packages

A package that has only Python code uses the `ament_python` build type with `setup.py`, `setup.cfg`, and a `resource/<package_name>` marker file. For an example, see [`autoware_adapi_visualizers`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/system/autoware_default_adapi_helpers/autoware_adapi_visualizers).

A package that has C++ and Python code uses the `ament_cmake` build type and installs the Python module with `ament_python_install_package()` in `CMakeLists.txt`. For an example, see [`autoware_freespace_planning_algorithms`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_freespace_planning_algorithms).

Give Python modules, functions, and variables `snake_case` names, and classes `CamelCase` names, as PEP 8 requires.
