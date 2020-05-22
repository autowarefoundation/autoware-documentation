Unit Testing {#unit-testing}
========

@tableofcontents

Autoware.Auto uses the `ament_cmake` framework to build and run tests. The same
framework is also used to analyze the test results.

`ament_cmake` provides several convenience functions to make it easy to register tests in a
CMake-based package and to ensure that JUnit-compatible result files are generated. It currently
supports a few different testing frameworks like `pytest`, `gtest`, and `gmock`.

See below for an example of using `ament_cmake_gtest` with `colcon test`. All other tests follow
a similar pattern.

This example assumes that the package `my_cool_pkg` is generated with
[autoware_auto_create_package](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/tree/master/src/tools/autoware_auto_create_pkg).


# Create a unit test with gtest {#unit-testing-create-with-gtest}

In `my_cool_pkg/test`, create the `gtest` code file `test_my_cool_pkg.cpp`:

```{cpp}
#include "gtest/gtest.h"
#include "my_cool_pkg/my_cool_pkg.hpp"
TEST(TestMyCoolPkg, TestHello) {
  EXPECT_EQ(my_cool_pkg::print_hello(), 0);
}
```

For more examples of `gtest` features, see the
[gtest repo](https://github.com/google/googletest).

In `package.xml`, add the following line:

```{xml}
<test_depend>ament_cmake_gtest</test_depend>
```

Next add an entry under `BUILD_TESTING` in the `CMakeLists.txt` to compile the test
source files:

```{cmake}
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${TEST_MY_COOL_PKG} test/test_my_cool_pkg.cpp)

  autoware_set_compile_options(${TEST_MY_COOL_PKG})
  target_include_directories(${TEST_MY_COOL_PKG} PRIVATE "test/include" "include")
  target_link_libraries(${TEST_MY_COOL_PKG} ${PROJECT_NAME}) # adapt
...
endif()
```

This automatically links the test with the default main function provided by `gtest`. The code under test is usually in a different CMake target (`${PROJECT_NAME}` in the example)  and its shared object for linking and include directories need to be added.

To register a new `gtest` item, wrap the test code with the macro `TEST ()`. `TEST ()`
is a predefined macro that helps generate the final test code, and also registers
a `gtest` item to be available for execution.
Test case name should be in CamelCase.
This is as gtest inserts an underscore between the fixture name and the class case name when creating the test executable.

`gtest/gtest.h` also contains predefined macros of `gtest` like `ASSERT_TRUE(condition)`,
`ASSERT_FALSE(condition)`, `ASSERT_EQ(val1,val2)`, `ASSERT_STREQ(str1,str2)`,
`EXPECT_EQ()`, etc. `ASSERT_*` will abort the test if the condition is not
satisfied, while `EXPECT_*` will mark the test as failed but continue to next test
condition. More information about `gtest` can be found in the
[gtest repo](https://github.com/google/googletest).

In the demo `CMakeLists.txt`, `ament_add_gtest` is a predefined macro in `ament_cmake_gtest`
that helps simplify adding `gtest` code. Details can be viewed in
[ament_add_gtest.cmake](https://github.com/ament/ament_cmake/blob/master/ament_cmake_gtest/cmake/ament_add_gtest.cmake).


# Build test {#unit-testing-build-test}

By default, all necessary test files (ELF, CTesttestfile.cmake, etc.) are compiled by `colcon`:

```{bash}
ade$ cd ~/workspace/
ade$ colcon build --packages-select my_cool_pkg
```

Test files are generated under `~/workspace/build/my_cool_pkg`.


# Run test {#unit-testing-run-test}

To run test on a specific package, call:

```{bash}
ade$ colcon test --packages-select my_cool_pkg

Starting >>> my_cool_pkg
Finished <<< my_cool_pkg [7.80s]

Summary: 1 package finished [9.27s]
```

The test command output contains a brief report of all the test results.

To get job-wise information of all executed tests, call:

```{bash}
ade$ colcon test-result --all

build/my_cool_pkg/test_results/my_cool_pkg/copyright.xunit.xml: 8 tests, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/cppcheck.xunit.xml: 6 tests, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/cpplint.xunit.xml: 6 tests, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/lint_cmake.xunit.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/my_cool_pkg_exe_integration_test.xunit.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/pclint.xunit.xml: 0 tests, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/test_my_cool_pkg.gtest.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/uncrustify.xunit.xml: 6 tests, 0 errors, 0 failures, 0 skipped

Summary: 29 tests, 0 errors, 0 failures, 0 skipped
```

Look in the `~/workspace/log/test_<date>/<package_name>` directory for all the raw test
commands, `std_out`, and `std_err`. There's also the `~/workspace/log/latest_*/` directory
containing symbolic links to the most recent package-level build and test output.

To print the tests' details while the tests are being run, use the
`--event-handlers console_cohesion+` option to print the details directly to the console:

```{bash}
ade$ colcon test --event-handlers console_cohesion+ --packages-select my_cool_pkg

...
test 1
    Start 1: test_my_cool_pkg

1: Test command: /usr/bin/python3 "-u" "~/workspace/install/share/ament_cmake_test/cmake/run_test.py" "~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/test_my_cool_pkg.gtest.xml" "--package-name" "my_cool_pkg" "--output-file" "~/workspace/build/my_cool_pkg/ament_cmake_gtest/test_my_cool_pkg.txt" "--command" "~/workspace/build/my_cool_pkg/test_my_cool_pkg" "--gtest_output=xml:~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/test_my_cool_pkg.gtest.xml"
1: Test timeout computed to be: 60
1: -- run_test.py: invoking following command in '~/workspace/src/my_cool_pkg':
1:  - ~/workspace/build/my_cool_pkg/test_my_cool_pkg --gtest_output=xml:~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/test_my_cool_pkg.gtest.xml
1: [==========] Running 1 test from 1 test case.
1: [----------] Global test environment set-up.
1: [----------] 1 test from test_my_cool_pkg
1: [ RUN      ] test_my_cool_pkg.test_hello
1: Hello World
1: [       OK ] test_my_cool_pkg.test_hello (0 ms)
1: [----------] 1 test from test_my_cool_pkg (0 ms total)
1:
1: [----------] Global test environment tear-down
1: [==========] 1 test from 1 test case ran. (0 ms total)
1: [  PASSED  ] 1 test.
1: -- run_test.py: return code 0
1: -- run_test.py: inject classname prefix into gtest result file '~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/test_my_cool_pkg.gtest.xml'
1: -- run_test.py: verify result file '~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/test_my_cool_pkg.gtest.xml'
1/8 Test #1: test_my_cool_pkg ...................   Passed    0.09 sec

...

100% tests passed, 0 tests failed out of 8

Label Time Summary:
copyright      =   0.31 sec (1 test)
cppcheck       =   0.31 sec (1 test)
cpplint        =   0.38 sec (1 test)
gtest          =   0.09 sec (1 test)
integration    =   0.58 sec (1 test)
lint_cmake     =   0.31 sec (1 test)
linter         =   7.23 sec (6 tests)
pclint         =   5.57 sec (1 test)
uncrustify     =   0.35 sec (1 test)

Total Test time (real) =   7.91 sec
...
```


# Coverage  {#unit-testing-coverage}

Loosely described, a coverage metric is a measure of how much of the program code
has been exercised (covered) during testing.

In Autoware.Auto the [lcov tool] (http://ltp.sourceforge.net/documentation/technical_papers/gcov-ols2003.pdf)
(which uses `gcov` internally) is used to measure:

1. Statement coverage
2. Function coverage
3. Branch coverage

`lcov` also collects the results and generates `html` to visualize the coverage information.

Coverage for the latest successful CI run on the `master` branch is
[here](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/coverage/index.html).

Use the commands below to generate coverage information for `my_cool_pkg`:

\note `package_coverage.sh` prompts to delete `build`, `install`, and `log` directories, if present. Answer with `y` to
delete, or clean your build before generating the coverage report.

```{bash}
ade$ cd AutowareAuto
ade$ git lfs install
ade$ git lfs pull --include="*" --exclude=""
ade$ vcs import < autoware.auto.$ROS_DISTRO.repos
ade$ ./tools/coverage/package_coverage.sh my_cool_pkg
ade$ ./tools/coverage/coverage.sh  # coverage of all packages
```

This produces the high-level coverage report and also generates a coverage folder with an `index.html` file in it
assuming the build and tests passed successfully. The resulting `lcov/index.html` will have a similar form to the
following:

@image html images/lcov_result.jpg  "Example lcov output" width=80%

In Autoware.Auto, there is a separate "coverage" job as part of the CI pipeline that measures and reports the test
coverage of a merge request:

@image html images/coverage-test-job.png "coverage test job" width=80%

and the summary statistics are printed near the end of the log output:

@image html images/coverage-ci-output.png "coverage test job summary" width=40%
