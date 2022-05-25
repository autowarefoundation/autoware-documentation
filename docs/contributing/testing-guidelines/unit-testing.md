# Unit Testing

Autoware.Auto uses the `ament_cmake` framework to build and run tests. The same
framework is also used to analyze the test results.

`ament_cmake` provides several convenience functions to make it easy to register tests in a
CMake-based package and to ensure that JUnit-compatible result files are generated. It currently
supports a few different testing frameworks like `pytest`, `gtest`, and `gmock`.

In order to prevent tests running in parallel from interfering with each other when publishing and subscribing to ROS topics,
it is recommended to use commands from [`ament_cmake_ros`](https://github.com/ros2/ament_cmake_ros/tree/master/ament_cmake_ros/cmake) to run tests in isolation.

See below for an example of using `ament_add_ros_isolated_gtest` with `colcon test`.
All other tests follow a similar pattern.

## Create a unit test with gtest

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
<test_depend>ament_cmake_ros</test_depend>
```

Next add an entry under `BUILD_TESTING` in the `CMakeLists.txt` to compile the test
source files:

```{cmake}
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(${TEST_MY_COOL_PKG} test/test_my_cool_pkg.cpp)

  target_link_libraries(${TEST_MY_COOL_PKG} ${PROJECT_NAME})
...
endif()
```

This automatically links the test with the default main function provided by `gtest`. The code under test is usually in a different CMake target (`${PROJECT_NAME}` in the example) and its shared object for linking need to be added.

To register a new `gtest` item, wrap the test code with the macro `TEST ()`. `TEST ()`
is a predefined macro that helps generate the final test code, and also registers
a `gtest` item to be available for execution.
The test case name should be in CamelCase, since gtest inserts an underscore between the fixture name and the class case name when creating the test executable.

`gtest/gtest.h` also contains predefined macros of `gtest` like `ASSERT_TRUE(condition)`,
`ASSERT_FALSE(condition)`, `ASSERT_EQ(val1,val2)`, `ASSERT_STREQ(str1,str2)`,
`EXPECT_EQ()`, etc. `ASSERT_*` will abort the test if the condition is not
satisfied, while `EXPECT_*` will mark the test as failed but continue to next test
condition. More information about `gtest` can be found in the
[gtest repo](https://github.com/google/googletest).

In the demo `CMakeLists.txt`, `ament_add_ros_isolated_gtest` is a predefined macro in `ament_cmake_ros`
that helps simplify adding `gtest` code. Details can be viewed in
[ament_add_gtest.cmake](https://github.com/ros2/ament_cmake_ros/tree/master/ament_cmake_ros/cmake).

## Build test

By default, all necessary test files (`ELF`, `CTesttestfile.cmake`, etc.) are compiled by `colcon`:

```{bash}
cd ~/workspace/
colcon build --packages-select my_cool_pkg
```

Test files are generated under `~/workspace/build/my_cool_pkg`.

## Run test

To run all tests for a specific package, call:

```{bash}
colcon test --packages-select my_cool_pkg

Starting >>> my_cool_pkg
Finished <<< my_cool_pkg [7.80s]

Summary: 1 package finished [9.27s]
```

The test command output contains a brief report of all the test results.

To get job-wise information of all executed tests, call:

```{bash}
colcon test-result --all

build/my_cool_pkg/test_results/my_cool_pkg/copyright.xunit.xml: 8 tests, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/cppcheck.xunit.xml: 6 tests, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/lint_cmake.xunit.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/my_cool_pkg_exe_integration_test.xunit.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/test_my_cool_pkg.gtest.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/xmllint.xunit.xml: 1 test, 0 errors, 0 failures, 0 skipped

Summary: 18 tests, 0 errors, 0 failures, 0 skipped
```

Look in the `~/workspace/log/test_<date>/<package_name>` directory for all the raw test
commands, `std_out`, and `std_err`. There's also the `~/workspace/log/latest_*/` directory
containing symbolic links to the most recent package-level build and test output.

To print the tests' details while the tests are being run, use the
`--event-handlers console_cohesion+` option to print the details directly to the console:

```{bash}
colcon test --event-handlers console_cohesion+ --packages-select my_cool_pkg

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
1/5 Test #1: test_my_cool_pkg ...................   Passed    0.09 sec

...

100% tests passed, 0 tests failed out of 5

Label Time Summary:
copyright     =   0.49 sec*proc (1 test)
cppcheck      =   0.20 sec*proc (1 test)
gtest         =   0.05 sec*proc (1 test)
lint_cmake    =   0.18 sec*proc (1 test)
linter        =   1.34 sec*proc (4 tests)
xmllint       =   0.47 sec*proc (1 test)

Total Test time (real) =   7.91 sec
...
```

## Coverage

Loosely described, a coverage metric is a measure of how much of the program code
has been exercised (covered) during testing.

In the autoware.universe repository, [Codecov](https://app.codecov.io/gh/autowarefoundation/autoware.universe/) is used to automatically calculate coverage of any open pull request.

More details about the coverage metrics can be found in the [Codecov documentation](https://docs.codecov.com/docs/about-code-coverage).
