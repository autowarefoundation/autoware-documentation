# Testing in General

This article motivates developers to test code, explains the importance of testing, and details the
testing performed in Autoware.Core.

## Quick reference

1. [colcon](https://github.com/ros2/ros2/wiki/Colcon-Tutorial) is the tool of choice for building
   and running tests
2. [ament_cmake](https://github.com/ament/ament_cmake) is useful to specify tests in CMake
3. Chris Hobbs' [_Embedded Software Development for Safety Critical Systems_](https://www.amazon.com/Embedded-Software-Development-Safety-Critical-Systems/dp/1498726704),
   describes tests necessary for code running in safety critical environments
4. [ISO 26262 standard](https://www.iso.org/standard/51362.html) part 6 prescribes
   how to test code in automotive systems
5. [SQLite](https://www.sqlite.org/testing.html) is a software project that has an impressive and
   thoroughly described testing system

## Importance of testing

Dynamic and static testing methods make Autoware.Core reliable and robust, helping us to
perform anomaly detection and handling that would otherwise be difficult to find.
Through testing in Autoware.Core, we can estimate the number of Heisenbugs, and find
and eliminate [undefined behaviors](https://blog.regehr.org/archives/1520) for
which C and C++ languages are known for.

Dynamic analysis, simply called “testing” as a rule, means executing the code
while looking for errors and failures.

Static analysis means inspecting the code to look for faults. Static analysis is
using a program (instead of a human) to inspect the code for faults.

There are also formal verification methods (see the
[book](https://www.amazon.com/Embedded-Software-Development-Safety-Critical-Systems/dp/1498726704),
Chapter 15); note that the topics will not be covered in this document.

## Testing in Autoware.Core

This section introduces various types of tests that are run both manually and automatically.

### Style / linter tests

Some examples of tools used for style and linting are
[cpplint](https://github.com/google/styleguide/tree/gh-pages/cpplint),
[uncrustify](https://github.com/uncrustify/uncrustify).

Tests using the tools above allow Autoware.Core to follow C and C++ style guides which results
in uniform, easy to read code.

### Static code analysis

The [Cppcheck](https://github.com/danmar/cppcheck) tool is used for applications
written in Autoware.Core.

Static code analysis tools detect the following types of errors:

- API usage errors
- Best practice coding errors
- Buffer overflows
- Build system issues
- Class hierarchy inconsistencies
- Code maintainability issues
- Concurrent data access violations
- Control flow issues
- Cross-site request forgery (CSRF)
- Cross-site scripting (XSS)
- Deadlocks
- Error handling issues
- Hard-coded credentials
- Incorrect expression
- Insecure data handling
- Integer handling issues
- Integer overflows
- Memory—corruptions
- Memory—illegal accesses
- Null pointer dereferences
- Path manipulation
- Performance inefficiencies
- Program hangs
- Race conditions
- Resource leaks
- Rule violations
- Security best practices violations
- Security misconfigurations
- SQL injection
- Uninitialized members

### Unit tests

Unit-testing is a software testing method by which individual units of source code
are tested to determine whether they are fit for use.

The tool used for unit testing in Autoware.Core is [gtest](https://github.com/google/googletest).

### Integration tests

In integration-testing, the individual software modules are combined and tested as a group.
Integration testing occurs after unit testing.

While performing integration testing, the following subtypes of tests are written:

1. Fault injection testing
2. Back-to-back comparison between a model and code
3. Requirements-based testing
4. Anomaly detection during integration testing
5. Random input testing

### Memory tests

Memory tests allow the detection of unwanted calls to memory management APIs, such as:

- `malloc`
- `calloc`
- `realloc`
- `free`

For more details on memory tests see the
[memory testing](https://github.com/osrf/osrf_testing_tools_cpp#memory_tools) tool.

### In-vehicle tests

Regular in-vehicle testing is performed as part of ODD development and demonstrations. These tests
validate Autoware.Core in a realistic autonomous vehicle product.
