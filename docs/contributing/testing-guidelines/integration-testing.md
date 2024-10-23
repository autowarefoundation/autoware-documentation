# Integration testing

An integration test is defined as the phase in software testing where individual software modules are combined and tested as a group.
Integration tests occur after unit tests, and before validation tests.

The input to an integration test is a set of independent modules that have been unit tested.
The set of modules is tested against the defined integration test plan,
and the output is a set of properly integrated software modules that is ready for system testing.

## Value of integration testing

Integration tests determine if independently developed software modules work correctly when the modules are connected to each other.
In ROS 2, the software modules are called nodes.
Testing a single node is a special type of integration test that is commonly referred to as component testing.

Integration tests help to find the following types of errors:

- Incompatible interactions between nodes, such as non-matching topics, different message types, or incompatible QoS settings.
- Edge cases that were not touched by unit testing, such as a critical timing issue, network communication delays, disk I/O failures, and other such problems that can occur in production environments.
- Issues that can occur while the system is under high CPU/memory load, such as `malloc` failures. This can be tested using tools like `stress` and `udpreplay` to test the performance of nodes with real data.

With ROS 2, it is possible to program complex autonomous-driving applications with a large number of nodes.
Therefore, a lot of effort has been made to provide an integration-test framework that helps developers test the interaction of ROS 2 nodes.

## Integration-test framework

A typical integration-test framework has three parts:

1. A series of executables with arguments that work together and generate outputs.
2. A series of expected outputs that should match the output of the executables.
3. A launcher that starts the tests, compares the outputs to the expected outputs, and determines if the test passes.

In Autoware, we use the [launch_testing](https://github.com/ros2/launch/tree/master/launch_testing) framework.

### Smoke tests

Autoware has a dedicated API for smoke testing.
To use this framework, in `package.xml` add:

```xml
<test_depend>autoware_testing</test_depend>
```

And in `CMakeLists.txt` add:

```cmake
if(BUILD_TESTING)
  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${NODE_NAME})
endif()
```

Doing so adds smoke tests that ensure that a node can be:

1. Launched with a default parameter file.
2. Terminated with a standard `SIGTERM` signal.

For the full API documentation,
refer to the [package design page](https://github.com/autowarefoundation/autoware.universe/blob/main/common/autoware_testing/design/autoware_testing-design.md).

!!! note

    This API is not suitable for all smoke test cases.
    It cannot be used when a specific file location (eg: for a map) is required to be passed to the node, or if some preparation needs to be conducted before node launch.
    In such cases use the manual solution from the [component test section below](#integration-test-with-a-single-node-component-test).

### Integration test with a single node: component test

The simplest scenario is a single node.
In this case, the integration test is commonly referred to as a component test.

To add a component test to an existing node,
you can follow the example of the `lanelet2_map_loader` in the [`autoware_map_loader` package](https://github.com/autowarefoundation/autoware.universe/tree/main/map/autoware_map_loader)
(added in [this PR](https://github.com/autowarefoundation/autoware.universe/pull/1056)).

In [`package.xml`](https://github.com/autowarefoundation/autoware.universe/blob/main/map/autoware_map_loader/package.xml), add:

```xml
<test_depend>ros_testing</test_depend>
```

In [`CMakeLists.txt`](https://github.com/autowarefoundation/autoware.universe/blob/main/map/autoware_map_loader/CMakeLists.txt),
add or modify the `BUILD_TESTING` section:

```cmake
if(BUILD_TESTING)
  add_ros_test(
    test/lanelet2_map_loader_launch.test.py
    TIMEOUT "30"
  )
  install(DIRECTORY
    test/data/
    DESTINATION share/${PROJECT_NAME}/test/data/
  )
endif()
```

In addition to the command `add_ros_test`, we also install any data that is required by the test using the `install` command.

!!! note

    - The `TIMEOUT` argument is given in seconds; see the [add_ros_test.cmake file](https://github.com/ros2/ros_testing/blob/master/ros_testing/cmake/add_ros_test.cmake) for details.
    - The `add_ros_test` command will run the test in a unique `ROS_DOMAIN_ID` which avoids interference between tests running in parallel.

To create a test,
either read the [launch_testing quick-start example](https://github.com/ros2/launch/tree/master/launch_testing#quick-start-example),
or follow the steps below.

Taking [`test/lanelet2_map_loader_launch.test.py`](https://github.com/autowarefoundation/autoware.universe/blob/main/map/autoware_map_loader/test/lanelet2_map_loader_launch.test.py) as an example,
first dependencies are imported:

```python
import os
import unittest

from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
```

Then a launch description is created to launch the node under test.
Note that the [`test_map.osm`](https://github.com/autowarefoundation/autoware.universe/blob/main/map/autoware_map_loader/test/data/test_map.osm) file path is found and passed to the node,
something that cannot be done with the [smoke testing API](#smoke-tests):

```python
@pytest.mark.launch_test
def generate_test_description():

    lanelet2_map_path = os.path.join(
        get_package_share_directory("autoware_map_loader"), "test/data/test_map.osm"
    )

    lanelet2_map_loader = Node(
        package="autoware_map_loader",
        executable="autoware_lanelet2_map_loader",
        parameters=[{"lanelet2_map_path": lanelet2_map_path}],
    )

    context = {}

    return (
        LaunchDescription(
            [
                lanelet2_map_loader,
                # Start test after 1s - gives time for the map_loader to finish initialization
                launch.actions.TimerAction(
                    period=1.0, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ]
        ),
        context,
    )
```

!!! note

    - Since the node need time to process the input lanelet2 map, we use a `TimerAction` to delay the start of the test by 1s.
    - In the example above, the `context` is empty but it can be used to pass objects to the test cases.
    - You can find an example of using the `context` in the [ROS 2 context_launch_test.py](https://github.com/ros2/launch/blob/humble/launch_testing/test/launch_testing/examples/context_launch_test.py) test example.

Finally, a test is executed after the node executable has been shut down (`post_shutdown_test`).
Here we ensure that the node was launched without error and exited cleanly.

```python
@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
```

## Running the test

Continuing the example from above, first build your package:

```console
colcon build --packages-up-to autoware_map_loader
source install/setup.bash
```

Then either execute the component test manually:

```console
ros2 test src/universe/autoware.universe/map/autoware_map_loader/test/lanelet2_map_loader_launch.test.py
```

Or as part of testing the entire package:

```console
colcon test --packages-select autoware_map_loader
```

Verify that the test is executed; e.g.

```console
$ colcon test-result --all --verbose
...
build/autoware_map_loader/test_results/autoware_map_loader/test_lanelet2_map_loader_launch.test.py.xunit.xml: 1 test, 0 errors, 0 failures, 0 skipped
```

### Next steps

The simple test described in [Integration test with a single node: component test](#integration-test-with-a-single-node-component-test) can be extended in numerous directions, such as testing a node's output.

#### Testing the output of a node

To test while the node is running,
create an [_active test_](https://github.com/ros2/launch/tree/foxy/launch_testing#active-tests) by adding a subclass of Python's `unittest.TestCase` to `*launch.test.py`.
Some boilerplate code is required to access output by creating a node and a subscription to a particular topic, e.g.

```python
import unittest

class TestRunningDataPublisher(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node("test_node", context=cls.context)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown(context=cls.context)

    def setUp(self):
        self.msgs = []
        sub = self.node.create_subscription(
            msg_type=my_msg_type,
            topic="/info_test",
            callback=self._msg_received
        )
        self.addCleanup(self.node.destroy_subscription, sub)

    def _msg_received(self, msg):
        # Callback for ROS 2 subscriber used in the test
        self.msgs.append(msg)

    def get_message(self):
        startlen = len(self.msgs)

        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        executor.add_node(self.node)

        try:
            # Try up to 60 s to receive messages
            end_time = time.time() + 60.0
            while time.time() < end_time:
                executor.spin_once(timeout_sec=0.1)
                if startlen != len(self.msgs):
                    break

            self.assertNotEqual(startlen, len(self.msgs))
            return self.msgs[-1]
        finally:
            executor.remove_node(self.node)

    def test_message_content():
        msg = self.get_message()
        self.assertEqual(msg, "Hello, world")
```

## References

- [colcon](https://github.com/ros2/ros2/wiki/Colcon-Tutorial) is used to build and run tests.
- [launch testing](https://github.com/ros2/launch/tree/master/launch_testing) launches nodes and runs tests.
- [Testing guidelines](index.md) describes the different types of tests performed in Autoware and links to the corresponding guidelines.
