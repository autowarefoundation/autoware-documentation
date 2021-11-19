# Integration Testing

## Introduction

This article motivates developers to adopt integration testing by explaining how to write, run,
and evaluate the results of integration tests.

## Quick reference

1. [colcon](https://github.com/ros2/ros2/wiki/Colcon-Tutorial) is used to build and run test.
2. [launch testing](https://github.com/ros2/launch/tree/master/launch_testing) launches nodes and runs tests.
3. [Testing in general](testing-in-general.md) describes the big picture of testing.

## Integration testing

An integration test is defined as the phase in software testing where individual software
modules are combined and tested as a group. Integration tests occur after unit tests, and before
validation tests.

The input to an integration test is a set of independent modules that have been unit tested. The set
of modules are tested against the defined integration test plan, and the output is a set of
properly integrated software modules that are ready for system testing.

## Value of integration testing

Integration tests determine if independently developed software modules work correctly
when the modules are connected to each other. In ROS 2, the software modules are called
nodes. As a special case, testing a single node can be referred to as component testing.

Integration tests help to find the following types of errors:

- Incompatible interaction between nodes, such as non-matching topics, different message types, or incompatible QoS settings
- Reveal edge cases that were not touched with unit tests, such as a critical timing issue, network communication delay, disk I/O failure, and many other problems that can occur in production environments
- Using tools like `stress` and `udpreplay`, performance of nodes is tested with real data or while the system is under high CPU/memory load, where situations such as `malloc` failures can be detected

With ROS 2, it is possible to program complex autonomous-driving applications with a large number
of nodes. Therefore, a lot of effort has been made to provide an integration-test framework that
helps developers test the interaction of ROS2 nodes.

## Integration-test framework

A typical integration-test framework has three parts:

1. A series of executables with arguments that work together and generate outputs
2. A series of expected outputs that should match the output of the executables
3. A launcher that starts the tests, compares the outputs to the expected outputs, and determines if the test passes

In Autoware.Core, we use the [launch_testing](https://github.com/ros2/launch/tree/master/launch_testing) framework.

### Smoke tests

Autoware has dedicated API for smoke testing

To use this framework, in `package.xml` add:

```{xml}
<test_depend>autoware_testing</test_depend>
```

and in `CMakeLists.txt` add:

```{cmake}
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${NODE_NAME})
endif()
```

which adds smoke test that ensures that node can be:

1. launched with default parameter file,
2. terminated with a standard `SIGTERM` signal,

For full API documentation see [package design page](autoware-testing-package-design.md).

This API is not suitable for all smoke test cases. For example, it can not be used when some specific file location,
like map, is required to be passed to the node or some preparation need to be conducted before node launch. In such cases use manual solution from [section below](#integration-test-with-a-single-node-component-test).

### Integration test with a single node: component test

The simplest scenario is a single node. In this case, the integration test is commonly referred to as a component test.

To add a component test to an existing node, follow the example of the `lanelet2_map_provider` package that has an executable named `lanelet2_map_provider_exe`.

In `package.xml`, add

```{xml}
<test_depend>ros_testing</test_depend>
```

In `CMakeLists.txt`, add or modify the `BUILD_TESTING` section:

```{cmake}
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/lanelet2_map_provider_launch.test.py
    TIMEOUT "30"
  )
endif()
```

The `TIMEOUT` argument is given in seconds; see [here](https://github.com/ros2/ros_testing/blob/master/ros_testing/cmake/add_ros_test.cmake) for details.

To create test follow [launch_testing quick-start example](https://github.com/ros2/launch/tree/master/launch_testing#quick-start-example).

Let's look at `test/lanelet2_map_provider_launch.test.py` as an example.

The essential content is to first import dependencies:

```{python}
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing

import os
import pytest
import unittest
```

Then a launch description was created to launch the node under test. Note that `test_map.osm` file path is found and passed to the node. This is one of limitation of [smoke test](#integration-testing-smoke-test):

```{python}
@pytest.mark.launch_test
def generate_test_description():

    map_osm_file = os.path.join(
        get_package_share_directory('lanelet2_map_provider'),
        'data/test_map.osm'
    )

    lanelet2_map_provider = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_provider_exe',
        namespace='had_maps',
        parameters=[
            os.path.join(
                get_package_share_directory('lanelet2_map_provider'),
                'param/test.param.yaml'
            ),
            {
                'map_osm_file': map_osm_file
            }]
    )

    context = {'lanelet2_map_provider': lanelet2_map_provider}

    return LaunchDescription([
        lanelet2_map_provider,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()]
    ), context
```

and finally the test condition. As before, it is just a smoke test ensures the node can be

1. launched with its default parameter file,
2. terminated with a standard `SIGTERM` signal,

so the test code is executed after the node executable has been shut down (`post_shutdown_test`):

```{python}
@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, ndt_mapper):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=ndt_mapper)
```

## Running the test

Continuing the example from above, first build

```{bash}
$ ade enter
ade$ cd AutowareCore
ade$ colcon build --packages-up-to lanelet2_map_provider
ade$ source install/setup.bash
```

then either execute the component test manually

```{bash}
ade$ ros2 test src/mapping/had_map/lanelet2_map_provider/test/lanelet2_map_provider_launch.test.py
```

or as part of testing the entire package:

```{bash}
ade$ colcon test --packages-select lanelet2_map_provider
```

Verify that the test is executed; e.g.

```{bash}
ade$ colcon test-result --all --verbose
...
build/lanelet2_map_provider/test_results/lanelet2_map_provider/test_lanelet2_map_provider_launch.test.py.xunit.xml: 1 test, 0 errors, 0 failures, 0 skipped
```

### Next steps

The simple test described in [Integration test with a single node: component test](integration-test-with-a-single-node-component-test) can be extended in numerous directions:

#### Testing the output of a node

To test while the node is running, create an [_active test_](https://github.com/ros2/launch/tree/foxy/launch_testing#active-tests) by adding a subclass of Python's `unittest.TestCase` to `*launch.test.py`. Some boilerplate code is required to access output by creating a node and a subscription to a particular topic; e.g.

```{python}
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

#### Running multiple nodes together

To run multiple nodes together, simply add more nodes to the launch description in `*launch.test.py`.
The lidar stack has more elaborate examples on how to feed input and to test more than just the exit status of nodes; see [point_cloud_filter_transform_tf_publisher.test.py](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/perception/filters/point_cloud_filter_transform_nodes/test/point_cloud_filter_transform_tf_publisher.test.py) for details.
