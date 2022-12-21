# Console logging

ROS 2 logging is a powerful tool for understanding and debugging ROS nodes.

This page focuses on how to design console logging in Autoware and shows several practical examples.
To comprehensively understand how ROS 2 logging works, refer to the [logging documentation](https://docs.ros.org/en/humble/Concepts/About-Logging.html).

## Logging use cases in Autoware

- Developers debug code by seeing the console logs.
- Vehicle operators take appropriate risk-avoiding actions depending on the console logs.
- Log analysts analyze the console logs that are recorded in rosbag files.

To efficiently support these use cases, clean and highly visible logs are required.
For that, several rules are defined below.

## Rules

### Choose appropriate severity levels (required, non-automated)

#### Rationale

It's confusing if severity levels are inappropriate as follows:

- Useless messages are marked as `FATAL`.
- Very important error messages are marked as `INFO`.

#### Example

Use the following criteria as a reference:

- **DEBUG:** Use this level to show debug information for developers. Note that logs with this level is hidden by default.
- **INFO:** Use this level to notify events (cyclic notifications during initialization, state changes, service responses, etc.) to operators.
- **WARN:** Use this level when a node can continue working correctly, but unintended behaviors might happen.
- **ERROR:** Use this level when a node can't continue working correctly, and unintended behaviors would happen.
- **FATAL:** Use this level when the entire system can't continue working correctly, and the system must be stopped.

### Filter out unnecessary logs by setting logging options (required, non-automated)

#### Rationale

Some third-party nodes such as drivers may not follow the Autoware's guidelines.
If the logs are noisy, unnecessary logs should be filtered out.

#### Example

Use the `--log-level {level}` option to change the minimum level of logs to be displayed:

```xml
<launch>
  <!-- This outputs only FATAL level logs. -->
  <node pkg="demo_nodes_cpp" exec="talker" ros_args="--log-level fatal" />
</launch>
```

If you want to disable only specific output targets, use the `--disable-stdout-logs`, `--disable-rosout-logs`, and/or `--disable-external-lib-logs` options:

```xml
<launch>
  <!-- This outputs to rosout and disk. -->
  <node pkg="demo_nodes_cpp" exec="talker" ros_args="--disable-stdout-logs" />
</launch>
```

```xml
<launch>
  <!-- This outputs to stdout. -->
  <node pkg="demo_nodes_cpp" exec="talker" ros_args="--disable-rosout-logs --disable-external-lib-logs" />
</launch>
```

### Use throttled logging when the log is unnecessarily shown repeatedly (required, non-automated)

#### Rationale

If tons of logs are shown on the console, people miss important message.

#### Example

While waiting for some messages, throttled logs are usually enough.
In such cases, wait about 5 seconds as a reference value.

```cpp
// Compliant
void FooNode::on_timer() {
  if (!current_pose_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for current_pose_.");
    return;
  }
}

// Non-compliant
void FooNode::on_timer() {
  if (!current_pose_) {
    RCLCPP_ERROR(get_logger(), "Waiting for current_pose_.");
    return;
  }
}
```

#### Exception

The following cases are acceptable even if it's not throttled.

- The message is really worth displaying every time.
- The message level is DEBUG.

### Do not depend on rclcpp::Node in core library classes but depend only on rclcpp/logging.hpp (advisory, non-automated)

#### Rationale

Core library classes, which contain reusable algorithms, may also be used for non-ROS platforms.
When porting libraries to other platforms, fewer dependencies are preferred.

#### Example

```cpp
// Compliant
#include <rclcpp/logging.hpp>

class FooCore {
public:
  explicit FooCore(const rclcpp::Logger & logger) : logger_(logger) {}

  void process() {
    RCLCPP_INFO(logger_, "message");
  }

private:
  rclcpp::Logger logger_;
};

// Compliant
// Note that logs aren't published to `/rosout` if the logger name is different from the node name.
#include <rclcpp/logging.hpp>

class FooCore {
  void process() {
    RCLCPP_INFO(rclcpp::get_logger("foo_core_logger"), "message");
  }
};


// Non-compliant
#include <rclcpp/node.hpp>

class FooCore {
public:
  explicit FooCore(const rclcpp::NodeOptions & node_options) : node_("foo_core_node", node_options) {}

  void process() {
    RCLCPP_INFO(node_.get_logger(), "message");
  }

private:
  rclcpp::Node node_;
};
```

## Tips

### Use rqt_console to filter logs

To filter logs, using `rqt_console` is useful:

```bash
ros2 run rqt_console rqt_console
```

For more details, refer to [ROS 2 Documentation](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html).

### Useful marco expressions

To debug program, sometimes you need see which functions and lines of code are executed.
In that case, use `__LINE__` and `__FUNCTION__` macro:

```cpp
RCLCPP_DEBUG_STREAM(this->get_logger(), "executed line: "<< __LINE__<<" in function: " <<__FUNCTION__);
```

The example output is as follows:

> [DEBUG] [planner]: executed line 100 in function make_plan
