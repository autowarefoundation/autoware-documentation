# Task scheduling

This page describes how a node in Autoware controls when its work runs. It covers timers, callback groups, and executors.

For basic information, see [Executors](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html) and [Using callback groups](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html) in the ROS 2 documentation.

## Overview

A node does its work in callbacks:

- A timer callback runs at a fixed period.
- A subscription callback runs when a message arrives.
- A service callback runs when a request arrives.

An executor calls these callbacks. A callback group tells the executor which callbacks can run at the same time.

## Run the main processing from a timer

Many nodes of Autoware, for example in planning, control, and localization, run their main logic at a fixed rate, in a timer callback. The callback takes the newest input messages with polling subscribers, computes the output, and publishes it. The [topic message handling guideline](./topic-message-handling/index.md) explains the polling subscribers.

The `autoware_simple_pure_pursuit` node of Autoware Core is a short example. Its constructor creates a timer with a period of 30 ms:

```cpp
timer_ = rclcpp::create_timer(
  this, get_clock(), 30ms, std::bind(&SimplePurePursuitNode::on_timer, this));
```

The timer callback takes the inputs, calls the algorithm, and publishes the result:

```cpp
void SimplePurePursuitNode::on_timer()
{
  // 1. Subscribe data
  const auto odom_ptr = odom_sub_.take_data();
  const auto traj_ptr = traj_sub_.take_data();
  if (!odom_ptr || !traj_ptr) {
    return;
  }
  ...
  // 4. Delegate to core logic
  const auto control_command = core_logic_->create_control_command(odom, traj);
  ...
  // 6. Publish control command
  pub_control_command_->publish(control_command);
}
```

If an input is not available yet, the callback returns and tries again in the next period. To report this, use throttled logging. See [Console logging](./console-logging.md).

### Use the clock of the node

Create a timer with `rclcpp::create_timer(this, get_clock(), period, callback)`. This timer uses the clock of the node. When the `use_sim_time` parameter is `true`, the clock of the node follows the `/clock` topic of a simulator or of `ros2 bag play --clock`. As a result, the node runs at the correct rate in the simulators and with recorded data.

`create_wall_timer()` always uses the steady clock and ignores `use_sim_time`. Use it only for work that must follow the real time, also in a simulation. Most nodes in Autoware Core and Autoware Universe use `rclcpp::create_timer()`.

### Timers in nodes that support Agnocast

`autoware::agnocast_wrapper::Node` (see [Class design](./class-design.md#nodes-that-support-agnocast)) has no `create_timer()` member. Use the free function `autoware::agnocast_wrapper::create_timer()` with the same arguments, and store the timer as `AUTOWARE_TIMER_PTR`. This example comes from `autoware_ekf_localizer`:

```cpp
timer_control_ = autoware::agnocast_wrapper::create_timer(
  this, get_clock(), rclcpp::Duration::from_seconds(params_.ekf_dt),
  std::bind(&EKFLocalizerNode::timer_callback, this));
```

### Make the period a parameter

If the rate of a node can change between vehicles or use cases, read the period from a parameter. For example, `autoware_path_generator` reads its rate from the `planning_hz` parameter:

```cpp
timer_ = rclcpp::create_timer(
  this, get_clock(), rclcpp::Rate(params.planning_hz).period(),
  std::bind(&PathGenerator::run, this));
```

Add the parameter to the parameter file and to the schema file. See [Parameters](./parameters.md).

## Use callback groups for callbacks that run in parallel

All callbacks of a node are in the default callback group, unless you give them another group. The default group is mutually exclusive: the executor runs only one of its callbacks at a time, also with a multi-threaded executor. As a result, a long callback delays all the other callbacks of the node.

If a callback must run while another callback is busy, do these steps:

1. Put the callbacks in different callback groups.
2. Run the node with a multi-threaded executor. See [Select the executor](#select-the-executor).

For example, `autoware_ndt_scan_matcher` of Autoware Core creates three mutually exclusive callback groups. The map update timer, the initial pose subscription, and the sensor points subscription each get one:

```cpp
timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group =
  this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
rclcpp::CallbackGroup::SharedPtr sensor_callback_group =
  this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

AUTOWARE_SUBSCRIPTION_OPTIONS initial_pose_sub_opt;
initial_pose_sub_opt.callback_group = initial_pose_callback_group;
AUTOWARE_SUBSCRIPTION_OPTIONS sensor_sub_opt;
sensor_sub_opt.callback_group = sensor_callback_group;
```

The NDT alignment in the sensor points callback can take a long time. Because of the separate groups, the node still receives the initial poses during the alignment.

Follow these rules:

- Use `rclcpp::CallbackGroupType::MutuallyExclusive` groups. Autoware Core and Autoware Universe use them in almost all cases.
- Protect the data that callbacks of different groups share, for example with `std::mutex`. `autoware_ndt_scan_matcher` keeps such data in a `Guarded` object, which gives access to the value only while it holds a mutex lock.
- A callback group with the second argument of `create_callback_group()` set to `false` is not added to the executor of the node. The executor does not run its callbacks. The polling subscribers use such a group. See [Prevent calling a callback function](./topic-message-handling/index.md#prevent-calling-a-callback-function).

### Do not wait for a response in the group that receives it

A callback that waits for a service response blocks its callback group until the response arrives. If the response must come through the same callback group, it never arrives, and the node stops. This is a deadlock.

`autoware_pose_initializer` of Autoware Core avoids it. Its service callback runs in its own callback group, `group_srv_`. In this callback, the node sends a request to the NDT align service and waits for the response with `async_send_request(req).get()`. The response arrives in the default callback group, and the node runs with a multi-threaded executor. As a result, the response is not blocked.

For more information, see [Using callback groups](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html) in the ROS 2 documentation.

## Select the executor

The executor of a standalone node executable is set in the `CMakeLists.txt` file:

- `rclcpp_components_register_node()` creates an executable with a single-threaded executor. To get a multi-threaded executor, add `EXECUTOR MultiThreadedExecutor`. `autoware_pose_estimator_arbiter` of Autoware Universe is an example:

  ```cmake
  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "autoware::pose_estimator_arbiter::PoseEstimatorArbiter"
    EXECUTABLE ${PROJECT_NAME}_node
    EXECUTOR MultiThreadedExecutor
  )
  ```

- `autoware_agnocast_wrapper_register_node()` has two arguments. `ROS2_EXECUTOR` selects the executor when Agnocast is not used. The default is `SingleThreadedExecutor`. `AGNOCAST_EXECUTOR` selects the executor when Agnocast is used. The default is `SingleThreadedAgnocastExecutor`. `autoware_ndt_scan_matcher` uses:

  ```cmake
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ```

  An executor whose name starts with `AgnocastOnly` requires a node that inherits from `autoware::agnocast_wrapper::Node`. The opposite is also true: when Agnocast is enabled, a node that inherits from `autoware::agnocast_wrapper::Node` stops at construction unless `AGNOCAST_EXECUTOR` is an `AgnocastOnly` executor. Autoware Core and Autoware Universe use `AgnocastOnlyCallbackIsolatedExecutor` for these nodes. For the list of the executors, see the [README of `autoware_agnocast_wrapper`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/common/autoware_agnocast_wrapper/README.md#registering-a-node-with-autoware_agnocast_wrapper_register_node).

When a launch file loads the node into a component container, the executor of the container runs the node, and the setting in `CMakeLists.txt` has no effect. `component_container` has a single-threaded executor, and `component_container_mt` has a multi-threaded executor. `agnocast_env.launch.xml` of `autoware_agnocast_wrapper` selects the container executable from its `use_multithread` and `use_agnocast` arguments. The default of `use_agnocast` is the value of the `ENABLE_AGNOCAST` environment variable.

When Agnocast is enabled, a node that inherits from `autoware::agnocast_wrapper::Node` cannot be loaded into a component container. It runs as its own process, with the executor that `CMakeLists.txt` selects.
