# Task scheduling

## Scheduing in autoware

The software of autoware is the system of multiple nodes.In ROS2 the executor uses one or more threads of the underlying operating system to invoke the callbacks. The different type of callback has the different priority.  
In autoware,there are two types about "publish-subscribe":

- Timer callback
- Subscription callback

Timer callback has the high priority.For those with the same priority, the execution order is determined according to the registration order.  
For example about timer callback:

```C++
    const auto planning_hz = declare_parameter("planning_hz", 10.0);
    const auto period_ns = rclcpp::Rate(planning_hz).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&BehaviorPathPlannerNode::run, this));
```

For example about subscription callback:

```C++
  route_subscriber_ = create_subscription<LaneletRoute>(
    "~/input/route", qos_transient_local, std::bind(&BehaviorPathPlannerNode::onRoute, this, _1),
    createSubscriptionOptions(this));
```

In autoware,you can use the create_callback_group() to organizing the callbacks of a node in groups.  
For example:

```C++
  rclcpp::CallbackGroup::SharedPtr callback_group =
    node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
```

The type of MutuallyExclusive ensure that callbacks of this group must not be executed in parallel.You can see more information about scheduling of ROS2 in theses links:

- <https://docs.ros.org/en/humble/Concepts/About-Executors.html>
- <https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html>

## Analyse of data flow and running time

In autoware,there are many topics used by different nodes.You can analyse the data flow and running time of multiple nodes in the whole software. By the analyse,you can intuitively understand the nodes operation path and input/output flow.You can use CARET or TILDE of autoware to finish the analyse.

### CARET

By using caret,you can trace the application without changing codes of application.You can analyse the latency and the running time of nodes.  
You can install TIDLE by following steps:

1. Clone caret and enter the directory.

   ```bash
       git clone https://github.com/tier4/caret.git ros2_caret_ws
       cd ros2_caret_ws
   ```

2. Create the src directory and clone repositories into it.

   ```bash
       mkdir src
       vcs import src < caret.repos --recursive
   ```

3. Run setup_caret.sh.

   ```bash
       ./setup_caret.sh
   ```

4. Build the workspace.

   ```bash
       source /opt/ros/galactic/setup.bash(or source /opt/ros/humble/setup.bash)
       colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

5. Check whether CARET (ros2-tracing) is enabled.

   ```bash
       source ~/ros2_caret_ws/install/local_setup.bash
       ros2 run tracetools status # return Tracing enabled
   ```

You can get more information in the link:

- <https://tier4.github.io/CARET_doc/>

### TILDE

TILDE is a framework for latency measurement and deadline detection across multiple nodes. TILDE can track the topic output from the node and identify the original input topic. Latency measurement and deadline detection are possible to trace from input to output.By using the TILDE,you need change the codes of application and add the apis of TILDE in the application.  
You can install TIDLE by following steps:

1. Clone TILDE and enter the directory.

   ```bash
       git clone https://github.com/tier4/TILDE.git ros2_TILDE_ws
       cd ros2_TILDE_ws
   ```

2. Create the src directory and clone repositories into it.

   ```bash
       mkdir src
       vcs import src < build_depends.repos
   ```

3. Build the workspace.

   ```bash
   source /opt/ros/galactic/setup.bash(or source /opt/ros/humble/setup.bash)
   colcon build --symlink-install --cmake-args --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

You can get more information in the link:

- <https://github.com/tier4/TILDE/blob/master/doc/README.md>
