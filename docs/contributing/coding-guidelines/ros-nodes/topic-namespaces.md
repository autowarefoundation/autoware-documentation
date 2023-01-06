# Topic namespaces

## Overview

ROS allows topics, parameters and nodes to be namespaced which provides the following benefits:

- Multiple instances of the same node type will not cause naming clashes.
- Topics published by a node can be automatically namespaced with the node's namespace providing a meaningful and easily-visible connection.
- Keeps from cluttering the root namespace.
- Helps to maintain separation-of-concerns.

This page focuses on how to use namespaces in Autoware and shows some useful examples. For basic information on topic namespaces, refer to [this tutorial](https://design.ros2.org/articles/topic_and_service_names.html).

## How topics should be named in Node

Autoware divides the node into the following functional categories, and adds the start namespace for the nodes according to the categories.

- localization
- perception
- planning
- control
- sensing
- vehicle
- map
- system

When a node is run in a namespace, all topics which that node publishes are given that same namespace. All nodes in the Autoware stack must support namespaces by avoiding practices such as publishing topics in the global namespace.

In general, topics should be namespaced based on the function of the node which produces them and not the node (or nodes) which consume them.

Classify topics as input or output topics based on they are subscribed or published by the node. In the node, input topic is named `input/topic_name` and output topic is named `output/topic_name`.

Configure the topic in the node's launch file. Take the `joy_controller` node as an example, in the following example, set the input and output topics and remap topics in the `joy_controller.luanch.xml` file.

```xml
<launch>
  <arg name="input_joy" default="/joy"/>
  <arg name="input_odometry" default="/localization/kinematic_state"/>

  <arg name="output_control_command" default="/external/$(var external_cmd_source)/joy/control_cmd"/>
  <arg name="output_external_control_command" default="/api/external/set/command/$(var external_cmd_source)/control"/>
  <arg name="output_shift" default="/api/external/set/command/$(var external_cmd_source)/shift"/>
  <arg name="output_turn_signal" default="/api/external/set/command/$(var external_cmd_source)/turn_signal"/>
  <arg name="output_heartbeat" default="/api/external/set/command/$(var external_cmd_source)/heartbeat"/>
  <arg name="output_gate_mode" default="/control/gate_mode_cmd"/>
  <arg name="output_vehicle_engage" default="/vehicle/engage"/>

  <node pkg="joy_controller" exec="joy_controller" name="joy_controller" output="screen">
    <remap from="input/joy" to="$(var input_joy)"/>
    <remap from="input/odometry" to="$(var input_odometry)"/>

    <remap from="output/control_command" to="$(var output_control_command)"/>
    <remap from="output/external_control_command" to="$(var output_external_control_command)"/>
    <remap from="output/shift" to="$(var output_shift)"/>
    <remap from="output/turn_signal" to="$(var output_turn_signal)"/>
    <remap from="output/gate_mode" to="$(var output_gate_mode)"/>
    <remap from="output/heartbeat" to="$(var output_heartbeat)"/>
    <remap from="output/vehicle_engage" to="$(var output_vehicle_engage)"/>
  </node>
</launch>
```

## Topic names in the code

1. Have `~` so that namespace in launch configuration is applied(should not start from root `/`).

2. Have `~/input` `~/output` namespace before topic name used to communicate with other nodes.

   e.g., In node `obstacle_avoidance_planner`, using topic names of type `~/input/topic_name` to subscribe to topics.

   ```cpp
   objects_sub_ = create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{10},
    std::bind(&ObstacleAvoidancePlanner::onObjects, this, std::placeholders::_1));
   ```

   e.g., In node `obstacle_avoidance_planner`, using topic names of type `~/output/topic_name` to publish topic.

   ```cpp
   traj_pub_ = create_publisher<Trajectory>("~/output/path", 1);
   ```

3. Visualization or debug purpose topics should have `~/debug/` namespace.

   e.g., In node `obstacle_avoidance_planner`, in order to debug or visualizing topics, using topic names of type `~/debug/topic_name` to publish information.

   ```cpp
   debug_markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", durable_qos);

   debug_msg_pub_ =
    create_publisher<tier4_debug_msgs::msg::StringStamped>("~/debug/calculation_time", 1);
   ```

   The launch configurated namespace will be add the topics before, so the topic names will be as following:

   `/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/debug/marker /planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/debug/calculation_time`

4. Rationale: we want to make topic names remapped and configurable from launch files.
