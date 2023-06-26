# An example procedure for adding a new node - Case of localization

## Overview

This page provides a guide for evaluating Autoware when a new node is implemented, especially about developing a novel localization node.

To keep things simple, an example of operation within AWSIM, as opposed to actual hardware, is provided.
The same procedure could likely apply when operating on real hardware.

It is assumed that the method intended for addition has already been verified well with public datasets and so on.

## 1. Running Autoware in its standard configuration

First of all, it is important to be able to run the standard Autoware to establish a basis for performance and behavior comparison.

Autoware constantly incorporates new features.
It is crucial to initially confirm that it operates as expected with the current version, which helps in problem troubleshooting.

In this context, AWSIM is presumed.
Therefore, [AWSIM simulator](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/digital-twin-simulation/awsim-tutorial/) can be useful.
If you are using actual hardware, please refer to the [How-to guides](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/).

## 2. Recording a rosbag using Autoware

Before developing a new node, it is recommended to record a rosbag in order to evaluate.
If you need a new sensor, you should add it to your vehicle or AWSIM.

In this case, it is recommended to save all topics regardless of whether they are necessary or not.
For example, in Localization, since the initial position estimation service is triggered by the input to rviz and the GNSS topic, the initial position estimation does not start when playing back data unless those topics are saved.

Consider the use of the [mcap format](https://mcap.dev/) if data capacity becomes a concern.

It is worth noting that using `ros2 bag record` increases computational load and might affect performance.
After data recording, verifying the smooth flow of sensor data and unchanged time series is advised.
This verification can be accomplished, for example, by inspecting the image data with `rqt_image_view` during `ros2 bag play`.

## 3. Developing the new node

When developing a new node, it could be beneficial to reference a package that is similar to the one you intend to create.

It is advisable to thoroughly read the [Design page](https://autowarefoundation.github.io/autoware-documentation/main/design/), contemplate the addition or replacement of nodes in Autoware, and then implement your solution.

For example, a node doing NDT, a LiDAR-based localization method, is [ndt_scan_matcher](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/ndt_scan_matcher).
If you want to replace this with a different approach, implement a node which produces the same topics and provides the same services.

`ndt_scan_matcher` is launched as [pose_estimator](https://github.com/autowarefoundation/autoware.universe/blob/main/launch/tier4_localization_launch/launch/pose_estimator/pose_estimator.launch.xml), so it is necessary to replace the launch file as well.

## 4. Evaluating by a rosbag-based simulator

Once the new node is implemented, it is time to evaluate it.
[logging_simulator](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/rosbag-replay-simulation/) is a tool of how to evaluate the new node using the rosbag captured in step 2.

It works by replacing the command to start Autoware with logging_simulator.

For example, if the AWSIM-compatible execution command is

```
ros2 launch autoware_launch e2e_simulator.launch.xml ...
```

then the command to start logging_simulator is

```
ros2 launch autoware_launch logging_simulator.launch.xml ...
```

After launching logging_simulator, the rosbag file obtained in step 2 should be replayed using `ros2 bag play <rosbag_file>`.

If you remap the topics related to the localization that you want to verify this time, Autoware will use the data it is calculating this time instead of the data it recorded.

There is [ros2bag_extensions](https://github.com/tier4/ros2bag_extensions) available to filter the rosbag file and create a new rosbag file that contains only the topics you need.

## 5. Evaluating in a realtime environment

Once you have sufficiently verified the behavior in the logging_simulator, let's run it as Autoware with new nodes added in the realtime environment (AWSIM in this case).

To debug Autoware, the method described at [debug-autoware](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/others/debug-autoware/) is useful.

For reproducibility, you may want to fix the GoalPose.
In such cases, consider using
the [tier4_automatic_goal_rviz_plugin](https://github.com/autowarefoundation/autoware.universe/tree/main/common/tier4_automatic_goal_rviz_plugin).

## 6. Sharing the results

If your implementation works successfully, please consider a pull request to Autoware.

It is also a good idea to start by presenting your ideas in Discussion at [Show and tell](https://github.com/orgs/autowarefoundation/discussions/categories/show-and-tell).

For localization, [YabLoc's Proposal](https://github.com/orgs/autowarefoundation/discussions/3484) may provide valuable insights.
