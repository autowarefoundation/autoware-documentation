# Console logging

ROS2 logging is a powerful tool for understanding and debugging ros node.

The logging subsystem in ROS 2 delivers logging messages to a variety of targets, including console, disk and ROS2 network.

This page focus on console logging usage in autoware.

## Overview

Console logging is commonly used in autoware, almost every module has logging, following are examples:

```bash
RCLCPP_INFO(get_logger(), "Vehicle Disengage");
RCLCPP_ERROR(rclcpp::get_logger("map_loader"), "lanelet2_map_projector_type is not supported");
```

Console logging shows useful message to help user to use, debug and learn the system.

For the detailed API usage, please see [ros2 logging API documentation](https://docs.ros.org/en/humble/Concepts/About-Logging.html#apis).

## Guideline

- **Check console logging when running system**

When autoware running, check console logging to know the system condition.

the following log in initialization warns me to check the setup such as lanelet map.

```bash
[component_container_mt-28] [WARN] [1670838292.877830206] [planning.scenario_planning.lane_driving.motion_planning.obstacle_avoidance_planner]: failed to get transform from map to base_link: "map" passed to lookupTransform argument target_frame does not exist.
```

- **Severity in autoware**

`DEBUG`-> message only shows in debug mode, it is often used to debug autoware function and sanity check.

`INFO`-> useful message about autoware such like node or container initialization, autonomous vehicle driving condition, autoware module function condition etc.

`WARN`-> message that is not critical but should warn operator of autonomous vehicle, such as get vehicle infomation failed, optimization failed, etc.

`ERROR`-> message that alerts operator the autoware system is not working correctly or not capable of autodriving anymore, such like empty reference trajectory, Emergency stop etc.

operator should take action according to it.

- **Avoid terminal contamination**

To avoid output too many logs in the console, try to use throttled logger interface, basically like this:

```bash
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "waiting for current_pose...");
```

the third argument is the time between every logging message defaults in milliseconds, 5000 means this logger outputs message every 5 second.

choose right frequency in throttled logger, `INFO` message can be less frequent, but `ERROR` meassage should in high frequency.

## Useful Links

1. [customize node’s output level, format or colorizing](https://docs.ros.org/en/humble/Concepts/About-Logging.html#configuration)
2. [logging basic demo and commandline](https://docs.ros.org/en/humble/Concepts/About-Logging.html)
3. [rqt_console](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)
