# Debug Autoware

This page provides some methods for debugging Autoware.

## Print debug messages

The essential thing for debug is to print the program information clearly, which can quickly judge the program operation and locate the problem. Autoware uses ROS2 logging tool to print debug messages, how to design console logging refer to tutorial [Console logging](../contributing/coding-guidelines/ros-nodes/console-logging.md).

## Using ROS2 tools debug Autoware

### Using command line tools

ROS2 includes a suite of command-line tools for introspecting a ROS2 system. The main entry point for the tools is the command `ros2`, which itself has various sub-commands for introspecting and working with nodes, topics, services, and more. How to use the ROS2 command line tool refer to tutorial [CLI tools](http://docs.ros.org/en/galactic/Tutorials/Beginner-CLI-Tools.html).

### Using rviz2

Rviz2 is a port of Rviz to ROS2. It provides a graphical interface for users to view their robot, sensor data, maps, and more. You can run Rviz2 tool easily by:
it will open

```console
rviz2
```

When Autoware launch the simulators, the Rviz2 tool is opened by default to visualize the autopilot graphic information.

### Using rqt tools

RQt is a graphical user interface framework that implements various tools and interfaces in the form of plugins. You can run any RQt tools/plugins easily by:

```console
rqt
```

This GUI allows you to choose any available plugins on your system. You can also run plugins in standalone windows. For example, RQt Console:

```console
ros2 run rqt_console rqt_console
```

#### Common RQt tools

1. rqt_graph: view node interaction

   In complex applications, it may be helpful to get a visual representation of the ROS node interactions.

   ```console
   ros2 run rqt_graph rqt_graph
   ```

2. rqt_console: view messages

   rqt_console is a great gui for viewing ROS topics.

   ```console
   ros2 run rqt_console rqt_console
   ```

3. rqt_plot: view data plots

   rqt_plot is an easy way to plot ROS data in real time.

   ```console
   ros2 run rqt_plot rqt_plot
   ```

## Using a debugger with breakpoints

Many IDE(e.g. VSCode, CLion) supports debugging C/C++ executable with GBD on linux platform. The following lists some references for using the debugger:

- <https://code.visualstudio.com/docs/cpp/cpp-debug>
- <https://www.jetbrains.com/help/clion/debugging-code.html#useful-debugger-shortcuts>
