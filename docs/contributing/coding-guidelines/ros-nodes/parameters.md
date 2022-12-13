# Parameters

In autoware you can use xml or py to set values of parameters in nodes.In the folder of "src/universe/autoware.universe/launch",it contains most of the configuration parameters and launch files for different modules.Each module has the corresponding parameter configuration file.In autoware,you can use the configuration file or use the name of parameters to set values of parameters.

## Configure file of parameters
You can use the configure file to manage parameters.
For example,there is the yaml configure file about parameters of behavior_path_planner
```yaml
/**:
  ros__parameters:
    backward_path_length: 5.0
    forward_path_length: 850.0
    backward_length_buffer_for_end_of_lane: 5.0
    backward_length_buffer_for_end_of_pull_over: 5.0
    backward_length_buffer_for_end_of_pull_out: 5.0
    minimum_lane_change_length: 12.0
```
You can use and set parameters by configure file of parameters in node.
## Set parameters by using py
You can set parameters in py by using theses functions:
- os.path.join()
- LaunchConfiguration()
- yaml.safe_load()
- ComposableNode()

### Set the path of configuration file of parameters

You can use variable to set the path of configuration file.The variable can be set in the current file or the upper file.

For example:
```py
    behavior_path_planner_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_path_planner",
        "behavior_path_planner.param.yaml",
    ) 
```
The first parameter of join() is the absolute path, followed by the relative path.The "tier4_planning_launch_param_path" is set by top file(which can be xml or py).    
For example, "tier4_planning_launch_param_path" is set in "planning.launch.xml".The "planning.launch.xml" is the top-level planning launch file:
```xml
<!-- parameter path -->
  <arg name="tier4_planning_launch_param_path" default="$(find-pkg-share tier4_planning_launch)/config" description="tier4_planning_launch parameter path"/>
```
 "tier4_planning_launch_param_path" can be transmit to other launch files(which can be xml or py) downward
 ### Load parameters
You can ues the yaml.safe_load() to get the parameters.  
For example:
```py
    with open(behavior_path_planner_param_path, "r") as f:
        behavior_path_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]
```
### Launch composable node with parameters
You can ues the ComposableNode() to set the parameters when you launch nodes.  
For example:
```py
behavior_path_planner_component = ComposableNode(
        package="behavior_path_planner",
        plugin="behavior_path_planner::BehaviorPathPlannerNode",
        name="behavior_path_planner",
        namespace="",
        remappings=[
            ("~/input/route", LaunchConfiguration("input_route_topic_name")),
        ],
        parameters=[
            lane_following_param,
            behavior_path_planner_param,
           ]
    )
```
You can see more information about using ROS2 launch to launch composable node in the link:
- http://docs.ros.org/en/humble/How-To-Guides/Launching-composable-nodes.html


## Set parameters by using xml

When you run node by using xml,you can use the following commands to set parameters:
- param from(set parameters by file)
- param name(set parameter by name)  

For example:
```xml
<node pkg="behavior_path_planner" exec="behavior_path_planner" name="behavior_path_planner" output="screen">
    <param from="$(find-pkg-share tier4_planning_launch)/config/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/behavior_path_planner.param.yaml"/>
    <param from="$(var vehicle_info_param_file)"/>
    <param name="bt_tree_config_path" value="$(find-pkg-share behavior_path_planner)/config/behavior_path_planner_tree.xml"/>
```
When you use the command "param from",the path of configure file about parameters shall be set.You can use "find-pkg-share" to find the absolute path and then add the relative path as the path of configure file.For example:
```xml
<param from="$(find-pkg-share tier4_planning_launch)/config/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/behavior_path_planner.param.yaml"/>
```
You also can use variable which can be set in current xml or the upper xml(or py) as the path of configure file.For example:
```xml
<param from="$(var vehicle_info_param_file)"/>
```
The "vehicle_info_param_file" is set in the "autoware.launch.xml"(which is top-level configure file of autoware).  
When you use the command "param name",you can use parameter's name to set the value.For example:
```C++
<param name="bt_tree_config_path" value="$(find-pkg-share behavior_path_planner)/config/behavior_path_planner_tree.xml"/>
```
## Declare parameters by command in node
In node you shall declare parameters before using them and setting them.You can use the following commands to declare parameters:
- declare_parameter  

For example:
```C++
  p.backward_path_length = declare_parameter("backward_path_length", 5.0) + backward_offset;
  p.forward_path_length = declare_parameter("forward_path_length", 100.0);
```
You can see more information about parameters of ROS2 in the link:
- https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html