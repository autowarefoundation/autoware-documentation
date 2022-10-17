# Console logging

ROS2 logging is a powerful tool for understanding and debugging ros node.

The logging subsystem in ROS 2 delivers logging messages to a variety of targets, including console, disk and ROS2 network. 

This page focus on console logging and shows some useful example in practice. for detailed information, please see [logging documentation](https://docs.ros.org/en/humble/Concepts/About-Logging.html).

## API
There are four pattern of logging function:

 ```bash
RCLCPP_{severity}     
RCLCPP_{severity}_STREAM
RCLCPP_{severity}_{condition} 
RCLCPP_{severity}_STREAM_{condition}  
 ```
`STREAM` means output message in api is C++stream-style else printf style.

`severity` have five choice: 

- `DEBUG, INFO, WARN, ERROR or FATAL` 

Set severity level to `DEBUG` only outputs message in debug mode.

`condition`  choices are:

 - `THROTTLE` - output the given printf-style message no more than the given rate
 - `ONCE` - output the given printf-style message only the first time this line is hit
 - `EXPRESSION` - output the given printf-style message only if the given expression is true
 - `FUNCTION` - output the given printf-style message only if the given function returns true
 - `SKIPFIRST` - output the given printf-style message all but the first time this line is hit

## Console output formatting
Default output format is [{severity}] [{time}] [{name}]: {message}

To change log output format in console:
```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
```
- {severity} - The severity level.

- {name} - The name of the logger (may be empty).

- {message} - The log message (may be empty).

- {function_name} - The function name this was called from (may be empty).

- {file_name} - The file name this was called from (may be empty).

- {time} - The time in seconds since the epoch.

- {time_as_nanoseconds} - The time in nanoseconds since the epoch.

- {line_number} - The line number this was called from (may be empty)

## Examples
let's assume a ros node named planner and format is [{severity}] [{name}]: {message}

To record information every time the code executed somewhere inside planner function, you can do:
 ```bash
RCLCPP_INFO(this->get_logger(), " status %d", 0); 
RCLCPP_INFO_STREAM(this->get_logger(), "status " << 0);
 ```
>[INFO] [planner]: status 0

To record messages in function outside planner node, create a logger object:
```bash
string object_type = "static car";
auto logger = rclcpp::get_logger("obstacle_check");
RCLCPP_WARN_STREAM(logger, *rclcpp::get_clock(), "meet obstacle: "<<object_type<<"!");
```
>[WARN] [obstacle_check]: meet obstacle: static car!

To record a log message but no more one second a time, use throttled logger:
```bash
auto base_link = "base_link";
auto map_id = "world";
RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to look up transform from"<<base_link<<" to "<<map_id);
```
>[ERROR] [planner]: Failed to look up transform from base_link to world

To conditionally output log messages, use EXPRESSION：
```bash
double goal_velocity = 1.0;
RCLCPP_ERROR_EXPRESSION(this->get_logger(), goal_velocity!=0.0, "goal velocity should be zero!");
```
>[ERROR] [planner]: goal velocity should be zero!

To debug program, sometimes you need see which functions and lines of code are executed, use `__LINE__` and `__FUNCTION__` macro:
```bash
RCLCPP_DEBUG_STREAM(this->get_logger(), "executed line: "<< __LINE__<<" in function: " <<__FUNCTION__);
```
example output: 
>[DEBUG] [planner]: executed line 100 in function make plan

To filter logs, use | grep pipe:
```bash
ros2 launch planner planner | grep obstacle
```
the output will only contain obstacle_check's log.

## Tips
1. choose right severity of your log and record debug information as often as possible, it helps understand
program and doesn't effect in release code.
2. choose right frequency in throttled logger, one second a log may be too slow or too fast.
3. use "| grep {keywords}" to filter logs you want to see.
4. use marco expression to help you debug code.