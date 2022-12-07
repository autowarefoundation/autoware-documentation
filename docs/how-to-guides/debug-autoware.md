# Debug Autoware

The essential thing for debug is to print the program information clearly, which can quickly judge the program operation and locate the problem.
This page lists a simple example of how to debug Autoware.

## How to debug Autoware

Debug macros are very important in C programming, they are usually used to print debug information in early tests. When officially released, it is very convenient to cancel print all debug information by modifying the macro.

1. Define a parameter to indicate whether to display debug information

   ```cpp
   bool show_debug_info_;
   ```

2. Define a macro that prints debug information

   ```cpp
   #define DEBUG_INFO(...) {if(show_debug_info_){RCLCPP_INFO(__VA_ARGS__);}}
   ```
   
   **NOTE**: `__VA_ARGS__` is a variable argument macro.

   Next, you can use `DEBUG_INFO` to output debug information.

3. Output debug information

   We use `ekf_localizer` module as an example:
   ```cpp
   DEBUG_INFO(get_logger(), "[EKF] predictKinematicsModel calc time = %f [ms]", stop_watch_.toc());
   ```

   **NOTE**: Add [EKF] before debug information to facilitate filtering debug information of other modules.