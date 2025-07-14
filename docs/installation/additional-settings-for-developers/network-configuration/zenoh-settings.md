# Zenoh settings for ROS 2 and Autoware

## Install rmw_zenoh

1. Install rmw_zenoh

   ```bash
   sudo apt update && sudo apt install ros-<DISTRO>-rmw-zenoh-cpp
   # Replace <DISTRO> with your ROS 2 distribution codename, e.g., humble, iron, rolling
   ```

2. Set rmw_zenoh as the default RMW implementation

   Add the following line to your `~/.bashrc` file:

   ```bash
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   ```

3. Reload your shell configuration (or open a new terminal):

   ```bash
   source ~/.bashrc
   ```

For more details, see the [rmw_zenoh repository](https://github.com/ros2/rmw_zenoh).

## Avoid the GuardCondition Use-After-Free Issue with Zenoh

When running Autoware with Zenoh, a modification to `behavior_planning.launch.xml` is required to avoid a GuardCondition use-after-free issue in ROS 2 Humble’s multi-threaded executor. This issue is fixed in ROS 2 Rolling (2025.04+) but not backported.

There are two options to avoid this issue:

### Option 1: Use the Latest Autoware Main Branch

If you are using the latest Autoware main branch, you can use our patched `tier4_planning_launch` package directly.
This package already runs `behavior_path_planner` in a single-threaded node container, so no manual modification is needed.

1. Clone `evshary/tier4_planning_launch` and move to the directory:

   ```bash
   git clone https://github.com/evshary/tier4_planning_launch.git
   cd tier4_planning_launch
   ```


2. Build the workspace:

   ``` bash
   source ~/<YOUR-AUTOWARE-DIR>/install/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

3. Launch Autoware with Zenoh:

   Start the Zenoh router:

   ```bash
   # terminal 1
   ros2 run rmw_zenoh_cpp rmw_zenohd
   ```

   Launch Autoware with the `tier4_planning_launch` overlay applied:

   ```bash
   # terminal 2
   source ~/<YOUR-TIER4_PLANNING_LAUNCH-DIR>/install/setup.bash
   ros2 launch autoware_launch autoware.launch.xml ...
   ```

### Option 2: Manually Patch the Launch File

If you are using a version of Autoware that is not compatible with our `tier4_planning_launch`, you need to manually patch `behavior_planning.launch.xml` to run `autoware_behavior_path_planner` in a separate single-threaded container.

1. Open the launch file:

   ```bash
   vim ~/<YOUR-AUTOWARE-DIR>/src/universe/autoware_universe/launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml
   ```


2. Find the block for `autoware_behavior_path_planner`:

   ```xml
   <load_composable_node target="/planning/scenario_planning/lane_driving/behavior_planning/behavior_planning_container">
     <composable_node pkg="autoware_behavior_path_planner"
                     plugin="autoware::behavior_path_planner::BehaviorPathPlannerNode"
                     name="behavior_path_planner"
                     namespace="">
       ...
     </composable_node>
   </load_composable_node>
   ```

3. Modify opening and closing tags and add the `thread_num` param to ensure  single-threaded execution:
  
   ```xml
   <!-- <load_composable_node target="/planning/scenario_planning/lane_driving/behavior_planning/behavior_planning_container"> -->
   <node_container pkg="rclcpp_components" exec="$(var container_type)" name="behavior_planning_container2" namespace="" args="" output="both">
     <composable_node pkg="autoware_behavior_path_planner"
                     plugin="autoware::behavior_path_planner::BehaviorPathPlannerNode"
                     name="behavior_path_planner"
                     namespace="">
       ...
     </composable_node>
     <param name="thread_num" value="1"/>
   <!-- </load_composable_node> -->
   </node_container>
   ```

4. Rebuild your workspace (if necessary):

   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```


5. Launch modified Autoware with rmw_zenoh
 
   Start the Zenoh router
   ```bash
   # terminal 1
   ros2 run rmw_zenoh_cpp rmw_zenohd
   ```

   Launch modified Autoware
   ```bash
   # terminal 2
   source ~/<YOUR-AUTOWARE-DIR>/install/setup.bash
   ros2 launch autoware_launch autoware.launch.xml ...
   ```

## Logging

Zenoh is implemented in Rust and uses a logging library configurable via the `RUST_LOG` environment variable.
You can specify different levels (such as `info`, `debug`, or `trace`) for more or less verbosity.

### Example:

Start the Zenoh router with debug logs enabled:

```bash
export RUST_LOG=zenoh=debug
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Or launch autoware with info log in a single command:

```bash
RUST_LOG=zenoh=info ros2 launch autoware_launch autoware.launch.xml ...
```

For more information, see the [rmw_zenoh logging section](https://github.com/ros2/rmw_zenoh?tab=readme-ov-file#logging).