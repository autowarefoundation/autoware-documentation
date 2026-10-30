# Class design

We'll use the [`autoware_gnss_poser`](https://github.com/autowarefoundation/autoware_core/tree/1.10.0/sensing/autoware_gnss_poser) package as an example.

## Namespaces

```cpp
namespace autoware::gnss_poser
{
...
}  // namespace autoware::gnss_poser
```

- Everything should be under `autoware::gnss_poser` namespace.
- Closing braces must contain a comment with the namespace name. (`clang-format` adds the comment, and `cpplint` checks it.)

## Classes

### Nodes

#### `gnss_poser_node.hpp`

```cpp
class GNSSPoser : public rclcpp::Node
{
public:
  explicit GNSSPoser(const rclcpp::NodeOptions & node_options);
  ...
};
```

#### `gnss_poser_node.cpp`

```cpp
GNSSPoser::GNSSPoser(const rclcpp::NodeOptions & node_options)
: Node("gnss_poser", node_options)
{
  ...
}
```

- The class name should be in `CamelCase`.
- Node classes should inherit from `rclcpp::Node`. A node that supports Agnocast inherits from `autoware::agnocast_wrapper::Node` instead. See [Nodes that support Agnocast](#nodes-that-support-agnocast).
- The constructor must be marked explicit.
- The constructor must take `rclcpp::NodeOptions` as an argument.
- Default node name:
  - should not have `autoware_` prefix.
  - should **NOT** have `_node` suffix.
    - **Rationale:** Node names are useful in the runtime. And output of `ros2 node list` will show only nodes anyway. Having `_node` is redundant.
  - **Example:** `gnss_poser`.

##### Component registration

```cpp
...
}  // namespace autoware::gnss_poser

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gnss_poser::GNSSPoser)
```

- The component should be registered at the end of the `gnss_poser_node.cpp` file, outside the namespaces.
- A node that supports Agnocast uses the same macro in the source file. The CMake registration is different. See [Directory structure](directory-structure.md#exporting-a-composable-node-component-executables).

#### Nodes that support Agnocast

Agnocast is a zero-copy middleware that works together with ROS 2. The [`autoware_agnocast_wrapper`](https://github.com/autowarefoundation/autoware_core/tree/1.10.0/common/autoware_agnocast_wrapper) package of Autoware Core lets one node run on `rclcpp` or on Agnocast. Many nodes in Autoware Core and Autoware Universe use it. The real `autoware_gnss_poser` is one of them:

```cpp
#include <autoware/agnocast_wrapper/node.hpp>

namespace autoware::gnss_poser
{
class GNSSPoser : public autoware::agnocast_wrapper::Node
{
public:
  explicit GNSSPoser(const rclcpp::NodeOptions & node_options);
  ...
};
}  // namespace autoware::gnss_poser
```

```cpp
GNSSPoser::GNSSPoser(const rclcpp::NodeOptions & node_options)
: autoware::agnocast_wrapper::Node("gnss_poser", node_options)
{
  ...
}
```

- The rules for the class name, the constructor, and the node name stay the same.
- `autoware::agnocast_wrapper::Node` does not derive from `rclcpp::Node`. It gives a subset of the `rclcpp::Node` API, with the same member names and arguments.
- Write the types of publishers, subscriptions, and timers with the macros of the wrapper, for example `AUTOWARE_PUBLISHER_PTR(MessageT)`, `AUTOWARE_SUBSCRIPTION_PTR(MessageT)`, and `AUTOWARE_TIMER_PTR`. Then the same source code builds with Agnocast and without it.
- The build uses Agnocast only when the `ENABLE_AGNOCAST` environment variable is `1`. The default build uses `rclcpp` only.
- For all the rules, see the [README of `autoware_agnocast_wrapper`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/common/autoware_agnocast_wrapper/README.md).

### Libraries

A library package gives classes and functions to other packages, and it has no node. We'll use the [`autoware_interpolation`](https://github.com/autowarefoundation/autoware_core/tree/1.10.0/common/autoware_interpolation) package of Autoware Core as an example.

#### `include/autoware/interpolation/linear_interpolation.hpp`

```cpp
namespace autoware::interpolation
{
double lerp(const double src_val, const double dst_val, const double ratio);
...
}  // namespace autoware::interpolation
```

- Put everything under the `autoware::` namespace, followed by the package name without the `autoware_` prefix. **Example:** `autoware::interpolation`.
- Put the exported headers in `include/autoware/interpolation/`. See [Exporting headers](directory-structure.md#exporting-headers).
- Build the library as a shared library, with the package name as the target name:

  ```cmake
  ament_auto_add_library(autoware_interpolation SHARED
    src/linear_interpolation.cpp
    ...
  )
  ```

- Do not make core library classes depend on `rclcpp::Node` (advisory). If a class writes logs, give it an `rclcpp::Logger`. See [Console logging](console-logging.md#do-not-depend-on-rclcppnode-in-core-library-classes-but-depend-only-on-rclcpplogginghpp-advisory-non-automated).

#### Separate the algorithm from the node

In a node package, keep the algorithm in its own class. The node class declares the parameters, receives and sends the messages, and calls the algorithm class. Then you can test the algorithm without ROS communication.

The [`autoware_simple_pure_pursuit`](https://github.com/autowarefoundation/autoware_core/tree/1.10.0/control/autoware_simple_pure_pursuit) package of Autoware Core uses this structure:

```cpp
// src/simple_pure_pursuit.hpp
namespace autoware::control::simple_pure_pursuit
{
struct SimplePurePursuitParameters
{
  double lookahead_gain;
  double lookahead_min_distance;
  ...
};

class SimplePurePursuit
{
public:
  explicit SimplePurePursuit(const SimplePurePursuitParameters & params);

  [[nodiscard]] autoware_control_msgs::msg::Control create_control_command(
    const nav_msgs::msg::Odometry & odom,
    const autoware_planning_msgs::msg::Trajectory & traj) const;
  ...
};
}  // namespace autoware::control::simple_pure_pursuit
```

- `SimplePurePursuit` holds the algorithm. It gets its parameters in a struct, and it gets the messages as function arguments.
- `SimplePurePursuitNode` in `src/simple_pure_pursuit_node.hpp` inherits from `rclcpp::Node`. It keeps a `SimplePurePursuit` object and calls `create_control_command()` from its timer callback.
- `test/test_simple_pure_pursuit.cpp` tests `SimplePurePursuit` without a node.
