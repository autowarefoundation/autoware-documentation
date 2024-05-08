# Class design

We'll use the `autoware_gnss_poser` package as an example.

## Namespaces

```cpp
namespace autoware::gnss_poser
{
...
} // namespace autoware::gnss_poser
```

- Everything should be under `autoware::gnss_poser` namespace.
- Closing braces should contain a comment with the namespace name.

## Classes

### Nodes

#### `gnss_poser.hpp`

```cpp
class GNSSPoserNode : public rclcpp::Node
{
  public:
    explicit GNSSPoserNode(const rclcpp::NodeOptions & node_options);
  ...
}
```

#### `gnss_poser.cpp`

```cpp
GNSSPoserNode::GNSSPoserNode(const rclcpp::NodeOptions & node_options)
: Node("gnss_poser_node", node_options)
{
  ...
}
```

- The class name should be in `CamelCase`.
- Node classes should inherit from `rclcpp::Node`.
- The constructor should be explicit.
- The constructor should take `rclcpp::NodeOptions` as an argument.
- Default node name:
  - should not have `autoware_` prefix.
  - should have `_node` suffix.
  - **Example:** `gnss_poser_node`.

##### Component registration

```cpp
...
} // namespace autoware::gnss_poser

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gnss_poser::GNSSPoserNode)
```

- The component should be registered at the end of the `autoware_gnss_poser.cpp` file, outside the namespaces.

### Libraries

!!! warning

    Under Construction
