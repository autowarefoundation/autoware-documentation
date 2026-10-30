# Point cloud type transition

Tracked in [Point Cloud Type Transition #6708](https://github.com/autowarefoundation/autoware/issues/6708).

Autoware transitions from using raw memcpy operations and using [PointCloud2Iterator<>](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.hpp) to using [Point Cloud Message Wrapper](https://gitlab.com/ApexAI/point_cloud_msg_wrapper).

This way the point cloud messages can be edited in-place with a `std::vector<>`-like wrapper.

The only downside is that this requires strong type definitions for the point cloud message fields.

We have already made the compatibility requirements for `PointXYZIRC` and `PointXYZIRCAEDT` types with [autoware_universe#6996](https://github.com/autowarefoundation/autoware_universe/pull/6996).

Now we need to start using this wrapper in the rest of the codebase.

We also need to provide a tool to convert point cloud messages from outside types to the Autoware expected types.

The [point type adapter node of Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/tools/point_type_adapter/src/point_type_adapter_node.cpp) is a reference implementation.

For the fields of each point type and their meaning, see [Point cloud fields](../../../design/autoware-architecture-v1/components/sensing/data-types/point-cloud.md#point-cloud-fields).

## Transition

The plan applies to each node:

1. **2026 May:** Switch all memcpy usages to `PointCloud2Iterator<>`.
2. **2026 June:** Add two paths for the codebase: check the full compatibility of the point type with the expected point type.
   - If compatible, use the wrapper.
   - If not compatible, use `PointCloud2Iterator<>` and print warnings.
3. **2026 June:** Add a node that converts point cloud messages from outside types to the Autoware expected types.
4. **2026 December:** Drop the `PointCloud2Iterator<>` usage.

{{ diagram("images/point-types-timeline.svg", "The transition plan has three phases from May to December 2026, and the first two phases are not complete in Autoware 2.0.0.") }}

## Status in Autoware 2.0.0

The issue is open. The steps of May and June 2026 are not complete in Autoware 2.0.0 (`autoware_core` 1.10.0 and `autoware_universe` 0.53.1).

What is ready:

- The [`autoware_point_types`](https://github.com/autowarefoundation/autoware_core/tree/1.10.0/common/autoware_point_types) package of Autoware Core defines the point types `PointXYZI`, `PointXYZIRC`, `PointXYZIRCT`, `PointXYZIRADRT`, `PointXYZIRCAEDT`, and `PointXYZCPE`. It registers the types for PCL and defines field generators for `point_cloud_msg_wrapper`.
- The `memory.hpp` header of this package has a layout check for each type, for example `is_data_layout_compatible_with_point_xyzirc()`. This check is the compatibility test of the second step.
- Some nodes of `autoware_pointcloud_preprocessor` already use the wrapper: the concatenation nodes, the time synchronizer, and the ring outlier filter.

What is left:

- Many nodes still copy point data with `memcpy`, for example in `autoware_ground_filter`, `autoware_crop_box_filter`, `autoware_pointcloud_preprocessor`, and `autoware_compare_map_segmentation`.
- Many nodes still read point data with `PointCloud2Iterator<>`, for example in `autoware_pointcloud_preprocessor`, `autoware_probabilistic_occupancy_grid_map`, and `autoware_image_projection_based_fusion`.
- Autoware 2.0.0 has no node that converts outside point types to the Autoware point types.
