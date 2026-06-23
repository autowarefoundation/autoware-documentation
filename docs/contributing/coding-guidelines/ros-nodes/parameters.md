# Parameters

Autoware ROS nodes have declared parameters which values are provided during the node start up in the form of a parameter file. All the expected parameters with corresponding values should exist in the parameter file. Depending on the application, the parameter values might need to be modified.

Find more information on parameters from the official ROS documentation:

- [Understanding ROS 2 Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [About ROS 2 Parameters](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html)

## Workflow

A ROS package which uses the [declare_parameter(...)](https://docs.ros.org/en/ros2_packages/humble/api/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4N6rclcpp4Node17declare_parameterERKNSt6stringERKN6rclcpp14ParameterValueERKN14rcl_interfaces3msg19ParameterDescriptorEb) function should:

- use the [declare_parameter(...)](https://docs.ros.org/en/ros2_packages/humble/api/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4N6rclcpp4Node17declare_parameterERKNSt6stringERKN6rclcpp14ParameterValueERKN14rcl_interfaces3msg19ParameterDescriptorEb), without a default value
- create a parameter file
- create a schema file

The rationale behind this workflow is to have a verified single source of truth to pass to the ROS node and to be used in the web documentation. The approach reduces the risk of using invalid parameter values and makes maintenance of documentation easier. This is achieved by:

- [declare_parameter(...)](https://docs.ros.org/en/ros2_packages/humble/api/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4N6rclcpp4Node17declare_parameterERKNSt6stringERKN6rclcpp14ParameterValueERKN14rcl_interfaces3msg19ParameterDescriptorEb) throws an exception if an expected parameter is missing in the parameter file
- the schema validates the parameter file in the CI and renders a parameter table, as depicted in the graphics below

  ```mermaid
  flowchart TD
      NodeSchema[Schema file: *.schema.json]
      ParameterFile[Parameter file: *.param.yaml]
      WebDocumentation[Web documentation table]

      NodeSchema -->|Validation| ParameterFile
      NodeSchema -->|Generate| WebDocumentation
  ```

Note: a parameter value can still be modified and bypass the validation, as there is no validation during runtime.

## Declare Parameter Function

It is the [declare_parameter(...)](https://docs.ros.org/en/ros2_packages/humble/api/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4N6rclcpp4Node17declare_parameterERKNSt6stringERKN6rclcpp14ParameterValueERKN14rcl_interfaces3msg19ParameterDescriptorEb) function which sets the parameter values during a node startup.

```cpp
declare_parameter<INSERT_TYPE>("INSERT_PARAMETER_1_NAME"),
declare_parameter<INSERT_TYPE>("INSERT_PARAMETER_N_NAME")
```

As there is no _default_value_ provided, the function throws an exception if a parameter were to be missing in the provided `*.param.yaml` file. Use a type from the _C++ Type_ column in the table below for the [declare_parameter(...)](https://docs.ros.org/en/ros2_packages/humble/api/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4N6rclcpp4Node17declare_parameterERKNSt6stringERKN6rclcpp14ParameterValueERKN14rcl_interfaces3msg19ParameterDescriptorEb) function, replacing _INSERT_TYPE_.

| ParameterType Enum        | C++ Type                   |
| ------------------------- | -------------------------- |
| `PARAMETER_BOOL`          | `bool`                     |
| `PARAMETER_INTEGER`       | `int64_t`                  |
| `PARAMETER_DOUBLE`        | `double`                   |
| `PARAMETER_STRING`        | `std::string`              |
| `PARAMETER_BYTE_ARRAY`    | `std::vector<uint8_t>`     |
| `PARAMETER_BOOL_ARRAY`    | `std::vector<bool>`        |
| `PARAMETER_INTEGER_ARRAY` | `std::vector<int64_t>`     |
| `PARAMETER_DOUBLE_ARRAY`  | `std::vector<double>`      |
| `PARAMETER_STRING_ARRAY`  | `std::vector<std::string>` |

The table has been derived from [Parameter Type](https://github.com/ros2/rcl_interfaces/blob/humble/rcl_interfaces/msg/ParameterType.msg) and [Parameter Value](https://github.com/ros2/rcl_interfaces/blob/humble/rcl_interfaces/msg/ParameterValue.msg).

See example: _Lidar Apollo Segmentation TVM Nodes_ [declare function](https://github.com/autowarefoundation/autoware_universe/blob/f85c90b56ed4c7d6b52e787570e590cff786b28b/perception/lidar_apollo_segmentation_tvm_nodes/src/lidar_apollo_segmentation_tvm_node.cpp#L38)

## Parameter File

The parameter file is minimal as there is no need to provide the user with additional information, e.g., description or type. This is because the associated schema file provides the additional information. Use the template below as a starting point for a ROS node.

```yaml
/**:
  ros__parameters:
    INSERT_PARAMETER_1_NAME: INSERT_PARAMETER_1_VALUE
    INSERT_PARAMETER_N_NAME: INSERT_PARAMETER_N_VALUE
```

Note: `/**` is used instead of the explicit node namespace, this allows the parameter file to be passed to a ROS node which has been [remapped](https://design.ros2.org/articles/static_remapping.html).

To adapt the template to the ROS node, replace each `INSERT_PARAMETER_..._NAME` and `INSERT_PARAMETER_..._VALUE` for all parameters. Each [declare_parameter(...)](https://docs.ros.org/en/ros2_packages/humble/api/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4N6rclcpp4Node17declare_parameterERKNSt6stringERKN6rclcpp14ParameterValueERKN14rcl_interfaces3msg19ParameterDescriptorEb) takes one parameter as input. All the parameter files should have the `.param.yaml` suffix so that the auto-format can be applied properly.

Autoware has the following two types of parameter files for ROS packages:

- **Node parameter file**
  - Node parameter files store the typical parameters provided for each package in Autoware.
    - For example, [the parameter of `behavior_path_planner`](https://github.com/autowarefoundation/autoware_universe/tree/245242cee866de2d113e89c562353c5fc17f1f98/planning/behavior_path_planner/config)
  - All nodes in Autoware must have a parameter file if ROS parameters are declared in the node.
  - For `FOO_package`, the parameter is expected to be stored in `FOO_package/config`.
  - The launch file for individual packages must load node parameter by default:

```xml
<launch>
  <arg name="foo_node_param_path" default="$(find-pkg-share FOO_package)/config/foo_node.param.yaml" />

  <node pkg="FOO_package" exec="foo_node">
    ...
    <param from="$(var foo_node_param_path)" />
  </node>
</launch>
```

- **Launch parameter file**
  - When a user creates a launch package for the user's vehicle, the user should copy node parameter files for the nodes that are used in the launch file as "launch parameter files".
  - Launch parameter files are then customized specifically for the user's vehicle.
    - For example, [the customized parameter of `behavior_path_planner` stored under `autoware_launch`](https://github.com/autowarefoundation/autoware_launch/tree/5fa613b9d80bf4f0db77efde03a43f7ede6bac86/autoware_launch/config)
  - Launch parameter files are stored under the `autoware_launch` package.

### sync-params

Despite being unused in `autoware_launch`, node parameter files must be updated after adding or removing parameters in the node implementations. Each node parameter file serves as the master file for the corresponding launch parameter file.

In most cases, updating launch parameter files are automated by [sync-params](https://github.com/autowarefoundation/autoware_launch/actions/workflows/sync-params.yaml) workflow, given that the corresponding node parameter files have been properly updated.

A launch parameter file managed by sync-params workflow has a sync-params comment header starting with `# This file is managed by sync-params workflow`.
By default, a launch parameter file is identical to its corresponding node parameter file, except for the header.

#### Adding or removing parameters with sync-params workflow

In general, you should not create a PR for `autoware_launch` when you simply added or removed some node parameters. In such cases, follow these steps:

1. Merge the node PR containing the node parameter file update **first**.
2. Run sync-params workflow with the desired category. To see the available categories, see [the sync-param configuration file](https://github.com/autowarefoundation/autoware_launch/blob/b9c4c17a86fde1dbc5fd618f3707088c2c4faf1a/.github/sync-params.yaml).
3. A PR for the workflow will be created, or if already exists, the existing PR will be updated. [`tag:sync-params`](https://github.com/autowarefoundation/autoware_launch/pulls?q=is%3Aopen+is%3Apr+label%3Atag%3Async-params) label.

#### Overriding parameters with sync-params workflow

Some of the fields should be different from the corresponding node parameter file. Such fields may be marked with `# {OVERRIDE}` or `# {OVERRIDE: <reason>}` comment marker to keep the value different from the upstream node parameter file.

If you naively try to modify the parameter in the launch parameter file as in the next example, the next sync-params workflow run will try to revert the parameter to `foo: 42`, matching to the node parameter file.

```patch
 # NG
-foo: 42
+foo: 40
```

To make the change persist, open a PR that updates the launch parameter file as following:

```patch
 # OK
-foo: 42
+foo: 40 # {OVERRIDE}
```

If there is already a comment on the line, place the marker before the existing comment:

```patch
 # OK
-bar: asdf # existing comment
+bar: qwer # {OVERRIDE} existing comment
```

**Tip**: make the node parameter file as identical as possible to the intended launch parameter file, so that the override can be minimized.

#### Updating sync-params configurations

To add a new pair of a node parameter file and a launch parameter file, update [the sync-param configuration file](https://github.com/autowarefoundation/autoware_launch/blob/b9c4c17a86fde1dbc5fd618f3707088c2c4faf1a/.github/sync-params.yaml), following the next example.

```yaml
perception: # category
  - repository: autowarefoundation/autoware_universe # the source repository
    ref: main
    files:
      - # node parameter file path in the source repository
        source: perception/autoware_image_object_locator/config/bbox_object_locator.param.yaml
        # List of launch parameter file paths in autoware_launch repository
        variants:
          - path: autoware_launch/config/perception/object_recognition/detection/camera_vru_detection/near_range_camera_vru_detector.param.yaml
```

And then run `python .github/scripts/sync_params.py perception` (replace `perception` with your category) locally, which is the script that sync-params workflow uses under the hood.
Files in the same category might be updated as well, but commit only the file that you have just added and discard anything else. Other files should be addressed with the autogenerated sync-params PR.

To remove a pair, simply move the launch parameter file entry from the sync-param config file, and remove the sync-params comment header.

To specify the new upstream, in cases such as when the node package has been moved to a different repository, remove the old pair, and then re-add the launch parameter file with the new upstream.

## JSON Schema

[JSON Schema](https://json-schema.org/understanding-json-schema/about) is used the validate the parameter file(s) ensuring that it has the correct structure and content. Using JSON Schema for this purpose is considered best practice for cloud-native development. The schema template below shall be used as a starting point when defining the schema for a ROS node.

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "INSERT_TITLE",
  "type": "object",
  "definitions": {
    "INSERT_ROS_NODE_NAME": {
      "type": "object",
      "properties": {
        "INSERT_PARAMETER_1_NAME": {
          "type": "INSERT_TYPE",
          "description": "INSERT_DESCRIPTION",
          "default": "INSERT_DEFAULT",
          "INSERT_BOUND_CONDITION(S)": INSERT_BOUND_VALUE(S)
        },
        "INSERT_PARAMETER_N_NAME": {
          "type": "INSERT_TYPE",
          "description": "INSERT_DESCRIPTION",
          "default": "INSERT_DEFAULT",
          "INSERT_BOUND_CONDITION(S)": INSERT_BOUND_VALUE(S)
        }
      },
      "required": ["INSERT_PARAMETER_1_NAME", "INSERT_PARAMETER_N_NAME"],
      "additionalProperties": false
    }
  },
  "properties": {
    "/**": {
      "type": "object",
      "properties": {
        "ros__parameters": {
          "$ref": "#/definitions/INSERT_ROS_NODE_NAME"
        }
      },
      "required": ["ros__parameters"],
      "additionalProperties": false
    }
  },
  "required": ["/**"],
  "additionalProperties": false
}
```

The schema file path is `INSERT_PATH_TO_PACKAGE/schema/` and the schema file name is `INSERT_NODE_NAME.schema.json`. To adapt the template to the ROS node, replace each `INSERT_...` and add all parameters `1..N`.

See example: _Image Projection Based Fusion - Pointpainting_ [schema](https://github.com/autowarefoundation/autoware_universe/blob/main/perception/autoware_image_projection_based_fusion/schema/pointpainting.schema.json)

### Attributes

Parameters have several attributes, some are required and some optional. The optional attributes are highly encouraged when applicable, as they provide useful information about a parameter and can ensure the value of the parameter is within its bounds.

#### Required

- name
- type
  - see [JSON Schema types](http://json-schema.org/understanding-json-schema/reference/type.html)
- description

#### Optional

- default
  - a tested and verified value, see [JSON Schema default](https://json-schema.org/understanding-json-schema/reference/generic.html)
- bound(s)
  - type dependent, e.g., [integer](https://json-schema.org/understanding-json-schema/reference/numeric.html#integer), [range](https://json-schema.org/understanding-json-schema/reference/numeric.html#range) and [size](https://json-schema.org/understanding-json-schema/reference/object.html#size)

## Tips and Tricks

Using well established standards enables the use of conventional tooling. Below is an example of how to link a schema to the parameter file(s) using VS Code. This enables a developer with convenient features such as auto-complete and parameter bound validation.

In the root directory of where the project is hosted, create a `.vscode` folder with two files; `extensions.json` containing

```json
{
  "recommendations": ["redhat.vscode-yaml"]
}
```

and `settings.json` containing

```json
{
  "yaml.schemas": {
    "./INSERT_PATH_TO_PACKAGE/schema/INSERT_NODE_NAME.schema.json": "**/INSERT_NODE_NAME/config/*.param.yaml"
  }
}
```

The RedHat YAML extension enables validation of YAML files using JSON Schema and the `"yaml.schemas"` setting associates the `*.schema.json` file with all `*.param.yaml` files in the `config/` folder.
