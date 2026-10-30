# Message guidelines

## Format

All messages should follow [ROS message description specification](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html#background).

The accepted formats are:

- `.msg`
- `.srv`
- `.action`

The prototype files are expected to comply with following style.

```text
#
# <File description>
#

# <Field description> (required or optional)
# e.g. <Example value>
type field


# <Constants description>
# <Item1 description>
uint16 CONSTANT_ITEM1 = 0
# <Item2 description>
uint16 CONSTANT_ITEM2 = 0


```

On top of the message, briefly explain what the message contains and/or what it is used for. For an example, see [sensor_msgs/msg/Imu.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg#L1-L13).

!!! success ""

    Even though it is not strictly checked, try not to pass 100 characters in a line.

_Example:_

```text
# Number of times the vehicle performed an emergency brake (required)
# e.g. 10
uint32 count_emergency_brake


# Speed limit on a specific lane (optional)
# If the value is 0.0, the lane has no specific speed limit, so common speed limit in the country is applied
# e.g. 30.0
# default: 0.0
float32 limit_kmph


```

If a specific field is not always required, it is expected be noted as `optional`.

- Basically, all field should be `required` and contain valid value.
- An `optional` field may not contain valid value, so the subscriber/client must check if it is valid or not. For such field, default value should be provided.

The user can learn about the message information by calling

```bash
ros2 interface show <message> --all-comments
```

Each README.md in the repository should only provide extra illustrative descriptions and external resources. In `.msg` files, refer to the URL of README along with the corresponding anchor.

_Example:_

```text
# please refer to https://github.com/autowarefoundation/autoware_msgs/blob/main/README.md#format for the illustrative description of this field
```

## Naming

ROS 2 sets some naming rules for interface files, and Autoware adds more. The examples come from [autoware_msgs](https://github.com/autowarefoundation/autoware_msgs/tree/1.13.0).

### Type names

- Write the name of a message, service, or action type in `PascalCase`. The file has the same name, for example `VelocityReport.msg`.
- Add the `Stamped` suffix to a type that adds a `std_msgs/Header` to another type. For example, `GnssInsOrientationStamped` holds a header and a `GnssInsOrientation`. This suffix is also used in [common_interfaces](https://github.com/ros2/common_interfaces), for example `geometry_msgs/msg/PoseStamped`.
- `autoware_vehicle_msgs` uses the `Command` suffix for a type that requests a state, and the `Report` suffix for a type that gives the current state. For example, `GearCommand` and `GearReport`. Use the same suffixes for new types of this kind.
- Most service types in autoware_msgs start with a verb, for example `ChangeOperationMode`, `ClearRoute`, and `SetLaneletRoute`.

### Plural types

Use `Array` as a suffix when creating a plural type of a message. This suffix is commonly used in [common_interfaces](https://github.com/ros2/common_interfaces).

_Example:_ `TrafficLightGroupArray` in `autoware_perception_msgs` holds a list of `TrafficLightGroup`.

Some existing messages use a plural noun instead, for example `DetectedObjects` and `PredictedObjects` in `autoware_perception_msgs`.

### Field and constant names

- Write field names in `snake_case`. ROS 2 allows only lowercase letters, digits, and underscores. A field name must start with a letter. It must not end with an underscore or contain two underscores in a row.
- Write constant names in `CONSTANT_CASE`. ROS 2 requires uppercase constant names. See [Enumerations](#enumerations).
- For a field with a unit, follow [Default units](#default-units) and [Non-default units](#non-default-units).

## Default units

All the fields by default have the following units depending on their types:

| type           | default unit  |
| -------------- | ------------- |
| distance       | meter (m)     |
| angle          | radians (rad) |
| time           | second (s)    |
| speed          | m/s           |
| velocity       | m/s           |
| acceleration   | m/s²          |
| angular vel.   | rad/s         |
| angular accel. | rad/s²        |

!!! warning ""

    If a field in a message has any of these default units, don't add any suffix or prefix denoting the type.

## Non-default units

For non-default units, use following suffixes:

| type     | non-default unit | suffix  |
| -------- | ---------------- | ------- |
| distance | nanometer        | `_nm`   |
| distance | micrometer       | `_um`   |
| distance | millimeter       | `_mm`   |
| distance | kilometer        | `_km`   |
| angle    | degree (deg)     | `_deg`  |
| time     | nanosecond       | `_ns`   |
| time     | microsecond      | `_us`   |
| time     | millisecond      | `_ms`   |
| time     | minute           | `_min`  |
| time     | hour (h)         | `_hour` |
| velocity | km/h             | `_kmph` |

!!! tip ""

    If a unit that you'd like to use doesn't exist here, [create an issue/PR](https://github.com/autowarefoundation/autoware-documentation/issues) to add it to this list.

## Message field types

For list of types supported by the ROS interfaces [see here](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html#field-types).

Also copied here for convenience:

| Message Field Type | C++ equivalent   |
| ------------------ | ---------------- |
| `bool`             | `bool`           |
| `byte`             | `uint8_t`        |
| `char`             | `char`           |
| `float32`          | `float`          |
| `float64`          | `double`         |
| `int8`             | `int8_t`         |
| `uint8`            | `uint8_t`        |
| `int16`            | `int16_t`        |
| `uint16`           | `uint16_t`       |
| `int32`            | `int32_t`        |
| `uint32`           | `uint32_t`       |
| `int64`            | `int64_t`        |
| `uint64`           | `uint64_t`       |
| `string`           | `std::string`    |
| `wstring`          | `std::u16string` |

### Arrays

For arrays, use `unbounded dynamic array` type.

Example:

```text
int32[] unbounded_integer_array
```

## Enumerations

ROS 2 interfaces don't support enumerations directly.

It is possible to define integers constants and assign them to a non-constant integer parameter.

!!! success ""

    Constants are written in `CONSTANT_CASE`.

!!! success ""

    Assign a different value to each element of a constant.

_Example:_

```text
# Classification of error states in Autoware Localization
# Initialization rejected due to unsafe conditions
uint16 ERROR_UNSAFE = 1
# GNSS-based initialization not supported
uint16 ERROR_GNSS_SUPPORT = 2
# GNSS initialization failed
uint16 ERROR_GNSS = 3
# Pose estimation failed
uint16 ERROR_ESTIMATION = 4

# The type of state (required)
# e.g. 1(ERROR_UNSAFE)
uint16 type
```

!!! tip ""

    The constants are expected to be mutually exclusive and collectively exhaustive in the domain. It leads to clear and less confusing modelling.

## Example usages

- Don't use unit suffixes for default types:
  - Bad: `float32 path_length_m`
  - Good: `float32 path_length`
- Don't prefix the units:
  - Bad: `float32 kmph_velocity_vehicle`
  - Good: `float32 velocity_vehicle_kmph`
- Use recommended suffixes [if they are available in the table](#non-default-units):
  - Bad: `float32 velocity_vehicle_km_h`
  - Good: `float32 velocity_vehicle_kmph`
