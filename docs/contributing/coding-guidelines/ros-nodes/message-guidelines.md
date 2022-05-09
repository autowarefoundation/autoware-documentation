# Message guidelines

## Format

All messages should follow [ROS message description specification](https://docs.ros.org/en/galactic/Concepts/About-ROS-Interfaces.html#background).

The accepted formats are:

- `.msg`
- `.srv`
- `.action`

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

For list of types supported by the ROS interfaces [see here](https://docs.ros.org/en/galactic/Concepts/About-ROS-Interfaces.html#field-types).

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

ROS2 interfaces don't support enumerations directly.

It is possible to define integers constants and assign them to a non-constant integer parameter.

!!! success ""

    Constants are written in `CONSTANT_CASE`.

!!! success ""

    Assign a different value to each element of a constant.

Example from [shape_msgs/msg/SolidPrimitive.msg](https://github.com/ros2/common_interfaces/blob/f3cb4848560e91596e7688e8ac1816828fa460cb/shape_msgs/msg/SolidPrimitive.msg#L4-L11)

```text
uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 CONE=4
uint8 PRISM=5

# The type of the shape
uint8 type
```

## Comments

On top of the message, briefly explain what the message contains and/or what it is used for. See [sensor_msgs/msg/Imu.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg#L1-L13) for an example.

!!! tip ""

    If necessary, add line comments before the fields that explain the context and/or meaning.

!!! tip ""

    For simple fields like `x, y, z, w` you might not need to add comments.

!!! success ""

    Even though it is not strictly checked, try not to pass 100 characters in a line.

_Example:_

```text
# Number of times the vehicle performed an emergency brake
uint32 count_emergency_brake

# Seconds passed since the last emergency brake
uint64 duration
```

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
