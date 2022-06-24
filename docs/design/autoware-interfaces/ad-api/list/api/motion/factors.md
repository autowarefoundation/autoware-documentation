# /api/motion/factors

- Method: realtime stream
- Type: [autoware_ad_api_msgs/msg/MotionFactorArray](../../../types/autoware_ad_api_msgs/msg/motion_factor_array.md)

## Description

Get the motion factors. They are sorted in ascending order of distance.
For details, see the [motion factors](./index.md).

## Message

| Name             | Type                   | Description                                      |
| ---------------- | ---------------------- | ------------------------------------------------ |
| factors.pose     | geometry_msgs/msg/Pose | The pose related to the motion factor.           |
| factors.distance | float32                | The distance from the vehicle to the above pose. |
| factors.reason   | uint16                 | The reason of the motion factor.                 |
| factors.status   | uint16                 | The status of the motion factor.                 |
| factors.detail   | string                 | The additional information of the motion factor. |
