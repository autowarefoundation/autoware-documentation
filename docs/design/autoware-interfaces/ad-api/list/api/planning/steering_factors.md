# /api/planning/steering_factors

- Method: realtime stream
- Type: [autoware_ad_api_msgs/msg/SteeringFactorArray](../../../types/autoware_ad_api_msgs/msg/steering_factor_array.md)

## Description

Get the steering factors, sorted in ascending order of distance.
For details, see the [planning](./index.md).

## Message

| Name              | Type                   | Description                                           |
| ----------------- | ---------------------- | ----------------------------------------------------- |
| factors.pose      | geometry_msgs/msg/Pose | The pose related to the steering factor.              |
| factors.distance  | float32                | The distance from the vehicle head to the above pose. |
| factors.type      | uint16                 | The type of the steering factor.                      |
| factors.direction | uint16                 | The direction of the steering factor.                 |
| factors.status    | uint16                 | The status of the steering factor.                    |
| factors.detail    | string                 | The additional information of the steering factor.    |
