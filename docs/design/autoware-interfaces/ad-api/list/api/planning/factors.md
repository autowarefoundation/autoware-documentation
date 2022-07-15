# /api/planning/factors

- Method: realtime stream
- Type: [autoware_ad_api_msgs/msg/PlanningFactorArray](../../../types/autoware_ad_api_msgs/msg/planning_factor_array.md)

## Description

Get the planning factors. They are sorted in ascending order of distance.
For details, see the [planning factors](./index.md).

## Message

| Name             | Type                   | Description                                           |
| ---------------- | ---------------------- | ----------------------------------------------------- |
| factors.pose     | geometry_msgs/msg/Pose | The pose related to the planning factor.              |
| factors.distance | float32                | The distance from the vehicle head to the above pose. |
| factors.type     | uint16                 | The type of the planning factor.                      |
| factors.status   | uint16                 | The status of the planning factor.                    |
| factors.detail   | string                 | The additional information of the planning factor.    |
