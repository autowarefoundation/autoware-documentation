# /api/fail_safe/state

- Method: notification
- Type: [autoware_ad_api_msgs/msg/FailSafeState](../../../types/autoware_ad_api_msgs/msg/fail_safe_state.md)

## Description

Get the fail safe state. For details, see the [fail safe state](./index.md).

## Message

| Name          | Type   | Description                                         |
| ------------- | ------ | --------------------------------------------------- |
| state         | uint16 | A value of the [fail safe state](./index.md).       |
| auto_recovery | bool   | Whether the fail safe state recovers automatically. |
