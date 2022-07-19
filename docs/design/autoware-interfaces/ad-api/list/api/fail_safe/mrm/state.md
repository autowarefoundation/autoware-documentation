# /api/fail_safe/mrm/state

- Method: notification
- Type: [autoware_ad_api_msgs/msg/FailSafeState](../../../../types/autoware_ad_api_msgs/msg/fail_safe_state.md)

## Description

Get the fail-safe state. For details, see the [fail-safe](../index.md).

## Message

| Name     | Type   | Description                             |
| -------- | ------ | --------------------------------------- |
| state    | uint16 | The state of MRM operation.             |
| behavior | uint16 | The currently selected behavior of MRM. |
