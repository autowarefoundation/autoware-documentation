# /api/fail_safe/mrm_state

- Method: notification
- Type: [autoware_ad_api_msgs/msg/MrmState](../../../types/autoware_ad_api_msgs/msg/mrm_state.md)

## Description

Get the MRM state. For details, see the [fail-safe](./index.md).

## Message

| Name     | Type   | Description                             |
| -------- | ------ | --------------------------------------- |
| state    | uint16 | The state of MRM operation.             |
| behavior | uint16 | The currently selected behavior of MRM. |
