# /api/operation/mode/notice

- Method: notification
- Type: [autoware_ad_api_msgs/msg/OperationModeNotice](../../../../types/autoware_ad_api_msgs/msg/operation_mode_notice.md)

## Description

Get the operation mode. For details, see the [operation mode](./index.md).

## Message

| Name              | Type   | Description                                      |
| ----------------- | ------ | ------------------------------------------------ |
| selector_mode     | uint16 | The selected command for software control.       |
| is_driver_control | bool   | True if direct control by the driver is enabled. |
| is_in_transition  | bool   | True if the operation mode is in transition.     |
