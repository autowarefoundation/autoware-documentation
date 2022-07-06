# /api/operation/mode/state

- Method: notification
- Type: [autoware_ad_api_msgs/msg/OperationModeState](../../../../types/autoware_ad_api_msgs/msg/operation_mode_state.md)

## Description

Get the operation mode state. For details, see the [operation mode](./index.md).

## Message

| Name                   | Type   | Description                                              |
| ---------------------- | ------ | -------------------------------------------------------- |
| operation_mode         | uint16 | The selected command for Autoware control.               |
| is_in_autoware_control | bool   | True if direct control by the driver is enabled.         |
| is_in_transition       | bool   | True if the operation mode is in transition.             |
| change_to_stop         | bool   | True if the operation mode can be changed to stop.       |
| change_to_autonomous   | bool   | True if the operation mode can be changed to autonomous. |
| change_to_local        | bool   | True if the operation mode can be changed to local.      |
| change_to_remote       | bool   | True if the operation mode can be changed to remote.     |
