# /api/operation/mode/change

- Method: function call
- Type: [autoware_ad_api_msgs/srv/ChangeOperationMode](../../../../types/autoware_ad_api_msgs/srv/change_operation_mode.md)

## Description

Request to change the operation mode. For details, see the [operation mode](./index.md).
This API fail if the vehicle does not support mode change by software.

## Request

| Name | Type   | Description    |
| ---- | ------ | -------------- |
| mode | uint16 | operation mode |

## Response

| Name   | Type                                    | Description     |
| ------ | --------------------------------------- | --------------- |
| status | autoware_ad_api_msgs/msg/ResponseStatus | response status |
