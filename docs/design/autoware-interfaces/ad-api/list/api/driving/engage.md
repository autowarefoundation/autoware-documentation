# /api/driving/engage

- Method: function call
- Type: [autoware_ad_api_msgs/srv/DrivingEngage](../../../types/autoware_ad_api_msgs/srv/driving_engage.md)

## Description

Engage or disengage the vehicle. For details, see the [driving state](index.md).

## Request

| Name   | Type | Description                             |
| ------ | ---- | --------------------------------------- |
| engage | bool | Set true to engage, false to disengage. |

## Response

| Name   | Type                                    | Description     |
| ------ | --------------------------------------- | --------------- |
| status | autoware_ad_api_msgs/msg/ResponseStatus | response status |
