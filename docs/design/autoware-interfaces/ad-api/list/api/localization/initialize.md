# /api/localization/initialize

- Method: function call
- Type: [autoware_ad_api_msgs/srv/InitializeLocalization](../../../types/autoware_ad_api_msgs/srv/initialize_localization.md)

## Description

Request to initialize localization. For details, see the [pose state](./index.md).

## Request

| Name | Type                                             | Description                                                                 |
| ---- | ------------------------------------------------ | --------------------------------------------------------------------------- |
| pose | geometry_msgs/msg/PoseWithCovarianceStamped[<=1] | A global pose as the initial guess. If omitted, the GNSS pose will be used. |

## Response

| Name   | Type                                    | Description     |
| ------ | --------------------------------------- | --------------- |
| status | autoware_ad_api_msgs/msg/ResponseStatus | response status |
