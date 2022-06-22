# /api/pose/initialize

- Method: function call
- Type: [autoware_ad_api_msgs/srv/PoseInitialize](../../../types/autoware_ad_api_msgs/srv/pose_initialize.md)

## Description

Request to initialize the vehicle pose. For details, see the [pose state](./index.md).

## Request

| Name | Type                                        | Description                                                                    |
| ---- | ------------------------------------------- | ------------------------------------------------------------------------------ |
| pose | geometry_msgs/msg/PoseWithCovarianceStamped | The initial guess for pose estimation. If omitted, the GNSS pose will be used. |

## Response

| Name   | Type                                    | Description     |
| ------ | --------------------------------------- | --------------- |
| status | autoware_ad_api_msgs/msg/ResponseStatus | response status |
