# /api/external/get/rosbag_logging_mode

## Classification

- Behavior: Topic
- DataType: tier4_external_api_msgs/msg/RosbagLoggingMode

## Description

rosbag 記録モードを取得する。

| Mode          | is_operation_mode |
| ------------- | ----------------- |
| 常時記録      | false             |
| EM 発生時記録 | true              |

## Requirement

現在設定されている「常時記録」または「EM 発生時記録」いずれかの rosbag 記録モードを提供すること。
