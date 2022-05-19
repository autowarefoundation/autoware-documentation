# /api/external/set/rosbag_logging_mode

## Classification

- Behavior: Service
- DataType: tier4_external_api_msgs/srv/SetRosbagLoggingMode

## Description

rosbag 記録モードを設定する。

| Mode          | is_operation_mode |
| ------------- | ----------------- |
| 常時記録      | false             |
| EM 発生時記録 | true              |

## Requirement

現在の rosbag 記録モードを「常時記録」または「EM 発生時記録」いずれかに設定し、rosbag 記録の保存先および保存対象の制御を行うこと。
