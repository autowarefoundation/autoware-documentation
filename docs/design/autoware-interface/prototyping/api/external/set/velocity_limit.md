# /api/external/set/velocity_limit

## Classification

- Behavior: Service
- DataType: tier4_external_api_msgs/srv/SetVelocityLimit

## Description

車両の制限速度を設定する。

## Requirement

指定された制限速度以下になるように車両の制御を行うこと。現在の速度が制限速度を超えている場合、車両の状態を考慮して適切に制限速度以下になるような制御を行うこと。
