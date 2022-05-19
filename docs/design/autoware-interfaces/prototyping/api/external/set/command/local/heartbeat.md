# /api/external/set/command/local/heartbeat

## Classification

- Behavior: Topic
- DataType: tier4_external_api_msgs/msg/Heartbeat

## Description

車両との通信状態を確認するための信号を送信する。

## Requirement

この信号が途切れた場合、車両は監視されていない状態になったとして適切な制御を行うこと。
