# /api/external/set/command/local/control

## Classification

- Behavior: Topic
- DataType: tier4_external_api_msgs/msg/ControlCommandStamped

## Description

車両のアクセル・ブレーキ・ステアリングを制御するコマンドを送信する。

## Requirement

現在の車両状態を考慮し、指定されたコマンドを可能な限り反映した制御を行うこと。
