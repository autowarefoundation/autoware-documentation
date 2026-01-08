---
architecture: autoware components
interface_type: topic
interface_name: /control/command/control_cmd
data_type: "[autoware_control_msgs/msg/Control](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Control.msg)"
rate: 33
qos_reliability: reliable
qos_durability: volatile
qos_depth: 1
last_updated: 2025-12-01
endpoints:
  control: pub
  vehicle: sub
---

過去に送ったコマンドはクリアされるべきです。

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Send the control command to the vehicle.

異なる時刻のコマンドで複数回送信した場合、過去の時刻の情報は記憶されず、常に最新の時刻の

時系列での制御を行いたい場合はControlHorizonを参照してください（提案中）


- インターフェースの基本的な説明をここに記載する
- タイミングなどの仕様
- 対象となるODDによって考慮すべき項目(Rateなど)。
- 必要に応じて以下のようなセクションを追加する。
  - ステート遷移
  - シーケンス
  - データフロー

## Message

For details about the message, [see the readme of autoware_control_msgs](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/README.md).

特定時刻における速度と加速度と躍度の値は事前に計画され、整合性を持っている必要があります。これは一般的にはplanning/controlコンポーネントが担当します。
もしこれらの値が整合しない場合、どの値を使用するかは車両の実装に依存します。


- メッセージの詳細を記載する。メッセージパッケージのREADMEへのリンクでも良い。
- 時刻やフレームの扱い
- 任意フィールドの扱い
- 無効値や範囲外の扱い（エラーになるのか無視されるのか）
- サポートしていない場合の挙動（空配列、NaN、トピックが出ないなど）

## Errors

- コマンドに応じて変化するステータストピックなど。
- サービスの場合はレスポンスで想定されるエラーの説明なども。

## Support

- インターフェースのサポートが必須かどうかや、段階的なサポートがあるかなど。
- インターフェースをサポートできない場合の対応方法や影響についても記載する。

## Limitations

- 制限事項

## Requirement

- 実装するときに満たすべき項目を記載する。
- 任意な実装が許される項目も明示的に記載する。

## Prerequisites

- このインタフェースが動作するための前提条件を記載する。
- 条件が満たされていない場合の通知手段や挙動などが記載してあると良い。
- 前提トピックについては実装依存の部分があるので扱いが難しい。

## Design

- 上記の要件や前提条件を考慮して何故この仕様になったのか意図を記載する。

## History

| Date       | Description |
| ---------- | ----------- |
| 2025-12-01 | Release.    |
