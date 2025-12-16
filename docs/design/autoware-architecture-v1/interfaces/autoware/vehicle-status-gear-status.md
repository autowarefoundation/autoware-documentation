---
architecture: autoware basic components
interface_type: topic
interface_name: /vehicle/status/gear_status
data_type: autoware_vehicle_msgs/msg/GearReport
updated: 2025-12-01
rate: N/A
qos_reliability: reliable
qos_durability: volatile / transient_lopcal
qos_depth: 1
endpoints:
  vehicle: pub
  control: sub
  adapi: sub
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

車両のギア状態を取得する。変化時のみ通知することが望ましいが、現状多くの実装は周期的に送信することを前提に作られている。
そのため、通知方式で実装する場合はシステム全体で対応が行われているかの確認が必要になる点に注意すること。

## Message

メッセージの詳細は[autoware_vehicle_msgs/msg/GearReport](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/GearReport.msg)を参照すること。

まだ詳細な記載がないので以下のような説明がほしい。
- stampはメッセージの送信時刻 or 状態取得時の車両時刻 or 最後に状態が切り替えられた時刻である。


- メッセージの詳細を記載する。メッセージパッケージのREADMEへのリンクでも良い。
- 時刻やフレームの扱い
- 任意フィールドの扱い
- 無効値や範囲外の扱い（エラーになるのか無視されるのか）
- サポートしていない場合の挙動（空配列、NaN、トピックが出ないなど）

## Errors

状態を取得できない場合はNONEを出力するため、Autowareで異常処理を行う必要がある。

## Support

このインターフェースは必須である。ギアが存在しない車両の場合は車両インターフェースで以下の条件を満たすように実装すること。
- PARKING: 車両が重力などの外からの力を受けても停止保持できる状態。コマンドが入力された場合でも停止を保持する。
- DRIVE: 前進するコマンドのみを受け付ける。もしくは後退コマンドが来たら自動的に切り替える。

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
