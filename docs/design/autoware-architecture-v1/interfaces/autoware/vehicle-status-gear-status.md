---
architecture: autoware basic components
interface_type: topic
interface_name: /vehicle/status/gear_status
data_type: autoware_vehicle_msgs/msg/GearReport
updated: 2025-12-01
rate: N/A
qos_reliability: reliable
qos_durability: volatile / transient_local
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
- DRIVE_2などの定義や具体例。

## Errors

状態を取得できない場合はNONEを出力するため、Autowareで異常処理を行う必要がある。

## Support

このインターフェースは必須である。ギアが存在しない車両の場合は車両インターフェースで以下の条件を満たすように実装すること。

- PARKING: 車両が重力などの外からの力を受けても停止保持できる状態。コマンドが入力された場合でも停止を保持する。
- DRIVE: 前進するコマンドのみを受け付ける。もしくは後退コマンドが来たら自動的に切り替える。
- etc.

## Limitations

- 制限事項

## Requirement

車両の一般的なギア状態(P, N, D, R)を取得できる。
車両固有の独自のギア状態も取得できる。

## Prerequisites

特になし。

## Design

ギアが存在する車両については、取得したギア状態を送信する。
ギアがない車両については、処理の一般化のためにドライバでギアを再現する。
細かくギアが制御できる車両のためにユーザー定義領域を確保する？

## History

| Date       | Description |
| ---------- | ----------- |
| 2025-12-01 | Release.    |
