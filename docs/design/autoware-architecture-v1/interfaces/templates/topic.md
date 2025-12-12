---
architecture: autoware basic components
interface_type: topic
interface_name: /aaa/bbb/ccc
data_type: foo_msgs/msg/Message
updated: 2025-12-01
rate: 10~20
qos_reliability: reliable
qos_durability: volatile
qos_depth: 1
endpoints:
  localization: pub
  planning: sub
  perception: sub
---

# /aaa/bbb/ccc

## Specifications

{% include 'design/architecture/templates/topic.jinja2' %}

## Description

- インターフェースの基本的な説明をここに記載する
- タイミングなどの仕様
- 対象となるODDによって考慮すべき項目(Rateなど)。
- 必要に応じて以下のようなセクションを追加する。
  - ステート遷移
  - シーケンス
  - データフロー

## Message

- 主にトピックの場合、ここにメッセージの詳細を記載する。
- サポートしていない場合（空配列、NaN、トピックが出ないなど）
- 無効値や範囲外の扱い（エラーになるのか無視されるのか）

## Request

- 主にトピックの場合、ここにリクエストの詳細を記載する。

## Response

- 主にトピックの場合、ここにレスポンスの詳細を記載する。

## Errors

- コマンドに応じて変化するステータストピックなど。
- サービスの場合はレスポンスで想定されるエラーの説明なども。

## Support

- このインターフェースをサポートできない場合の対応方法について記載する。
- サポートしない場合の影響についても記載する。

## Limitations

- 制限事項

## Requirement

- 実装するときに満たすべき項目を記載する。
- 任意な実装が許される項目も明示的に記載する。

## Prerequisites

- このインタフェースが動作するための前提条件を記載する。
- 条件が満たされていない場合は動作を保証しなくても良い。
- 条件が満たされていない場合の通知手段や挙動などが記載してあると良い。
- 前提トピックについては実装依存の部分があるので扱いが難しい。

## Design

- 上記の要件や前提条件を考慮して何故この仕様になったのか意図を記載する。

## History

| Date       | Description |
| ---------- | ----------- |
| 2025-12-01 | Release.    |
