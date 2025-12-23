---
architecture: autoware components
interface_type: topic
interface_name: /control/control_mode_request
data_type: foo_msgs/srv/Service
updated: 2025-12-01
timeout: ---
endpoints:
  vehicle: srv
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/service.jinja2' %}

## Description

- インターフェースの基本的な説明をここに記載する
- タイミングなどの仕様
- 対象となるODDによって考慮すべき項目(Rateなど)。
- 必要に応じて以下のようなセクションを追加する。
  - ステート遷移
  - シーケンス
  - データフロー

## Request

- メッセージの詳細を記載する。メッセージパッケージのREADMEへのリンクでも良い。
- 時刻やフレームの扱い
- 任意フィールドの扱い
- 無効値や範囲外の扱い（エラーになるのか無視されるのか）
- サポートしていない場合の挙動（空配列、NaN、トピックが出ないなど）

## Response

- Requestと同様
- エラー時の挙動や戻り値など

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
