---
architecture: autoware components
interface_type: topic
interface_name: /autoware/example/topic
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

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

多くの場合、何らかの機能の
リンクを記載してこのインターフェースがどの部分を担当するか記載しても良いです。

- インターフェースの基本的な説明をここに記載する
- タイミングなどの仕様
- 対象となるODDによって考慮すべき項目(Rateなど)。
- 必要に応じて以下のようなセクションを追加する。
  - ステート遷移
  - シーケンス
  - データフロー
- アーキテクチャの方に機能の説明があってそれを参照しても良い

## Message

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

## Use Cases

- 要件を導出するためのユースケース。

## Requirement

実装者に向けた情報で、インターフェースを実装するときに満たすべき項目を記載する。
ここまでの利用者向けの説明の繰り返しになるかもしれないが、必須部分と任意実装が許される部分を明示的に分けて記載する。
安全に関する条件もあれば記載する。
設計を変更する際に維持すべき項目をチェックできる。

## Design

- 上記の要件や前提条件を考慮して何故この仕様になったのか意図を記載する。
- 考慮された他のデザインや選択理由など。再設計を検討する際にチェックできる。

## History

| Date       | Description |
| ---------- | ----------- |
| 2025-12-01 | Release.    |
