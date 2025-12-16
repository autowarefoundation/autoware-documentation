# Autoware Architecture

## インターフェースページ雛形

- [topic](../templates/topic.md)

## インターフェースページ一覧

- /control/control_mode_request
- /vehicle/status/control_mode
- [/vehicle/status/gear_status](./vehicle-status-gear-status.md)

## アーキテクチャの適用方法

1. 最上位レベルのアーキテクチャを選択する
2. 選択したアーキテクチャの各コンポーネントについて、内部アーキテクチャを選択する
3. 上記をアーキテクチャが細分化できなくなるまで繰り返す

- 特定のモジュールはサブレベルの「モジュール配置」と「モジュール定義」を内包できる。
- 複数のモジュール配置で使い回せるモジュール定義がある場合も想定する。
- 互換性は低下するが複数モジュールの組み合わせを単一の複合モジュールとして扱っても良い。
