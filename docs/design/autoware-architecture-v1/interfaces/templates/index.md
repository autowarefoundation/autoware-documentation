# Autoware Architecture

## インターフェースページ雛形

- [topic](./topic.md)

## インターフェースページ一覧

- [/vehicle/status/gear_status](./vehicle-status-gear-status.md)

## アーキテクチャの適用方法

1. 最上位レベルのアーキテクチャを選択する
2. 選択したアーキテクチャの各コンポーネントについて、内部アーキテクチャを選択する
3. 上記をアーキテクチャが細分化できなくなるまで繰り返す

- 複数のタイプで使い回せるモジュールがある場合がある。
- 特定のモジュールはサブレベルの「モジュール配置」と「モジュール定義」がある。
- 互換性は低下するが複数モジュールの組み合わせを単一の複合モジュールとして扱っても良い。

## アーキテクチャの階層

![monolithic](./images/monolithic.drawio.svg)

![components](./images/components.drawio.svg)
