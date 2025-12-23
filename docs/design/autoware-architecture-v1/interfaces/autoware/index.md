# Autoware Architecture

## インターフェースページ雛形

- [topic](../templates/topic.md)
- [service](../templates/service.md)

## インターフェースページ一覧

- [/control/control_mode_request](./control-control-mode-request.md)
- [/control/command/control_cmd](./control-command-control-cmd.md)
- [/control/command/gear_cmd](./control-command-gear-cmd.md)
- [/control/command/turn_indicators_cmd](./control-command-turn-indicators-cmd.md)
- [/control/command/hazard_lights_cmd](./control-command-hazard-lights-cmd.md)
- [/vehicle/status/control_mode](./vehicle-status-contro-mode.md)
- [/vehicle/status/velocity_status](./vehicle-status-velocity-status.md)
- [/vehicle/status/steering_status](./vehicle-status-steering-status.md)
- [/vehicle/status/gear_status](./vehicle-status-gear-status.md)
- [/vehicle/status/turn_indicators_status](./vehicle-status-turn-indicators-status.md)
- [/vehicle/status/hazard_lights_status](./vehicle-status-hazard-lights-status.md)
- [/vehicle/status/actuation_status](./vehicle-status-actuation-status.md)
- [/vehicle/doors/command](./vehicle-doors-command.md)
- [/vehicle/doors/layout](./vehicle-doors-layout.md)
- [/vehicle/doors/status](./vehicle-doors-status.md)

## アーキテクチャの適用方法

1. 最上位レベルのアーキテクチャを選択する
2. 選択したアーキテクチャの各コンポーネントについて、内部アーキテクチャを選択する
3. 上記をアーキテクチャが細分化できなくなるまで繰り返す

- 特定のモジュールはサブレベルの「モジュール配置」と「モジュール定義」を内包できる。
- 複数のモジュール配置で使い回せるモジュール定義がある場合も想定する。
- 互換性は低下するが複数モジュールの組み合わせを単一の複合モジュールとして扱っても良い。
