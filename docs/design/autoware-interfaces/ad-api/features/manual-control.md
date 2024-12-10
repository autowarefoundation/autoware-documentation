# Manual control

## Related API

- {{ link_ad_api('/api/remote/control_mode/list') }}
- {{ link_ad_api('/api/remote/control_mode/select') }}
- {{ link_ad_api('/api/remote/control_mode/status') }}
- {{ link_ad_api('/api/remote/operator/status') }}
- {{ link_ad_api('/api/remote/command/pedal') }}
- {{ link_ad_api('/api/remote/command/acceleration') }}
- {{ link_ad_api('/api/remote/command/velocity') }}
- {{ link_ad_api('/api/remote/command/steering') }}
- {{ link_ad_api('/api/remote/command/gear') }}
- {{ link_ad_api('/api/remote/command/turn_indicators') }}
- {{ link_ad_api('/api/remote/command/hazard_lights') }}
- {{ link_ad_api('/api/local/control_mode/list') }}
- {{ link_ad_api('/api/local/control_mode/select') }}
- {{ link_ad_api('/api/local/control_mode/status') }}
- {{ link_ad_api('/api/local/operator/status') }}
- {{ link_ad_api('/api/local/command/pedal') }}
- {{ link_ad_api('/api/local/command/acceleration') }}
- {{ link_ad_api('/api/local/command/velocity') }}
- {{ link_ad_api('/api/local/command/steering') }}
- {{ link_ad_api('/api/local/command/gear') }}
- {{ link_ad_api('/api/local/command/turn_indicators') }}
- {{ link_ad_api('/api/local/command/hazard_lights') }}

## Description

This API is used to manually control the vehicle, and provides the same interface for different operators: remote and local.
For example, the local operator controls a vehicle without a driver's seat using a joystick, while the remote operator provides remote support when problems occur with autonomous driving.
The command sent will be used when [operation mode](./operation_mode.md) is remote or local.

## Operator status

The application needs to determine whether the operator is able to drive and send that information via the operator status API.
If the operator is unable to continue driving during manual operation, Autoware will perform MRM to bring the vehicle to a safe state.
For level 3 and below, the operator status is referenced even during autonomous driving.

## Control mode

Since there are multiple ways to control a vehicle, such as pedals or acceleration, the application must first select a control mode.

| Mode         | Description                                                                |
| ------------ | -------------------------------------------------------------------------- |
| disabled     | This is the initial mode. When selected, all command APIs are unavailable. |
| pedal        | This mode provides longitudinal control using the pedals.                  |
| acceleration | This mode provides longitudinal control using the target acceleration.     |
| velocity     | This mode provides longitudinal control using the target velocity.         |

## Commands

The commands available in each mode are as follows.

| Command         | disabled |  pedal  | acceleration | velocity |
| --------------- | :------: | :-----: | :----------: | :------: |
| pedal           |    -     | &check; |      -       |    -     |
| acceleration    |    -     |    -    |   &check;    |    -     |
| velocity        |    -     |    -    |      -       | &check;  |
| steering        |    -     | &check; |   &check;    | &check;  |
| gear            |    -     | &check; |   &check;    | &check;  |
| turn_indicators |    -     | &check; |   &check;    | &check;  |
| hazard_lights   |    -     | &check; |   &check;    | &check;  |
