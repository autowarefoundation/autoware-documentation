# Manual control

## Related API

- {{ link_ad_api('/api/remote/control_mode/select') }}
- {{ link_ad_api('/api/remote/control_mode/status') }}
- {{ link_ad_api('/api/remote/command/pedal') }}
- {{ link_ad_api('/api/remote/command/accel') }}
- {{ link_ad_api('/api/remote/command/steer') }}
- {{ link_ad_api('/api/remote/command/gear') }}
- {{ link_ad_api('/api/remote/command/turn_indicators') }}
- {{ link_ad_api('/api/remote/command/hazard_lights') }}
- {{ link_ad_api('/api/local/control_mode/select') }}
- {{ link_ad_api('/api/local/control_mode/status') }}
- {{ link_ad_api('/api/local/command/pedal') }}
- {{ link_ad_api('/api/local/command/accel') }}
- {{ link_ad_api('/api/local/command/steer') }}
- {{ link_ad_api('/api/local/command/gear') }}
- {{ link_ad_api('/api/local/command/turn_indicators') }}
- {{ link_ad_api('/api/local/command/hazard_lights') }}

## Description

This API is used to manually control the vehicle, and provides the same interface for different operators: remote and local.
For example, the local operator controls a vehicle without a driver's seat using a joystick, while the remote operator provides remote support when problems occur with autonomous driving.
Since there are multiple ways to control a vehicle, such as pedals or acceleration, the application must first select a control mode.

<!-- The control modes supported by the vehicle can be retrieved using the list API. -->
