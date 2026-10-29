# Operation mode

## Related API

- {{ link_ad_api('/api/operation_mode/state') }}
- {{ link_ad_api('/api/operation_mode/change_to_autonomous') }}
- {{ link_ad_api('/api/operation_mode/change_to_stop') }}
- {{ link_ad_api('/api/operation_mode/change_to_local') }}
- {{ link_ad_api('/api/operation_mode/change_to_remote') }}
- {{ link_ad_api('/api/operation_mode/enable_autoware_control') }}
- {{ link_ad_api('/api/operation_mode/disable_autoware_control') }}

## Description

As shown below, Autoware assumes that the vehicle interface has two modes, Autoware control and direct control.
In direct control mode, the vehicle is operated using devices such as steering and pedals.
If the vehicle does not support direct control mode, it is always treated as Autoware control mode.
Autoware control mode has four operation modes.

| Mode       | Description                                                                   |
| ---------- | ----------------------------------------------------------------------------- |
| Stop       | Keep the vehicle stopped.                                                     |
| Autonomous | Autonomously control the vehicle.                                             |
| Local      | Manually control the vehicle from nearby with some device such as a joystick. |
| Remote     | Manually control the vehicle from a web application on the cloud.             |

{{ diagram("operation_mode/architecture.svg", "The operation mode selects the command under Autoware control. The driver controls the vehicle under direct control.") }}

## States

### Autoware control flag

The flag `is_autoware_control_enabled` indicates if the vehicle is controlled by Autoware.
The enable and disable APIs can be used if the control can be switched by software.
These APIs will always fail if the vehicle does not support mode switching or is switched by hardware.

### Operation mode and availability flags

The field `mode` indicates what command is used when Autoware control is enabled.
The flags `is_stop_mode_available`, `is_autonomous_mode_available`, `is_local_mode_available`, and `is_remote_mode_available` show if it is possible to transition to each mode.
A change request to a mode that is not available fails with `ERROR_NOT_AVAILABLE`.

In the default implementation, the availability comes from the diagnostic graph (the `/system/operation_mode/availability` topic).
For example, the autonomous mode is available only when the map, localization, planning (including the route), perception, control, vehicle, and system diagnostics are OK.
Use the [diagnostics API](diagnostics.md) to find the cause when a mode is not available.

### Transition flag

Autoware cannot always guarantee safety during a mode change, for example when it switches to autonomous mode during overspeed.
For this situation, the flag `is_in_transition` is true while the mode changes.
The operator who changed the mode must ensure safety while this flag is true. The flag becomes false when the mode change is complete.
If the mode change does not complete, Autoware returns to the previous mode.
