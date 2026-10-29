# Control

## Related API

- {{ link_ad_api('/api/control/command/acceleration') }}
- {{ link_ad_api('/api/control/command/pedals') }}
- {{ link_ad_api('/api/control/command/steering') }}
- {{ link_ad_api('/api/control/command/velocity') }}

## Commands

The control commands are target values that Autoware is sending to the vehicle. Note that unlike vehicle status, this does not represent the actual vehicle status.
Some commands are not available, depending on the vehicle control method.
For example, the pedals command is available only if the vehicle is controlled by pedals.

The `vehicle_command` node of `autoware_default_adapi_universe` publishes these commands. It reads the control command (`/control/command/control_cmd`) and the actuation command (`/control/command/actuation_cmd`) that Autoware sends to the vehicle interface.
