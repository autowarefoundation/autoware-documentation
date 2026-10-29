# Manual control

## Description

An interface to drive a vehicle manually through Autoware is necessary for a vehicle without a steering wheel or pedals, and for the development of a remote operation system.

## Requirements

- The vehicle must be able to transition to a safe state when a communication problem occurs.
- AD API must support the following commands and statuses.
  - Pedals, acceleration, or velocity as longitudinal control
  - Steering tire angle as lateral control
  - Gear
  - Turn indicators
  - Hazard lights
- The following are under consideration.
  - Headlights
  - Wipers
  - Parking brake
  - Horn

## Sequence

The diagram shows remote control. Local control uses the same sequence with the `/api/manual/local/*` APIs and {{ link_ad_api('/api/operation_mode/change_to_local') }}.

{{ diagram("sequence.svg", "Select a control mode, change to the remote mode, then send commands while the vehicle status comes back.") }}

<div class="aw-steps" markdown>

1. **Select the control mode.** Call {{ link_ad_api('/api/manual/remote/control_mode/select') }}. This request fails while the operation mode is already remote. The default implementation supports only the `PEDALS` mode.
2. **Change the operation mode.** Call {{ link_ad_api('/api/operation_mode/change_to_remote') }}.
3. **Drive.** Send {{ link_ad_api('/api/manual/remote/operator/heartbeat') }} and the commands of the selected mode periodically. Watch {{ link_ad_api('/api/vehicle/status') }} for the result.

</div>

## Related features

- [Manual control](../../features/manual-control.md)
- [Operation mode](../../features/operation_mode.md)
- [Vehicle status](../../features/vehicle-status.md)
