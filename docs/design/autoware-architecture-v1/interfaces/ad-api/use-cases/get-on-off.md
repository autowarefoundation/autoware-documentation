# Get on and get off

When the vehicle stops at a bus stop or a pickup point, the application opens the doors for the passengers and closes them before the vehicle starts again. This use case needs a vehicle that has a software interface for the doors.

## Related API

- [Vehicle doors](../features/vehicle-doors.md)

## Sequence

{{ diagram("sequence/get-on-off.svg", "Open the doors for the passengers, then close them.") }}

<div class="aw-steps" markdown>

1. **Find the doors.** Call {{ link_ad_api('/api/vehicle/doors/layout') }}. Use the `roles` field to find the doors for getting on (`GET_ON`) and getting off (`GET_OFF`).
2. **Open the doors.** Call {{ link_ad_api('/api/vehicle/doors/command') }} with the `OPEN` command for these doors.
3. **Wait for the passengers.** Watch {{ link_ad_api('/api/vehicle/doors/status') }} and your own passenger checks.
4. **Close the doors.** Call {{ link_ad_api('/api/vehicle/doors/command') }} with the `CLOSE` command. Wait until the status of all doors is `CLOSED`.

</div>

## Restrictions

The default implementation (`vehicle_door` node of `autoware_default_adapi_universe`) rejects a door command in these cases:

- The command opens a door and the operation mode is not stop.
- Autoware control is disabled. Set the `check_autoware_control` parameter to `false` to allow door commands during direct control.
