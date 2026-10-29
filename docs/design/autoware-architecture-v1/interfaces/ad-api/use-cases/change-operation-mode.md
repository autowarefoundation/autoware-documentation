# Change the operation mode

The operator changes the operation mode, for example from stop to autonomous, or enables and disables Autoware control. A software switch in the application uses the operation mode API. A hardware switch on the vehicle changes the control mode directly, and AD API only reports the result.

## Related API

- [Operation mode](../features/operation_mode.md)

## Sequence

### Change the mode with a software switch

The application calls one of the `/api/operation_mode/change_to_*` APIs, or {{ link_ad_api('/api/operation_mode/enable_autoware_control') }} and {{ link_ad_api('/api/operation_mode/disable_autoware_control') }}. The request fails if the target mode is not available. Check the `is_*_mode_available` flags of {{ link_ad_api('/api/operation_mode/state') }} before the request.

{{ diagram("sequence/operation-mode-software.svg", "Change the mode with a software switch in the application.") }}

### Change the mode with a hardware switch

The operator uses a switch on the vehicle. The enable and disable APIs always fail on a vehicle that changes the control mode only by hardware.

{{ diagram("sequence/operation-mode-hardware.svg", "Change the mode with a hardware switch on the vehicle.") }}
