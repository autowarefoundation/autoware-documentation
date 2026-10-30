# Release notes

These notes list the changes of each AD API interface version. The version is the value that {{ link_ad_api('/api/interface/version') }} returns.
Autoware 2.0.0 provides v1.9.1. The default implementation sets this value in [`interface.cpp`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/api/autoware_default_adapi/src/interface.cpp) of the `autoware_default_adapi` package (Autoware Core 1.10.0).

The interface version is separate from the package version of [`autoware_adapi_msgs`](https://github.com/autowarefoundation/autoware_adapi_msgs). Autoware 2.0.0 uses `autoware_adapi_msgs` 1.9.2. This release only adds comments to the message files and does not change the interface.
An API with the status "not released" in the [list of APIs](list/index.md) is not part of any interface version yet, even if the default implementation already provides it.

## v1.9.1

- [Change] Add roundabout as [Planning Behavior](features/planning-factors.md)

## v1.9.0

- [New] Add [control command API](features/control.md)
- [New] Add {{ link_ad_api('/api/vehicle/metrics') }}
- [New] Add {{ link_ad_api('/api/system/diagnostics/reset') }}
- [New] Add {{ link_ad_api('/api/fail_safe/list_mrm_description') }}
- [New] Add {{ link_ad_api('/api/fail_safe/mrm_request/list') }}
- [New] Add {{ link_ad_api('/api/fail_safe/mrm_request/send') }}
- [Change] Support more detailed status in [diagnostics API](features/diagnostics.md)
- [Change] Deprecate the MRM behavior constants in [fail-safe API](features/fail-safe.md)

## v1.8.0

- [New] Add [manual control API](features/manual-control.md)

## v1.7.0

- [Chore] Change the messages for the prototype implementation.

## v1.6.0

- [Change] Fix communication method of {{ link_ad_api('/api/vehicle/status') }}
- [Change] Add restrictions to {{ link_ad_api('/api/routing/clear_route') }}
- [Change] Add restrictions to {{ link_ad_api('/api/vehicle/doors/command') }}

## v1.5.0

- [New] Add {{ link_ad_api('/api/routing/change_route_points') }}
- [New] Add {{ link_ad_api('/api/routing/change_route') }}

## v1.4.0

- [New] Add {{ link_ad_api('/api/vehicle/status') }}

## v1.3.0

- [New] Add [heartbeat API](features/heartbeat.md)
- [New] Add [diagnostics API](features/diagnostics.md)

## v1.2.0

- [New] Add [vehicle doors API](features/vehicle-doors.md)
- [Change] Add pull-over constants to {{ link_ad_api('/api/fail_safe/mrm_state') }}

## v1.1.0

- [New] Add {{ link_ad_api('/api/fail_safe/mrm_state') }}
- [New] Add {{ link_ad_api('/api/vehicle/dimensions') }}
- [New] Add {{ link_ad_api('/api/vehicle/kinematics') }}
- [Change] Add options to [the routing API](features/routing.md)

## v1.0.0

- [New] Add [interface API](features/interface.md)
- [New] Add [localization API](features/localization.md)
- [New] Add [routing API](features/routing.md)
- [New] Add [operation mode API](features/operation_mode.md)
